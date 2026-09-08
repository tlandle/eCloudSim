#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# License: TDG-Attribution-NonCommercial-NoDistrib
"""Offline association-trial harness for the Khonsu migration import (schema row 59).

OFFLINE. This is NOT a CARLA campaign. It exercises the destination edge's
migration-import association logic directly with synthetic inputs and runs in
seconds. No CarlaUE4, no ecav.py, no GPU, no orchestration server.

What is under test
------------------
The destination-side association function the closed-loop obstacle import runs.
When the migration daemon calls ``dst_edge.import_tracked_obstacle_state`` the
carried KF snapshot is injected as a warm track
(``AB3DMOTStateTransferMixin._inject_warm_kf``); on the next tracker tick that
warm track is reconciled against the destination's own re-detections by
``AB3DMOT_libs.matching.data_association`` (which calls ``compute_affinity``).
``data_association`` holds BOTH branches this figure varies:

  * stable-id merge  -- compute_affinity forces a match (affinity -0.01) when a
    detection's carried identity (info[CID]) equals the track's carla_id and the
    track's anchoring epoch is still valid; a disagreeing identity is pushed to
    COST_MAX.
  * position gate    -- the geometric distance metric (dist_3d for the Car
    config) plus the affinity threshold and the kinematic innovation / static
    gates in data_association.

The harness calls this PRODUCTION function UNCHANGED. It builds the imported
track through the real import injection (_inject_warm_kf) and the real tracker
prediction() step, builds the local detections through the real
tracker.process_dets(), and then calls the real data_association() with the
destination tracker's own metric / threshold / algorithm / anchoring settings.
The synthetic imported track, distractor detections, and carried-pose error are
injected only at that function's inputs. The gate logic is not reimplemented.

Two modes, one function
-----------------------
The mode switch is purely which identity the destination re-detections carry,
which is exactly the field data_association keys on:

  * assoc_mode=stable_id -- the true re-detection carries the object's identity
    (info[CID] == imported track carla_id); the imported track's carried
    identity binds it to the correct detection regardless of pose error.
    Distractors are anonymous local detections (CID=-1), the realistic clutter
    case (unlabelled LiDAR/camera returns).
  * assoc_mode=geometry  -- no re-detection carries identity (all CID=-1), so
    association is position-nearest under the distance metric and the position
    gate. This is what today's closed loop does (see the note below).

Grid: assoc_mode x distractors {0,1,2} x pose_err_m {0.0,0.5,1.0,2.0,3.5} x 40
reps. RNG seed = global trial index (deterministic).

Fidelity notes (reported, not worked around)
--------------------------------------------
1. compute_affinity reads ``det.info[CID]`` but the tracker's process_dets sets
   ``det.carla_id`` and does NOT set ``det.info``. So in the current closed loop
   det_cid is always -1 and the stable-id branch is dormant (association is
   pure geometry). To exercise the stable-id merge the harness attaches the
   (frame,guid,cid) info row to the detection, which is the exact input
   compute_affinity reads. geometry mode matches production as-is.
2. compute_affinity's disagreeing-identity "forbid" sets affinity to
   COST_MAX = +1e3, but data_association maximizes affinity, so a distractor
   carrying a conflicting identity would be preferred, not forbidden. This
   harness models distractors as anonymous (CID=-1), so the inversion does not
   affect these results; it is flagged for the record.
"""
from __future__ import annotations

import argparse
import csv
import os
import subprocess
import sys

import numpy as np

# ── repo root on path (script lives in scripts/) ────────────────────────
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from AB3DMOT_libs.matching import CID, FRAME_IDX, GUID, data_association
from ecav.core.application.edge.edge_manager.ab3dmot_state_transfer import (
    AB3DMOTStateTransferMixin,
)
from ecav.core.application.edge.migration.payload import KFState
from ecav.core.tracking import get_tracker

# ── figure configuration ────────────────────────────────────────────────
ASSOC_FN = "AB3DMOT_libs.matching.data_association"
EVAL_TAG = "khonsu-eval-freeze-1l"
DEFAULT_OUT = "docs/kb/data/relay_eval_2026_08/frozen1l_assoc_rows.csv"

GRID_MODES = ["stable_id", "geometry"]
GRID_DISTRACTORS = [0, 1, 2]
GRID_POSE_ERR = [0.0, 0.5, 1.0, 2.0, 3.5]
REPS = 40

# True object pose in destination-locale ground coordinates (x >= 240 is the
# destination-side region used throughout the freeze-1 eval). Height (y) is
# constant; association is on the ground plane (KITTI x,z = CARLA x,y).
TRUE_X, TRUE_Y, TRUE_Z = 240.0, 1.0, 10.0
# Nearby-vehicle spacing for distractor clutter (annulus, metres).
DISTRACTOR_R_MIN, DISTRACTOR_R_MAX = 1.5, 4.0
# Imported obstacle box dimensions [l, w, h] (a passenger car).
BOX_L, BOX_W, BOX_H = 4.0, 1.8, 1.5
# Per-tick ground velocity of the imported track. Non-zero keeps kf_speed_ground
# above the static-track threshold so the standard kinematic gate applies rather
# than the tighter stationary gate. Pre-subtracted from the injected state so the
# post-prediction() box lands exactly at the carried (noisy) pose.
IMPORT_VX = 0.6
OBSTACLE_CID = 999


class _ImportHost(AB3DMOTStateTransferMixin):
    """Minimal real host exposing exactly the attributes _inject_warm_kf needs.

    _inject_warm_kf is the production AB3DMOT obstacle-import injection (the code
    ``import_tracked_obstacle_state`` runs on the destination edge). It reads
    self.tracker (ID_count, min_hits, trackers), self.track_to_carla, and
    self.handoff_track_grace_ticks; nothing else on the edge is touched.
    """

    def __init__(self, tracker) -> None:
        self.tracker = tracker
        self.track_to_carla = {}
        self.handoff_track_grace_ticks = 0


def _new_tracker():
    """Construct the destination edge's tracker exactly as the edge does.

    Mirrors _PluggableEdgeBase: get_tracker('ab3dmot', {...anchoring...}). The
    Car config resolves to metric=dist_3d, thres=-6, algm=hungar, anchoring=True,
    anchoring_epoch=40 inside AB3DMOT.get_param.
    """
    wrapper = get_tracker("ab3dmot", {"min_hits": 3, "max_age": 6, "anchoring": True})
    return wrapper.tracker


def _kfstate_at(x: float, z: float) -> KFState:
    """KFState (migration payload contents) for the imported obstacle.

    state_vector = [x, y, z, theta, l, w, h, dx, dy, dz]. x is pre-subtracted by
    IMPORT_VX so prediction() advances it back to the intended carried pose.
    """
    sv = np.array(
        [x - IMPORT_VX, TRUE_Y, z, 0.0, BOX_L, BOX_W, BOX_H, IMPORT_VX, 0.0, 0.0],
        dtype=float,
    )
    cov = np.eye(10) * 10.0
    return KFState(state_vector=sv, covariance=cov, hits=5, anchoring_age=0)


def _run_trial(mode: str, distractors: int, pose_err_m: float, seed: int) -> int:
    """One association trial. Returns 1 if the imported track binds the true
    re-detection, else 0. Uses the real import injection, prediction, and
    data_association; only the inputs are synthetic."""
    rng = np.random.default_rng(seed)

    # Carried-pose error: the imported track's state is offset from truth by
    # Gaussian noise of the given std on the ground plane.
    if pose_err_m > 0.0:
        nx, nz = rng.normal(0.0, pose_err_m, size=2)
    else:
        nx, nz = 0.0, 0.0
    carried_x, carried_z = TRUE_X + float(nx), TRUE_Z + float(nz)

    tracker = _new_tracker()
    host = _ImportHost(tracker)

    # --- imported track: real destination-side warm-KF injection + predict ---
    host._inject_warm_kf(OBSTACLE_CID, _kfstate_at(carried_x, carried_z))
    trks = tracker.prediction()  # real tracker prediction() -> Box3D trk inputs

    # --- local detections: true re-detection + distractor clutter ---
    # info row layout is [FRAME_IDX, GUID, CID]; CID carries beacon identity.
    det_rows = [[BOX_H, BOX_W, BOX_L, TRUE_X, TRUE_Y, TRUE_Z, 0.0]]
    true_cid = OBSTACLE_CID if mode == "stable_id" else -1
    info_rows = [[0, 111, true_cid]]

    for i in range(distractors):
        radius = rng.uniform(DISTRACTOR_R_MIN, DISTRACTOR_R_MAX)
        angle = rng.uniform(0.0, 2.0 * np.pi)
        ox, oz = radius * np.cos(angle), radius * np.sin(angle)
        det_rows.append(
            [BOX_H, BOX_W, BOX_L, TRUE_X + ox, TRUE_Y, TRUE_Z + oz, 0.0]
        )
        # Distractors are anonymous local clutter (no beacon identity).
        info_rows.append([0, 200 + i, -1])

    dets_arr = np.asarray(det_rows, dtype=float)
    info_arr = np.asarray(info_rows, dtype=np.int64)
    dets = tracker.process_dets(dets_arr, info_arr)  # real detection construction

    # stable_id needs the identity available at the field compute_affinity reads.
    if mode == "stable_id":
        for det, info_row in zip(dets, info_rows):
            det.info = np.asarray(info_row)

    # --- the function under test, unchanged, with the tracker's own settings ---
    matched, _unmatched_dets, _unmatched_trks, _cost, _affi = data_association(
        dets,
        trks,
        tracker.metric,
        tracker.thres,
        tracker.algm,
        None,
        anchoring=tracker.anchoring,
        anchoring_epoch=tracker.anchoring_epoch,
    )

    # trk index 0 is the imported track; det index 0 is the true re-detection.
    success = any(int(m[0]) == 0 and int(m[1]) == 0 for m in matched)
    return 1 if success else 0


def _git_short_sha() -> str:
    try:
        out = subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=_REPO_ROOT,
            stderr=subprocess.DEVNULL,
        )
        return out.decode().strip()
    except Exception:  # noqa: BLE001
        return "unknown"


def _fmt_pose(p: float) -> str:
    return f"{p:.1f}"


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-o", "--out", default=DEFAULT_OUT, help="output CSV path")
    ap.add_argument("--reps", type=int, default=REPS, help="trials per grid point")
    args = ap.parse_args()

    commit = _git_short_sha()
    rows = []
    idx = 0
    for mode in GRID_MODES:
        for distractors in GRID_DISTRACTORS:
            for pose_err in GRID_POSE_ERR:
                for rep in range(args.reps):
                    seed = idx
                    success = _run_trial(mode, distractors, pose_err, seed)
                    tag = (
                        f"assoc_{mode}_d{distractors}_p{_fmt_pose(pose_err)}_r{rep}"
                    )
                    rows.append(
                        {
                            "tag": tag,
                            "assoc_mode": mode,
                            "distractors": distractors,
                            "pose_err_m": _fmt_pose(pose_err),
                            "rep": rep,
                            "success": success,
                            "eval_tag": EVAL_TAG,
                            "assoc_fn": ASSOC_FN,
                            "assoc_fn_commit": commit,
                        }
                    )
                    idx += 1

    fieldnames = [
        "tag",
        "assoc_mode",
        "distractors",
        "pose_err_m",
        "rep",
        "success",
        "eval_tag",
        "assoc_fn",
        "assoc_fn_commit",
    ]
    outdir = os.path.dirname(os.path.abspath(args.out))
    if outdir:
        os.makedirs(outdir, exist_ok=True)
    with open(args.out, "w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)

    print(f"wrote {len(rows)} rows -> {args.out}")
    print(f"assoc_fn={ASSOC_FN} commit={commit}")

    # Console summary: mean success per (mode, distractors) across the pose_err
    # sweep, so the stable_id~1.0 / geometry-degrades shape is visible on run.
    print("\nmean success per (mode, distractors) across pose_err sweep:")
    for mode in GRID_MODES:
        for distractors in GRID_DISTRACTORS:
            sub = [
                r["success"]
                for r in rows
                if r["assoc_mode"] == mode and r["distractors"] == distractors
            ]
            mean = sum(sub) / len(sub) if sub else float("nan")
            print(f"  {mode:9s} d{distractors}  mean_success={mean:.3f}  (n={len(sub)})")

    print("\nmean success per (mode, distractors, pose_err_m):")
    for mode in GRID_MODES:
        for distractors in GRID_DISTRACTORS:
            cells = []
            for pose_err in GRID_POSE_ERR:
                sub = [
                    r["success"]
                    for r in rows
                    if r["assoc_mode"] == mode
                    and r["distractors"] == distractors
                    and r["pose_err_m"] == _fmt_pose(pose_err)
                ]
                mean = sum(sub) / len(sub) if sub else float("nan")
                cells.append(f"p{_fmt_pose(pose_err)}={mean:.2f}")
            print(f"  {mode:9s} d{distractors}  " + "  ".join(cells))


if __name__ == "__main__":
    main()
