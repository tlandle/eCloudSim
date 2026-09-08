#!/usr/bin/env python3
"""Recompute the forced-handoff microbench as final displacement error (FDE).

The original microbench (ecav .../edge/migration/harness.py) reports, per arm,
the mean of the per-frame ONE-STEP-AHEAD xy error over the first five
post-handoff frames (harness FrameResult.error_xy vs the synthetic detection =
ground truth). That is a one-step online-tracking error, not FDE at a fixed
horizon. This script reuses the SAME replay platform (same synthetic
trajectories, same handoff frame, same three payload arms driven through the
same Mamba3DTracker) and instead reports FDE: the destination's open-loop
forecast endpoint at the +3 s and +5 s horizons against the actor's true
position at that horizon (ground truth, matching the reference the original
code used).

Arms (per the paper): history = full latent migration; one_frame =
history_depth=1 (latest observation only); cold = no migration.

Aggregation matches the original: mean over the first five post-handoff frames.
At issue frame f the destination has observed post-handoff detections
0..f (online), then forecasts open-loop H frames ahead; the endpoint is
compared to ground truth at handoff+f+H. dt = 0.05 s, so H=100 is 5 s and
H=60 is 3 s.

Run (conda opencda310, GPU required by the mamba kernel):
    python scripts/recompute_microbench_fde.py
Writes mock_data/measured_microbench_fde.csv.
"""
from __future__ import annotations

import copy
import csv
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))
ECAV = os.path.abspath(os.path.join(REPO, "..", "..", "TrafficSimulator_eCloud",
                                    "ecloudsim_distributed_sandbox"))
if ECAV not in sys.path:
    sys.path.insert(0, ECAV)

import torch  # noqa: E402

from ecav.core.application.edge.migration.harness import TRAJECTORIES, _tracker_cfg  # noqa: E402
from ecav.core.application.edge.migration.factories import (  # noqa: E402
    inject_latent_into_tracker, latent_from_tracklet)
from ecav.core.tracking.mamba3dmot.tracker import Mamba3DTracker  # noqa: E402

DT = 0.05
HANDOFF = 10          # 0.5 s of pre-handoff history at edge A (matches harness default)
WINDOW = 5            # first five post-handoff issue frames (matches harness gap_window)
H5 = 100              # 5.0 s / dt
H3 = 60               # 3.0 s / dt
MANEUVERS = ["straight", "turn", "brake", "lane_change"]
WEIGHTS = os.path.join(ECAV, "ecav/core/tracking/mamba3dmot/mamba3dmot_weights.pth")


def _rollout_endpoints(tracklet, horizons):
    """Open-loop forecast: coast the tracklet's own motion model forward.

    Returns {H: (x, y)} the predicted center at each horizon (frames ahead).
    Each step calls predict() (sets predicted_last_bbox) then update(None)
    (appends the prediction as a pseudo-observation and advances), which is the
    tracker's own dead-reckoning / autoregressive rollout path.
    """
    clone = copy.deepcopy(tracklet)
    want = set(horizons)
    out = {}
    hmax = max(horizons)
    for step in range(1, hmax + 1):
        clone.predict()
        if clone.predicted_last_bbox is None:
            break
        if step in want:
            out[step] = (float(clone.predicted_last_bbox[0]),
                         float(clone.predicted_last_bbox[1]))
        clone.update(None, frame_id=clone.frame_id + step)
    return out


def _fde_for_arm(cfg, device, traj, arm):
    """Window-mean FDE at 3 s and 5 s for one arm on one maneuver.

    arm in {history, one_frame, cold}. Returns (fde3, fde5) in metres.
    """
    pre = traj[:HANDOFF]

    # Source edge A: accumulate pre-handoff history.
    edge_a = Mamba3DTracker(cfg, device=device)
    for det in pre:
        edge_a.update(det.reshape(1, -1), np.array([0.95], dtype=np.float32))
    src = edge_a.tracked_tracklets[0]

    # Destination setup per arm.
    if arm == "cold":
        edge = Mamba3DTracker(cfg, device=device)          # no migration
    else:
        depth = 1 if arm == "one_frame" else None          # one_frame = latest obs only
        latent = latent_from_tracklet(src, persistent_vehicle_id=42,
                                      last_observation_t=0.5, history_depth=depth)
        edge = Mamba3DTracker(cfg, device=device)
        inject_latent_into_tracker(edge, latent)

    fdes3, fdes5 = [], []
    for f in range(WINDOW):
        det = traj[HANDOFF + f]
        edge.update(det.reshape(1, -1), np.array([0.95], dtype=np.float32))
        if not edge.tracked_tracklets:
            continue
        trk = edge.tracked_tracklets[0]
        if trk.predicted_last_bbox is None and len(trk.memo_bank) < 2:
            # No forecast available yet (cold start, single frame): the
            # destination cannot place the actor's future position. Score the
            # static hold at the last observation, which is the best a
            # zero-velocity state can do.
            ep3 = ep5 = (float(trk.memo_bank[-1][0]), float(trk.memo_bank[-1][1]))
        else:
            eps = _rollout_endpoints(trk, (H3, H5))
            last = (float(trk.memo_bank[-1][0]), float(trk.memo_bank[-1][1]))
            ep3 = eps.get(H3, last)
            ep5 = eps.get(H5, last)
        gt3 = traj[HANDOFF + f + H3]
        gt5 = traj[HANDOFF + f + H5]
        fdes3.append(float(np.hypot(ep3[0] - gt3[0], ep3[1] - gt3[1])))
        fdes5.append(float(np.hypot(ep5[0] - gt5[0], ep5[1] - gt5[1])))
    return (float(np.mean(fdes3)) if fdes3 else float("nan"),
            float(np.mean(fdes5)) if fdes5 else float("nan"))


def main() -> int:
    device = "cuda" if torch.cuda.is_available() else "cpu"
    cfg = _tracker_cfg(WEIGHTS)
    total = HANDOFF + WINDOW + H5 + 1     # ground truth must extend to the farthest horizon

    rows = []
    for man in MANEUVERS:
        gen = TRAJECTORIES[man]
        traj = gen(total) if man == "straight" else gen(total, handoff=HANDOFF)
        res = {}
        for arm in ("history", "one_frame", "cold"):
            f3, f5 = _fde_for_arm(cfg, device, traj, arm)
            res[arm] = (f3, f5)
        rows.append([
            man,
            round(res["history"][1], 3), round(res["one_frame"][1], 3), round(res["cold"][1], 3),
            round(res["history"][0], 3), round(res["one_frame"][0], 3), round(res["cold"][0], 3),
        ])
        print(f"{man:<12} FDE5 hist={res['history'][1]:.3f} one={res['one_frame'][1]:.3f} "
              f"cold={res['cold'][1]:.3f}  | FDE3 hist={res['history'][0]:.3f} "
              f"one={res['one_frame'][0]:.3f} cold={res['cold'][0]:.3f}")

    out = os.path.join(REPO, "mock_data", "measured_microbench_fde.csv")
    with open(out, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["maneuver", "history_fde5_m", "one_frame_fde5_m", "cold_fde5_m",
                    "history_fde3_m", "one_frame_fde3_m", "cold_fde3_m"])
        w.writerows(rows)
    print(f"wrote {os.path.relpath(out, REPO)} ({len(rows)} rows)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
