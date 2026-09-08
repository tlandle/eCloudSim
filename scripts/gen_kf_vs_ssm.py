#!/usr/bin/env python3
"""Fig 4: real AB3DMOT Kalman filter vs the SSM (Mamba) tracker, one-step error.

Both trackers cold-start and are driven through the IDENTICAL synthetic
detection stream (the harness's traj_* maneuvers). Per frame we record the
one-step-ahead position error vs ground truth and the velocity error.

  kalman = AB3DMOT_libs.kalman_filter.KF (filterpy, dim_x=10 constant-velocity,
           AB3DMOT's F/H/Q/R/P). Driven directly (single clean track, so the
           association / birth / anchoring machinery of AB3DMOT.track() is not
           needed). One-step position = kf.x[:2] after predict(); velocity =
           explicit state kf.x[7:9]/dt.
  ssm    = Mamba3DTracker. One-step position = predicted_last_bbox; velocity =
           one-step implied (pred - prev_obs)/dt (no explicit velocity state).

Detections are clean (obs = ground truth), so the SSM velocity error equals its
one-step error / dt by construction; the KF velocity error is independent
because the KF carries an explicit, smoothed velocity state.

columns: maneuver, tracker, t, vel_err_mps, one_step_err_m, track_id, seed
t is seconds relative to maneuver onset (the handoff frame, where each traj_*
maneuver begins). SSM tracker only + a real Kalman filter. No ADE/FDE columns.
"""
import csv
import os
import sys

import numpy as np

ECAV = "/home/atlas/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox"
if ECAV not in sys.path:
    sys.path.insert(0, ECAV)

import torch  # noqa: E402
from AB3DMOT_libs.kalman_filter import KF as ABKF  # noqa: E402
from ecav.core.application.edge.migration import harness as H  # noqa: E402
from ecav.core.tracking.mamba3dmot.tracker import Mamba3DTracker  # noqa: E402

OUT = os.path.join(ECAV, "docs/kb/data/relay_eval_2026_08")
WEIGHTS = os.path.join(ECAV, "ecav/core/tracking/mamba3dmot/mamba3dmot_weights.pth")
MANEUVERS = ["straight", "turn", "brake", "lane_change"]
DT = 0.05
ONSET = 10          # frame where each maneuver begins (harness handoff)
TOTAL = 70          # 0.5 s pre-onset + 3.0 s of maneuver
SEED = 1


def build_traj(man):
    gen = H.TRAJECTORIES[man]
    return gen(TOTAL) if man == "straight" else gen(TOTAL, handoff=ONSET)


def kf_series(traj):
    """Real AB3DMOT filterpy KF. Returns {frame: (one_step_err_m, vel_err_mps)}."""
    def to_state(b):  # harness box [x,y,z,l,w,h,yaw] -> KF state [x,y,z,theta,l,w,h]
        return np.array([b[0], b[1], b[2], b[6], b[3], b[4], b[5]], dtype=float)
    info = np.array([0.95, -1.0, -1.0])   # [score, guid=-1, cid=-1]; anchoring not used
    trk, prev_gt, out = None, None, {}
    for i in range(len(traj)):
        z = to_state(traj[i])
        gt = np.asarray(traj[i][:2], dtype=float)
        if trk is None:
            trk = ABKF(z.copy(), info, 0)
            prev_gt = gt
            continue
        trk.kf.predict()
        pred_xy = np.asarray(trk.kf.x[:2]).reshape(-1)
        vel_xy = np.asarray(trk.kf.x[7:9]).reshape(-1) / DT
        one_step = float(np.hypot(*(pred_xy - gt)))
        true_vel = (gt - prev_gt) / DT
        vel_err = float(np.hypot(*(vel_xy - true_vel)))
        out[i] = (one_step, vel_err)
        trk.kf.update(z.reshape((7, 1)))
        prev_gt = gt
    return out, 0


def ssm_series(cfg, device, traj):
    """Mamba SSM tracker. Returns {frame: (one_step_err_m, vel_err_mps)}, track_id."""
    tracker = Mamba3DTracker(cfg, device=device)
    results = H._drive(tracker, traj, start_frame=0)
    out = {}
    for i in range(1, len(results)):
        pred = results[i].predicted_bbox
        if pred is None:
            continue
        gt = np.asarray(traj[i][:2], dtype=float)
        prev_gt = np.asarray(traj[i - 1][:2], dtype=float)
        pred_xy = np.asarray(pred[:2], dtype=float)
        one_step = float(np.hypot(*(pred_xy - gt)))
        est_vel = (pred_xy - prev_gt) / DT      # SSM has no explicit velocity state
        true_vel = (gt - prev_gt) / DT
        vel_err = float(np.hypot(*(est_vel - true_vel)))
        out[i] = (one_step, vel_err)
    tid = int(tracker.tracked_tracklets[0].track_id) if tracker.tracked_tracklets else -1
    return out, tid


def main():
    device = "cuda" if torch.cuda.is_available() else "cpu"
    cfg = H._tracker_cfg(WEIGHTS)
    rows = []
    for man in MANEUVERS:
        traj = build_traj(man)
        kf_out, kf_id = kf_series(traj)
        ssm_out, ssm_id = ssm_series(cfg, device, traj)
        for i in sorted(set(kf_out) & set(ssm_out)):
            t = round((i - ONSET) * DT, 3)
            os_kf, ve_kf = kf_out[i]
            os_ss, ve_ss = ssm_out[i]
            rows.append([man, "kalman", t, round(ve_kf, 4), round(os_kf, 4), kf_id, SEED])
            rows.append([man, "ssm",    t, round(ve_ss, 4), round(os_ss, 4), ssm_id, SEED])
    path = os.path.join(OUT, "microbench_kf_vs_ssm.csv")
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["maneuver", "tracker", "t", "vel_err_mps", "one_step_err_m", "track_id", "seed"])
        w.writerows(rows)
    print(f"wrote {path} ({len(rows)} rows)")

    # Compact summary: mean one-step error over the maneuver window (t in [0,2] s).
    import statistics as st
    print("\nmean one_step_err_m over maneuver window t in [0,2] s:")
    for man in MANEUVERS:
        for trk in ("kalman", "ssm"):
            vals = [r[4] for r in rows if r[0] == man and r[1] == trk and 0.0 <= r[2] <= 2.0]
            print(f"  {man:<12} {trk:<7} {st.mean(vals):.3f}  (n={len(vals)})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
