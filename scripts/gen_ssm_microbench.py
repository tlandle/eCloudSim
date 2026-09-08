#!/usr/bin/env python3
"""Land the two UNBLOCKED SSM microbench CSVs from the real forced-handoff harness.

Metric everywhere: one_step_err_m = mean over the first five post-handoff frames
of the tracker's one-step-ahead xy position error vs ground truth
(FrameResult.error_xy in harness.py). SSM tracker only (Mamba3DTracker). No
Kalman arm, no ADE/FDE/displacement columns.

  microbench_onestep.csv    (Table 6): maneuver, arm, one_step_err_m
  microbench_history_depth.csv (Fig 5): maneuver, depth_frames, track_id, seed, one_step_err_m

The harness is deterministic (fixed synthetic trajectories, eval-mode model, no
RNG), so there is exactly one real track and one seed per cell.
"""
import csv
import os
import sys

import numpy as np

ECAV = "/home/atlas/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox"
if ECAV not in sys.path:
    sys.path.insert(0, ECAV)

import torch  # noqa: E402
from ecav.core.application.edge.migration import harness as H  # noqa: E402
from ecav.core.application.edge.migration.factories import (  # noqa: E402
    inject_latent_into_tracker, latent_from_tracklet)
from ecav.core.tracking.mamba3dmot.tracker import Mamba3DTracker  # noqa: E402

OUT = os.path.join(ECAV, "docs/kb/data/relay_eval_2026_08")
WEIGHTS = os.path.join(ECAV, "ecav/core/tracking/mamba3dmot/mamba3dmot_weights.pth")
MANEUVERS = ["straight", "turn", "brake", "lane_change"]
HANDOFF = 10
TOTAL = 30
WINDOW = 5
SEED = 1


def window_mean(results, window=WINDOW):
    # Match harness _summary_row b_win/c_win exactly: nanmean of the first
    # `window` frames by POSITION (a cold birth frame is NaN and stays in the
    # window), not the first `window` non-NaN errors.
    arr = np.array([r.error_xy for r in results], dtype=float)
    w = min(window, len(arr))
    seg = arr[:w]
    return float(np.nanmean(seg)) if np.any(~np.isnan(seg)) else float("nan")


def build_traj(man):
    gen = H.TRAJECTORIES[man]
    return gen(TOTAL) if man == "straight" else gen(TOTAL, handoff=HANDOFF)


def onestep_rows(cfg, device):
    """Table 6: history / one_frame / cold, 5f-window one-step error."""
    rows = []
    for man in MANEUVERS:
        out = H.run(WEIGHTS, handoff_frame=HANDOFF, total_frames=TOTAL,
                    trajectory=man, verbose=False)
        # b=full history, b1=history_depth=1 (the "one_frame" arm the harness
        # mislabels "kalman"; it is the SSM model with one frame of history),
        # c=cold.
        rows.append([man, "history",   round(window_mean(out["b_results"]), 3)])
        rows.append([man, "one_frame", round(window_mean(out["b1_results"]), 3)])
        rows.append([man, "cold",      round(window_mean(out["c_results"]), 3)])
    return rows


def depth_err(cfg, device, traj, depth):
    """5f-window one-step error when depth frames of history are migrated.

    depth=0 -> cold (nothing migrated). depth>0 -> inject the latent truncated
    to the last `depth` frames.
    """
    pre, post = traj[:HANDOFF], traj[HANDOFF:]
    edge_a = Mamba3DTracker(cfg, device=device)
    H._drive(edge_a, pre, start_frame=0)
    src = edge_a.tracked_tracklets[0]

    if depth == 0:
        edge = Mamba3DTracker(cfg, device=device)                 # cold
    else:
        latent = latent_from_tracklet(src, persistent_vehicle_id=42,
                                      last_observation_t=0.5, history_depth=depth)
        edge = Mamba3DTracker(cfg, device=device)
        inject_latent_into_tracker(edge, latent)

    results = H._drive(edge, post, start_frame=HANDOFF)
    tid = int(edge.tracked_tracklets[0].track_id) if edge.tracked_tracklets else -1
    return window_mean(results), tid


def history_depth_rows(cfg, device):
    """Fig 5: depth sweep {0,1,2,5,10}."""
    rows = []
    for man in MANEUVERS:
        traj = build_traj(man)
        for depth in (0, 1, 2, 5, 10):
            val, tid = depth_err(cfg, device, traj, depth)
            rows.append([man, depth, tid, SEED, round(val, 3)])
    return rows


def write_csv(name, header, rows):
    path = os.path.join(OUT, name)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)
    print(f"wrote {path} ({len(rows)} rows)")
    return path


def main():
    device = "cuda" if torch.cuda.is_available() else "cpu"
    cfg = H._tracker_cfg(WEIGHTS)

    r1 = onestep_rows(cfg, device)
    write_csv("microbench_onestep.csv", ["maneuver", "arm", "one_step_err_m"], r1)
    print("--- microbench_onestep.csv ---")
    for r in r1:
        print(r)

    r2 = history_depth_rows(cfg, device)
    write_csv("microbench_history_depth.csv",
              ["maneuver", "depth_frames", "track_id", "seed", "one_step_err_m"], r2)
    print("--- microbench_history_depth.csv ---")
    for r in r2:
        print(r)
    return 0


if __name__ == "__main__":
    sys.exit(main())
