# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""Conversion between Mamba3DMOT tracklets and TrackLatent payloads.

A live MambaTracklet3D carries the per-track state the migration unit
needs to transport. These helpers extract that state into a TrackLatent
(source side) and re-instantiate a tracklet from a TrackLatent into a
destination Mamba3DTracker (destination side).

Kept in a separate module so :mod:`.payload` stays free of torch and
tracker imports.
"""
from __future__ import annotations

import logging
from typing import Optional

import numpy as np

from ecav.core.tracking.mamba3dmot.basetrack import TrackState
from ecav.core.tracking.mamba3dmot.tracker import Mamba3DTracker
from ecav.core.tracking.mamba3dmot.tracklet import MambaTracklet3D

from .payload import TrackLatent

logger = logging.getLogger(__name__)


def _estimate_accel_mps2(memo_bank, memo_tick, sim_tick_s=0.05):
    """freeze-1l: position-domain 2nd-order least-squares acceleration over the
    transferred record.

    Returns ``(a_mps2, n_fit)``. Uses the record's per-frame POSITIONS and their
    source ticks: elapsed seconds ``t_i = (tick_i - tick_0) * sim_tick_s`` and
    along-track distance ``s_i`` = position projected onto the record's motion
    direction (unit vector from the first to the last valid-tick frame, which
    discards perpendicular detection jitter). Fits ``s(t) = c0 + c1 t + c2 t^2``
    by least squares and returns ``a = 2*c2``. Fitting position (not a speed
    slope) is far less jitter-sensitive. Needs >= 3 distinct-tick points for the
    quadratic; returns ``(0.0, n)`` below that (a depth-1/2 record -> pure CV).
    """
    if memo_bank is None or memo_tick is None:
        return 0.0, 0
    K = min(len(memo_bank), len(memo_tick))
    # Frames with a valid (>= 0) source tick, in record order.
    pts = []  # (tick_int, x, y)
    for i in range(K):
        ti = memo_tick[i]
        if ti is None:
            continue
        ti = int(ti)
        if ti < 0:
            continue
        p = np.asarray(memo_bank[i], dtype=np.float64)
        pts.append((ti, float(p[0]), float(p[1])))
    if len(pts) < 3:
        return 0.0, len(pts)
    # Strictly increasing ticks (drop repeated / regressed stamps) so the time
    # axis is well-defined for the quadratic fit.
    dedup = []
    for tk, x, y in pts:
        if dedup and tk <= dedup[-1][0]:
            continue
        dedup.append((tk, x, y))
    if len(dedup) < 3:
        return 0.0, len(dedup)
    t0 = dedup[0][0]
    ts = np.asarray([(tk - t0) * sim_tick_s for tk, _, _ in dedup],
                    dtype=np.float64)
    xs = np.asarray([x for _, x, _ in dedup], dtype=np.float64)
    ys = np.asarray([y for _, _, y in dedup], dtype=np.float64)
    dx, dy = xs[-1] - xs[0], ys[-1] - ys[0]
    norm = (dx * dx + dy * dy) ** 0.5
    if norm < 1e-6:
        return 0.0, len(dedup)
    ux, uy = dx / norm, dy / norm
    s = (xs - xs[0]) * ux + (ys - ys[0]) * uy
    # np.polyfit returns highest-degree first: [c2, c1, c0]. a = 2*c2.
    c2 = float(np.polyfit(ts, s, 2)[0])
    return 2.0 * c2, len(dedup)


def latent_from_tracklet(
    tracklet: MambaTracklet3D,
    *,
    persistent_vehicle_id: int,
    risk_score: float = 0.0,
    last_observation_t: float = 0.0,
    history_depth: Optional[int] = None,
    vel_mps: Optional[np.ndarray] = None,
) -> TrackLatent:
    """Snapshot a live tracklet's per-track state into a TrackLatent.

    ``history_depth`` truncates the migrated memo/diff banks to the
    most-recent N frames. This is how the Reactive-Kalman baseline (B1) is
    expressed: ``history_depth=1`` migrates only the latest bbox and the
    latest diff, which is the information a Kalman filter carries. The
    default (``None``) migrates the full history (the proposed design).
    """
    memo = np.asarray(tracklet.memo_bank, dtype=np.float32)
    diff = np.asarray(tracklet.diff_memo_bank, dtype=np.float32)
    # freeze-1k: per-frame source ticks, parallel to memo_bank; None (unstamped)
    # -> -1 sentinel so the array stays integer. Truncated with the memo below.
    _mt = getattr(tracklet, 'memo_tick', None)
    tick = np.asarray(
        [(-1 if t is None else int(t)) for t in (_mt or [])], dtype=np.int32)
    if history_depth is not None and history_depth > 0:
        memo = memo[-history_depth:]
        diff = diff[-history_depth:]
        if tick.size:
            tick = tick[-history_depth:]
    pred = (
        np.asarray(tracklet.predicted_last_bbox, dtype=np.float32).copy()
        if tracklet.predicted_last_bbox is not None
        else None
    )
    return TrackLatent(
        track_id=int(tracklet.track_id),
        persistent_vehicle_id=int(persistent_vehicle_id),
        memo_bank=memo.copy(),
        diff_memo_bank=diff.copy(),
        memo_tick=(tick.copy() if tick.size else None),
        bbox_3d=np.asarray(tracklet._bbox_3d, dtype=np.float32).copy(),
        predicted_last_bbox=pred,
        frame_id=int(tracklet.frame_id),
        start_frame=int(tracklet.start_frame),
        score=float(tracklet.score),
        is_activated=bool(tracklet.is_activated),
        state_flag=int(tracklet.state_flag),
        time_since_update=int(tracklet.time_since_update),
        risk_score=float(risk_score),
        last_observation_t=float(last_observation_t),
        vel_mps=(np.asarray(vel_mps, dtype=np.float32).copy()
                 if vel_mps is not None else None),
    )


def inject_latent_into_tracker(
    tracker: Mamba3DTracker,
    latent: TrackLatent,
    *,
    preserve_track_id: bool = True,
) -> MambaTracklet3D:
    """Re-instantiate a tracklet from a TrackLatent inside ``tracker``.

    The destination tracker must have been constructed with the same model
    config as the source. The shared motion model weights are already loaded
    by the destination's own ``Mamba3DTracker.__init__``; this function only
    restores the per-track state.

    Returns the inserted tracklet. The tracker's ``tracked_tracklets`` list
    is updated in place. The tracker's frame_id is advanced if the migrated
    track is from a newer frame than the destination currently has, so that
    the destination's clock does not regress.
    """
    # Build a tracklet with the migrated current bbox as init, then restore
    # the memo banks and metadata. The MambaTracklet3D constructor seeds
    # memo_bank with a single entry; we overwrite that with the full history.
    bbox_for_init = latent.bbox_3d.tolist()
    t = MambaTracklet3D(
        cfgs=tracker.cfgs,
        bbox_3d=bbox_for_init,
        score=latent.score,
        device=tracker.device,
    )

    # Restore Mamba motion-model input history
    t.memo_bank = [row.copy() for row in latent.memo_bank]
    t.diff_memo_bank = [row.copy() for row in latent.diff_memo_bank]
    # freeze-1m: number of transferred frames at import (the seam between imported
    # and local frames in memo_bank), for the Defect B history rebuild + seam log.
    t._n_imported = len(latent.memo_bank)
    # freeze-1m identity-settling metric: the carla_id this track was EXPORTED
    # for, so the destination can measure whether the 8 m association gate binds
    # the imported track to a neighbouring vehicle's id before settling on its
    # own (paper §7 cross-locale identity limitation, quantified in the rerun).
    t._migrated_cid = int(getattr(latent, 'persistent_vehicle_id', -1))
    # freeze-1m accel-consumption fix: count the DESTINATION's OWN local frames
    # since import (distinct from the migrated frames, which fill memo_bank during
    # coasting). The gate uses the TRANSFERRED _migrated_accel_mps2 directly while
    # this is below the threshold, then hands over to the destination's own
    # estimate, so warm reliably consumes the migrated acceleration in the blind
    # window instead of re-deriving it locally (the defect the a-mag probe found).
    t._n_local_since_import = 0
    t._bbox_3d = latent.bbox_3d.copy()
    t.predicted_last_bbox = (
        latent.predicted_last_bbox.copy()
        if latent.predicted_last_bbox is not None
        else None
    )

    # Restore identity + bookkeeping. We bypass next_id() because we want to
    # preserve the source-side track_id for cross-locale continuity.
    if preserve_track_id:
        t.track_id = int(latent.track_id)
        # Advance the basetrack id counter so future new tracks don't collide
        # with the migrated id.
        try:
            # MambaTracklet3D._count is a class attr in BaseTrack
            from ecav.core.tracking.mamba3dmot import basetrack as _bt
            _bt.BaseTrack._count = max(_bt.BaseTrack._count, int(latent.track_id))
        except Exception:  # noqa: BLE001
            logger.exception("Failed to advance BaseTrack id counter")

    t.frame_id = int(latent.frame_id)
    t.start_frame = int(latent.start_frame)
    t.score = float(latent.score)
    t.is_activated = bool(latent.is_activated)
    t.state_flag = int(latent.state_flag) if latent.state_flag is not None else TrackState.Tracked
    t.time_since_update = int(latent.time_since_update)
    # Time-denominated ground velocity from the source (m/s). While the
    # migrated track coasts unobserved, the destination dead-reckons with
    # this instead of the frame-denominated diffs, which are in the SOURCE
    # cadence and mis-scale at the destination's cadence. Cleared on the
    # first fresh observation (tracklet.update / re_activate).
    t._migrated_vel_mps = (
        np.asarray(latent.vel_mps, dtype=np.float64).copy()
        if getattr(latent, 'vel_mps', None) is not None else None)

    # freeze-1l acceleration path. Estimate a constant acceleration at the
    # destination from the transferred record (memo frames + per-frame source
    # ticks, both already in the payload; no new record field, no byte change)
    # by position-domain least squares, and carry it so the pre-commit coast
    # (tracklet) and the immature-import forecast (predictor) can use constant-
    # acceleration extrapolation in place of CV. kf stays pure constant velocity
    # (baseline definition) and is excluded. ON by default for every record-
    # carrying arm in every scenario (no per-scenario opt-in, which a reviewer
    # would read as tuning); kill-switch MIGRATION_ACCEL=0 only. The |a| floor
    # gate below sets the carried accel to 0 on constant-speed migrations, so
    # they reduce EXACTLY to CV (the downstream CV branches act only on a
    # truthy _migrated_accel_mps2).
    import os as _osa
    _accel_on = (_osa.environ.get('MIGRATION_ACCEL', '1') != '0')  # default on
    _mode = _osa.environ.get('MIGRATION_MODE', 'warm').lower()
    if _accel_on and _mode != 'kf':
        # Restore the per-frame source ticks so the fit has a time axis. The
        # ticks ride in the payload already and are otherwise unused at the
        # destination; restoring them is gated here so the frozen arms' tracklet
        # state stays unchanged when the accel path is off.
        _mt = getattr(latent, 'memo_tick', None)
        if _mt is not None:
            t.memo_tick = [int(x) for x in list(_mt)]
        _sts = 0.05
        try:
            _sts = float(getattr(tracker, 'cfgs', {}).get('sim_tick_s', 0.05))
        except Exception:  # noqa: BLE001
            _sts = 0.05
        _a, _nfit = _estimate_accel_mps2(
            t.memo_bank, getattr(t, 'memo_tick', None), sim_tick_s=_sts)
        # Noise floor: p99 of the estimator's |a| on the freeze-1k constant-
        # speed (hl_warm) records + margin. PROVISIONAL default 1.0 m/s^2.
        # The per-frame transferred record is NOT logged in freeze-1k (only the
        # aggregate migrated |v| is), so the exact p99 cannot be computed
        # offline. A synthetic constant-speed record with 0.3 m per-frame
        # detection jitter over a 10-frame / 0.2 s window puts |a| up to
        # ~1.7 m/s^2 through this speed-vs-tick estimator, so the floor is NOT
        # sub-1. TODO(smoke): run the freeze-1k hl_warm 20-seed set with
        # MIGRATION_ACCEL=1, take p99 of the [CAMIGRATED] est_a, add margin, and
        # set MIGRATION_ACCEL_FLOOR to it; ensure the mildest ONCOMING_ACCEL
        # sweep level exceeds the measured floor. Override via env, no code
        # change.
        _floor = float(_osa.environ.get('MIGRATION_ACCEL_FLOOR', '1.0'))
        t._migrated_accel_mps2 = _a if abs(_a) > _floor else 0.0
        logger.info(
            "[CAMIGRATED] track=%s est_a=%.3f n_fit=%d floor=%.3f applied=%.3f",
            int(latent.track_id), _a, _nfit, _floor, t._migrated_accel_mps2)

    # REPLACE on the same track_id (idfix): drop any existing tracklet with
    # this id before inserting, so the final update overwrites the coasted
    # shadow rather than appending a same-id duplicate.
    if preserve_track_id:
        tracker.tracked_tracklets = [
            _tk for _tk in tracker.tracked_tracklets
            if int(getattr(_tk, 'track_id', -1)) != int(t.track_id)]
    tracker.tracked_tracklets.append(t)
    if tracker.frame_id < latent.frame_id:
        tracker.frame_id = int(latent.frame_id)
    return t
