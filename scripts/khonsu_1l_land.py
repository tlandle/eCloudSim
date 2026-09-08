#!/usr/bin/env python3
# Author: Tyler Landle <tlandle3@gatech.edu>
"""freeze-1l central-campaign landers (two modes).

Post-hoc log landers for the freeze-1l central campaign in
idfix_wt/evaluation_outputs/frozen_1l_central. Parses .log files only; no
simulator, GPU, or sim-code changes. STD columns are produced with
khonsu_design_extract's conventions; gating / launch / forecast helpers are
imported from khonsu_ablation_diag. Existing scripts are unmodified.

Modes:
  acc  -> frozen1l_acc_rows.csv   (acc_{arm}_a{A}_s{S}, arm in
          warm/warmabl/kf/reactive/cold)
          STD + oncoming_accel, decision_path, est_a, mtr_implied_a,
          actual_a, motion
  hln  -> frozen1l_hl_n{N}_rows.csv  (hln{N}_{arm}_s{S})
          STD + ns3_lut_n, age_med_ms, age_p95_ms

Usage:
  python scripts/khonsu_1l_land.py acc <logdir> [-o out.csv] [--tag TAG]
  python scripts/khonsu_1l_land.py hln <logdir> [--outdir DIR] [--tag TAG]

Both re-parse the whole logdir on each run (overwrite). Incomplete logs (no
[RUNROW]) and degenerate startup transients are skipped with a stderr note, so
running mid-campaign lands only the finished cells.
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import re
import sys

# khonsu_design_extract (STD conventions) and khonsu_ablation_diag (gating /
# launch / forecast helpers) live beside this script; add scripts/ to the path
# so a bare `python scripts/khonsu_1l_land.py ...` invocation can import them.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import khonsu_design_extract as kde  # noqa: E402
import khonsu_ablation_diag as kad  # noqa: E402

# STD column order shared with khonsu_design_extract (schema `STD`).
STD_COLS = [
    "tag", "mode", "trigger", "band", "refresh", "mirror", "look", "rep",
    "eps", "ct", "collided", "dist_m", "time_s", "completed", "tx", "by",
    "eps_raw", "contact_raw", "machine", "oncoming_speed", "trigger_dist",
    "eval_tag", "final_update",
]
ACC_EXTRA = ["oncoming_accel", "commanded_a", "actual_a", "speed_at_crossing",
             "est_a", "est_a_max", "mtr_implied_a", "decision_path", "motion"]
HLN_EXTRA = ["ns3_lut_n", "age_med_ms", "age_p95_ms"]
# FDE lander (Tyler's metric): min of the two highest-scoring modes' final
# displacement at the horizon, mean over the first five post-commit refreshes.
# No ADE / minADE / miss-rate columns (Tyler's rule).
FDE_COLS = ["tag", "arm", "seed", "fde5_m", "fde3_m", "fde5_top_m",
            "eval_tag", "machine"]
# Top-mode frozen-rate lander (freeze-1k headline).
TOPMODE_COLS = ["tag", "arm", "seed", "completed", "collided",
                "forecast_blocks", "frozen_blocks", "eval_tag"]
MODES_HDR_RE = re.compile(
    r"\[MODESROW\] tick=(\d+) tid=(-?\d+) cid=(-?\d+) h=([0-9.]+)")
MODE_TUP_RE = re.compile(
    r"m(\d+)=\((nan|-?[0-9.]+),(nan|-?[0-9.]+),(-?[0-9.]+)\)")
FROZEN_DISP_M = 1.0     # top-mode self-motion over 5 s below this = frozen
ACTUAL_MOVE_M = 10.0    # actor's true motion over the same window above this

# ---- non-central blocks (freshness, t25b, theta+pdst, faults, repl_period) ----
AGEROW_TICK_RE = re.compile(
    r"\[AGEROW\] tick=(\d+) edge=\S+ realized_age_ms=([0-9.]+)")
PDSTROW_RE = re.compile(
    r"\[PDSTROW\] tick=(\d+) npc=(-?\d+) p_dst=([0-9.]+) "
    r"mtr_theta=([0-9.]+) fired=(\w+)")
FAULTROW_RE = re.compile(
    r"\[FAULTROW\] fault=(\S+) double_emission_ms=([0-9.]+) recovered=(\w*)")
FAULTPATH_RE = re.compile(
    r"\[FAULTPATH\] fault=(\S+) tid=(-?\d+) stale_consumed=(\w+) "
    r"fallback=(\w+) recovered=(\w+)")
HANDOFF_WBFU_RE = re.compile(r"\[HANDOFFROW\][^\n]*warm_before_first_use=(\w+)")
FRESH_TAG_RE = re.compile(r"^fresh_(?P<scn>flow|accel)_i(?P<inj>\d+)_s(?P<seed>\d+)$")
T25B_TAG_RE = re.compile(r"^t25b_(?P<mode>[a-z]+)_x(?P<shift>-?\d+)_s(?P<seed>\d+)$")
THETA_TAG_RE = re.compile(r"^th_mtr(?P<theta>[0-9.]+)_s(?P<seed>\d+)$")
FLT_TAG_RE = re.compile(r"^flt_(?P<fault>[a-z_]+)_e(?P<ep>on|off)_s(?P<seed>\d+)$")
REPL_TAG_RE = re.compile(r"^repl_p(?P<period>[0-9.]+)_s(?P<seed>\d+)$")
AGESWEEP_EXTRA = ["scenario", "inject_ms", "baseline_age_ms", "realized_age_ms"]
THETA_EXTRA = ["mtr_theta"]
PDST_COLS = ["tag", "tick", "p_dst", "mtr_theta", "fired", "eval_tag"]
FAULTS_COLS = ["tag", "fault", "epochs", "rep", "double_emission_ms",
               "stale_consumed", "fallback", "recovered", "eval_tag"]
REPL_EXTRA = ["repl_period_s", "bytes_per_crossing", "warm_before_first_use",
              "age_at_decision_ms"]

# Modes whose final update is built in (warm path + the final-sync arms).
_FINAL_BUILTIN = {
    "warm", "edgewarp", "edgewarp_full", "repl_final", "kf_final",
}

CAMIG_RE = re.compile(
    r"\[CAMIGRATED\] track=(\d+) est_a=(-?[0-9.]+) n_fit=(\d+) "
    r"floor=(-?[0-9.]+) applied=(-?[0-9.]+)")
EVAL_HDR_RE = re.compile(
    r"\[EVAL\] tick=(\d+) arm=\S+ tag=(\S+) Track (\d+) -> \S+ "
    r"\(CARLA ID (\d+)\)")
AGEROW_RE = re.compile(r"\[AGEROW\][^\n]*?realized_age_ms=(-?[0-9.]+)")
ACC_TAG_RE = re.compile(r"^acc_(?P<arm>[a-z]+)_a(?P<accel>[0-9.]+)_s(?P<seed>\d+)$")
HLN_TAG_RE = re.compile(r"^hln(?P<n>\d+)_(?P<arm>[a-z]+)_s(?P<seed>\d+)$")

ARM_MODE_LABEL = {"edgewarp_full": "edgewarp"}  # figure-time relabel (schema 46)


# ---------------------------------------------------------------------------
# Shared helpers
# ---------------------------------------------------------------------------
def _read(path):
    return open(path, encoding="utf-8", errors="ignore").read()


def _raw_collisions(text):
    """Uncapped collision episodes/contact ticks from the per-tick warnings,
    deduplicated at >1 s gaps (khonsu_design_extract.main convention)."""
    ts = []
    for ln in text.splitlines():
        if "WARNING" in ln and "Collision" in ln and "Eval" not in ln:
            m = re.search(r"(\d{2}):(\d{2}):(\d{2}),(\d{3})", ln)
            if m:
                h, mn, sec, ms = map(int, m.groups())
                ts.append(h * 3600 + mn * 60 + sec + ms / 1000.0)
    eps, last = 0, None
    for t in ts:
        if last is None or t - last > 1.0:
            eps += 1
        last = t
    return eps, len(ts)


def _eval_tag(text, default):
    m = re.search(r"\[EVAL\] tick=\d+ arm=\S+ tag=(\S+)", text)
    return m.group(1) if m else default


def std_row(path, name, machine, oncoming_speed, trigger_dist, tag_default):
    """Assemble the STD columns for one log, or return None (no RUNROW) /
    'INVALID' (degenerate startup) exactly as khonsu_design_extract does."""
    core = kde.parse_log(path)
    if core is None or core == "INVALID":
        return core
    text = _read(path)
    eps_raw, contact_raw = _raw_collisions(text)
    seed = ""
    sm = re.search(r"_s(\d+)", name)
    if sm:
        seed = sm.group(1)
    rm = re.search(r"_r(\d+)", name)
    rep = rm.group(1) if rm else seed  # acc/hln index on _s; _r if present
    raw_mode = core.get("mode", "")
    if raw_mode in _FINAL_BUILTIN:
        final_update = "builtin"
    elif core.get("refresh") == "full":
        final_update = "commit_full"
    else:
        final_update = "none"
    row = dict(core)
    row.update({
        "tag": name,
        "rep": rep,
        "machine": machine,
        "oncoming_speed": oncoming_speed,
        "trigger_dist": trigger_dist,
        "eval_tag": _eval_tag(text, tag_default),
        "final_update": final_update,
        "eps_raw": eps_raw,
        "contact_raw": contact_raw,
    })
    return row


def _pctl(vals, q):
    """Linear-interpolated percentile q in [0,1] over vals; '' if empty."""
    if not vals:
        return ""
    s = sorted(vals)
    if len(s) == 1:
        return round(s[0], 1)
    idx = q * (len(s) - 1)
    lo = int(idx)
    hi = min(lo + 1, len(s) - 1)
    frac = idx - lo
    return round(s[lo] * (1 - frac) + s[hi] * frac, 1)


# ---------------------------------------------------------------------------
# ACC mode
# ---------------------------------------------------------------------------
def _dest_track_ids(text, d, gid):
    """Tracker track_ids that EVAL associates with the gating carla_id while it
    is in the destination locale (Actual x >= DEST_X_MIN). track_ids are reused
    across CARLA ids over a run, so a CAMIGRATED track is only associated with
    the gating oncoming through its destination-locale EVAL records."""
    if gid == "" or gid is None:
        return set()
    ax_by_tick = {r["tick"]: r["ax"] for r in d["eval"].get(gid, [])}
    ids = set()
    for m in EVAL_HDR_RE.finditer(text):
        tick, tk, cid = int(m.group(1)), int(m.group(3)), int(m.group(4))
        if cid != gid:
            continue
        ax = ax_by_tick.get(tick)
        if ax is not None and ax >= kad.DEST_X_MIN:
            ids.add(tk)
    return ids


def _quad_c2(ts, ss):
    """Stdlib quadratic least squares: fit s = c0 + c1 t + c2 t^2 and return c2
    (so a = 2*c2). Solves the 3x3 normal equations by Cramer's rule (no numpy;
    landers are stdlib-only). None if < 3 points or singular."""
    n = len(ts)
    if n < 3:
        return None
    S0 = float(n)
    S1 = sum(ts)
    S2 = sum(t * t for t in ts)
    S3 = sum(t ** 3 for t in ts)
    S4 = sum(t ** 4 for t in ts)
    T0 = sum(ss)
    T1 = sum(t * s for t, s in zip(ts, ss))
    T2 = sum(t * t * s for t, s in zip(ts, ss))

    def det3(m):
        return (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))

    dM = det3([[S0, S1, S2], [S1, S2, S3], [S2, S3, S4]])
    if abs(dM) < 1e-12:
        return None
    dC2 = det3([[S0, S1, T0], [S1, S2, T1], [S2, S3, T2]])
    return dC2 / dM


def _gt_accel_speed(d, gid):
    """GT-MEASURED acceleration and along-track speed of the gating oncoming
    over the 2 s window ending at its crossing.

    From [GT INJECT DBG] world=(x,y) frames for gid: crossing = the first frame
    with x >= 250; window = [cross-40, cross] (2 s at 0.05 s/frame); project
    (x,y) onto the first->last motion unit vector to get along-track s(t) with
    t=(frame-f0)*0.05; a = 2*c2 of a quadratic fit of s vs t. speed_at_crossing
    = along-track speed over the last two window samples. ('','') if gid lacks
    GT frames or never reaches x >= 250."""
    if gid == "" or gid is None:
        return "", ""
    try:
        gtf = d["gt"].get(int(gid), {})
    except (TypeError, ValueError):
        gtf = {}
    if len(gtf) < 3:
        return "", ""
    frames = sorted(gtf)
    cross = next((f for f in frames if gtf[f][0] >= 250.0), None)
    if cross is None:
        return "", ""
    win = [f for f in frames if cross - 40 <= f <= cross]
    if len(win) < 3:
        return "", ""
    f0 = win[0]
    xs = [gtf[f][0] for f in win]
    ys = [gtf[f][1] for f in win]
    ts = [(f - f0) * kad.SIM_DT for f in win]
    dx, dy = xs[-1] - xs[0], ys[-1] - ys[0]
    norm = math.hypot(dx, dy)
    if norm < 1e-6:
        return "", ""
    ux, uy = dx / norm, dy / norm
    s = [(xs[i] - xs[0]) * ux + (ys[i] - ys[0]) * uy for i in range(len(win))]
    c2 = _quad_c2(ts, s)
    a = round(2.0 * c2, 3) if c2 is not None else ""
    dt = (win[-1] - win[-2]) * kad.SIM_DT
    spd = round((s[-1] - s[-2]) / dt, 3) if dt > 0 else ""
    return a, spd


def _camig_assigned(text):
    """Assign each [CAMIGRATED] the sim tick of the nearest preceding EGO-DBG
    tick=N line.

    CAMIGRATED carries no tick. Both CAMIGRATED and EGO-DBG lines are wall-clock
    timestamped and written in emission order, so walking the log once and
    tagging each CAMIGRATED with the last-seen EGO-DBG tick anchors it in sim
    time (EGO-DBG is emitted every 5 ticks, so the assignment error is <= 5
    ticks). Returns [(assigned_tick_or_None, track_id, applied)]."""
    out = []
    last = None
    for ln in text.splitlines():
        if "[EGO-DBG] tick=" in ln:
            m = re.search(r"\[EGO-DBG\] tick=(\d+)", ln)
            if m:
                last = int(m.group(1))
        elif "[CAMIGRATED]" in ln:
            m = CAMIG_RE.search(ln)
            if m:
                out.append((last, int(m.group(1)), float(m.group(5))))
    return out


def _gating_camigs(text, d, gid):
    """CAMIGRATED lines (assigned ticks) for the gating oncoming: resolved via
    its destination-locale EVAL track_ids, falling back to all lines when there
    is exactly one distinct migrated track (CAMIGRATED fires only for migrated
    obstacles = the oncoming)."""
    camigs = _camig_assigned(text)
    if not camigs:
        return []
    dest_ids = _dest_track_ids(text, d, gid)
    cands = [c for c in camigs if c[1] in dest_ids]
    if not cands and len({c[1] for c in camigs}) == 1:
        cands = camigs
    return cands


# Physical guards for the MTR-implied acceleration read. In the maneuvering
# panel the accelerating oncoming clears before the (late) launch, so the block
# nearest launch is often a passed/reacquired track in the wrong lane or a
# broken CV extrapolation (observed: warm y=187 off-lane, kf +0.25s at x=1467 ->
# v=4500 m/s). Restrict to the oncoming lane, +x motion, and plausible speeds /
# acceleration so the read reflects the oncoming's approach forecast.
_V_MAX_MPS = 40.0        # reject forecast steps above this (garbage)
_A_MAX_MPS2 = 30.0       # reject implied |a| above this (garbage)


def _mtr_implied_a(d, gid, lt):
    """Implied acceleration of the mature MTR forecast for the gating oncoming.

    Per forecast block the task's formula is v(+0.25)=|f0.25-Actual|/0.25,
    v(+0.5)=|f0.5-f0.25|/0.25, implied a=(v(+0.5)-v(+0.25))/0.25. A single block
    is a second difference of predicted positions and is jitter-dominated (the
    same reason wave-1's estimator moved to a 2nd-order fit), so the reported
    value is the MEDIAN of that per-block implied a over the oncoming's mature
    in-lane approach blocks (|ay-199|<band, +x motion, moving v025>1 m/s, and
    plausible speeds / |a| so passed-and-reacquired or broken-CV blocks after
    the crossing are dropped). '' if there is no such mature forecast."""
    if gid == "" or gid is None:
        return ""
    accs = []
    for r in d["eval"].get(gid, []):
        if abs(r["ay"] - kad.ONCOMING_Y) >= kad.ONCOMING_BAND:
            continue
        f025 = r["futs"].get(0.25)
        f050 = r["futs"].get(0.50)
        if f025 is None or f050 is None:
            continue
        if f025[0] < r["ax"] - 0.1:  # oncoming moves +x; reject -x actors in-band
            continue
        v025 = math.hypot(f025[0] - r["ax"], f025[1] - r["ay"]) / 0.25
        v050 = math.hypot(f050[0] - f025[0], f050[1] - f025[1]) / 0.25
        if not (1.0 < v025 <= _V_MAX_MPS and 0.0 <= v050 <= _V_MAX_MPS):
            continue
        a = (v050 - v025) / 0.25
        if abs(a) > _A_MAX_MPS2:
            continue
        accs.append(a)
    if not accs:
        return ""
    accs.sort()
    n = len(accs)
    med = accs[n // 2] if n % 2 else (accs[n // 2 - 1] + accs[n // 2]) / 2.0
    return round(med, 3)


def _decision_path(d, gid, lt, text):
    """Which forecast the sustained-launch decision consumed:
      'coast'    launch before the gating track's destination commit
                 (first_dst_track_tick) -> pre-commit coast forecast;
      'immature' the imported track was still in its CV-immature window at
                 launch (a CVMIGRATED for the gating track with no CVSWITCH
                 before launch, or first_cv_publish <= launch < switch);
      'mature'   launch at/after the gating track's first_use with no active
                 immature window (the MTR forecast);
      ''         undeterminable (no launch, or no migrated gating track, e.g.
                 cold has no import to classify)."""
    if lt is None or gid == "" or gid is None:
        return ""
    hf = d["handoff"].get(int(gid)) if str(gid).lstrip("-").isdigit() else None
    if not hf:
        return ""  # gating oncoming never migrated (cold): no migrated forecast
    def _int(v):
        try:
            return int(v)
        except (TypeError, ValueError):
            return None
    first_dst = _int(hf.get("first_dst_track_tick"))
    first_use = _int(hf.get("first_use_tick"))
    if first_dst is not None and lt < first_dst:
        return "coast"
    # CV-immature window for the gating track's destination track_ids.
    dest_ids = _dest_track_ids(text, d, gid)
    cvmig = {int(m.group(1)): int(m.group(2)) for m in re.finditer(
        r"\[CVMIGRATED\] tid=(\d+) first_cv_publish_tick=(\d+)", text)}
    cvsw = {int(m.group(1)): int(m.group(2)) for m in re.finditer(
        r"\[CVSWITCH\] tid=(\d+) switch_tick=(\d+)", text)}
    for tk in dest_ids:
        p = cvmig.get(tk)
        if p is not None and p <= lt and (tk not in cvsw or lt < cvsw[tk]):
            return "immature"
    if first_use is not None and lt >= first_use:
        return "mature"
    if first_dst is not None and lt >= first_dst:
        return "mature"
    return ""


def run_acc(args):
    rows = []
    for name in sorted(os.listdir(args.logdir)):
        if not name.endswith(".log"):
            continue
        stem = name[:-4]
        tm = ACC_TAG_RE.match(stem)
        if not tm:
            continue
        path = os.path.join(args.logdir, name)
        row = std_row(path, stem, args.machine, args.oncoming_speed,
                      args.trigger_dist, args.tag)
        if row is None:
            print(f"skip (no RUNROW, incomplete): {name}", file=sys.stderr)
            continue
        if row == "INVALID":
            print(f"skip (INVALID degenerate startup): {name}", file=sys.stderr)
            continue
        arm = tm.group("arm")
        commanded = tm.group("accel")  # tag float (a0.5/a2/a6); regex allows '.'
        text = _read(path)
        # warmabl (MIGRATION_ACCEL=0) is a distinct arm; the RUNROW mode is
        # 'warm', so relabel to keep (mode, trigger) arm identity separable.
        if arm == "warmabl" or "MIGRATION_ACCEL=0" in text:
            row["mode"] = "warm_ablation"
        d = kad.parse_log(path)
        oncoming, _amb = kad.identify_oncoming(d)
        lt = kad.launch_tick(d)
        gid, _note = kad.gating_track(d, oncoming, lt)
        # In the accel panel the accelerating oncoming often crosses and passes
        # before the (late) launch, so gating_track (tuned for the flow overtake
        # where the oncoming sits ahead at launch) finds nothing. Fall back to
        # the single migrated oncoming (the one CAMIGRATED fires for), else the
        # single GT-injected non-ego mover (num_actors=2).
        if gid == "":
            migrated = [c for c in oncoming if c in d["handoff"]]
            if len(migrated) == 1:
                gid = migrated[0]
            elif len(oncoming) == 1:
                gid = oncoming[0]
        # GT-measured accel + speed at the crossing (2 s pre-crossing window).
        actual_a, spd_cross = _gt_accel_speed(d, gid)
        # est_a AT LAUNCH (nearest assigned tick to the sustained launch) and
        # est_a_max (run maximum). Both blank for warm_ablation (term off) and
        # arms with no gating CAMIGRATED (kf/reactive/cold).
        est_a = ""
        est_a_max = ""
        if row["mode"] != "warm_ablation":
            gc = _gating_camigs(text, d, gid)
            if gc:
                if lt is not None:
                    pool = [c for c in gc if c[0] is not None] or gc
                    best = min(pool, key=lambda c: abs(
                        (c[0] if c[0] is not None else -1) - lt))
                else:
                    best = gc[-1]  # no launch: last emitted (nearest commit)
                est_a = round(best[2], 3)
                est_a_max = round(max(gc, key=lambda c: abs(c[2]))[2], 3)
        row["oncoming_accel"] = commanded  # back-compat alias of commanded_a
        row["commanded_a"] = commanded
        row["actual_a"] = actual_a
        row["speed_at_crossing"] = spd_cross
        row["est_a"] = est_a
        row["est_a_max"] = est_a_max
        row["mtr_implied_a"] = _mtr_implied_a(d, gid, lt)
        row["decision_path"] = _decision_path(d, gid, lt, text)
        row["motion"] = "maneuvering"
        rows.append(row)

    cols = STD_COLS + ACC_EXTRA
    _write(rows, cols, args.out)
    return rows


# ---------------------------------------------------------------------------
# HLN mode
# ---------------------------------------------------------------------------
def run_hln(args):
    by_n = {}
    for name in sorted(os.listdir(args.logdir)):
        if not name.endswith(".log"):
            continue
        stem = name[:-4]
        tm = HLN_TAG_RE.match(stem)
        if not tm:
            continue
        path = os.path.join(args.logdir, name)
        row = std_row(path, stem, args.machine, args.oncoming_speed,
                      args.trigger_dist, args.tag)
        if row is None:
            print(f"skip (no RUNROW, incomplete): {name}", file=sys.stderr)
            continue
        if row == "INVALID":
            print(f"skip (INVALID degenerate startup): {name}", file=sys.stderr)
            continue
        n = tm.group("n")
        arm = tm.group("arm")
        text = _read(path)
        if arm == "warmabl" or "MIGRATION_ACCEL=0" in text:
            row["mode"] = "warm_ablation"
        else:
            row["mode"] = ARM_MODE_LABEL.get(row.get("mode", ""), row.get("mode", ""))
        # ns3_lut_n from the tag; cross-check LAUNCHENV NS3_LUT_N.
        env = re.search(r"NS3_LUT_N=(\d+)", text)
        if env and env.group(1) != n:
            print(f"WARN {stem}: tag N={n} != LAUNCHENV NS3_LUT_N={env.group(1)}",
                  file=sys.stderr)
        row["ns3_lut_n"] = n
        ages = [float(m.group(1)) for m in AGEROW_RE.finditer(text)]
        row["age_med_ms"] = _pctl(ages, 0.50)
        row["age_p95_ms"] = _pctl(ages, 0.95)
        by_n.setdefault(n, []).append(row)

    cols = STD_COLS + HLN_EXTRA
    total = 0
    for n, rows in sorted(by_n.items()):
        out = os.path.join(args.outdir, f"frozen1l_hl_n{n}_rows.csv")
        _write(rows, cols, out)
        total += len(rows)
    if not by_n:
        print("no hln logs matched (none landed yet)", file=sys.stderr)
    return by_n


# ---------------------------------------------------------------------------
# FDE mode (Tyler's metric) and top-mode frozen-rate mode
# ---------------------------------------------------------------------------
def _gating_oncoming(d):
    """The gating oncoming carla_id: the single migrated oncoming (CAMIGRATED /
    HANDOFFROW target), else the single identified oncoming; None if neither."""
    onc, _amb = kad.identify_oncoming(d)
    mig = [c for c in onc if c in d["handoff"]]
    if len(mig) == 1:
        return mig[0]
    return onc[0] if onc else None


def _seed_from(stem):
    m = re.search(r"_s(\d+)", stem) or re.search(r"_r(\d+)", stem)
    return m.group(1) if m else ""


def _arm_label(stem, text, mode):
    if "warmabl" in stem or "MIGRATION_ACCEL=0" in text:
        return "warm_ablation"
    return ARM_MODE_LABEL.get(mode, mode)


def _parse_modesrows(text):
    """Parse [MODESROW] lines into {cid: {tick: {h: [(x,y,score), ...]}}}.
    Endpoints logged as nan (mode had no trajectory) keep their score with
    (None, None, score) so the mode count is preserved but is skipped downstream.
    """
    out = {}
    for line in text.splitlines():
        h = MODES_HDR_RE.search(line)
        if not h:
            continue
        tick, cid, hor = int(h.group(1)), int(h.group(3)), float(h.group(4))
        modes = []
        for mm in MODE_TUP_RE.finditer(line):
            xs, ys, ss = mm.group(2), mm.group(3), mm.group(4)
            if xs == "nan" or ys == "nan":
                modes.append((None, None, float(ss)))
            else:
                modes.append((float(xs), float(ys), float(ss)))
        out.setdefault(cid, {}).setdefault(tick, {})[hor] = modes
    return out


def _min_top2_fde(modes_at_h, truth):
    """FDE = min over the two highest-scoring modes of |endpoint - truth|, and
    the top (highest-scoring) mode's FDE. Returns (min_fde, top_fde) or
    (None, None) if no scorable mode/truth."""
    if truth is None:
        return None, None
    valid = [e for e in modes_at_h if e[0] is not None]
    if not valid:
        return None, None
    ranked = sorted(valid, key=lambda e: -e[2])
    top2 = ranked[:2]
    errs = [math.hypot(e[0] - truth[0], e[1] - truth[1]) for e in top2]
    top = math.hypot(ranked[0][0] - truth[0], ranked[0][1] - truth[1])
    return min(errs), top


def run_fde(args):
    """frozen1l_fde_rows.csv: per run, FDE = min of the two highest-scoring
    modes' final displacement at the horizon, mean over the first five
    post-commit refreshes. fde5_top_m is the single top mode's 5 s FDE."""
    rows = []
    for name in sorted(os.listdir(args.logdir)):
        if not name.endswith(".log"):
            continue
        stem = name[:-4]
        path = os.path.join(args.logdir, name)
        text = _read(path)
        if "[MODESROW]" not in text:
            continue  # not a 6-mode subset log
        core = kde.parse_log(path)
        if core is None or core == "INVALID":
            print(f"skip ({'no RUNROW' if core is None else 'INVALID'}): {name}",
                  file=sys.stderr)
            continue
        d = kad.parse_log(path)
        gid = _gating_oncoming(d)
        if gid is None:
            print(f"skip (no gating oncoming): {name}", file=sys.stderr)
            continue
        modes = _parse_modesrows(text).get(int(gid), {})
        if not modes:
            print(f"skip (no MODESROW for gating cid {gid}): {name}",
                  file=sys.stderr)
            continue
        hf = d["handoff"].get(int(gid), {})
        try:
            commit = int(hf.get("first_dst_track_tick"))
        except (TypeError, ValueError):
            commit = None
        ticks = sorted(t for t, hd in modes.items()
                       if 3.0 in hd and 5.0 in hd
                       and (commit is None or t >= commit))[:5]
        ev, gt = kad.actual_timeline(d, gid)
        f5, f3, f5top = [], [], []
        for T in ticks:
            m5, t5 = _min_top2_fde(
                modes[T][5.0], kad.lookup_actual(ev, gt, T + kad.horizon_ticks(5.0)))
            if m5 is not None:
                f5.append(m5)
                f5top.append(t5)
            m3, _ = _min_top2_fde(
                modes[T][3.0], kad.lookup_actual(ev, gt, T + kad.horizon_ticks(3.0)))
            if m3 is not None:
                f3.append(m3)
        if not f5:
            print(f"skip (no scorable 5 s refresh): {name}", file=sys.stderr)
            continue
        rows.append({
            "tag": stem,
            "arm": _arm_label(stem, text, core.get("mode", "")),
            "seed": _seed_from(stem),
            "fde5_m": round(sum(f5) / len(f5), 3),
            "fde3_m": round(sum(f3) / len(f3), 3) if f3 else "",
            "fde5_top_m": round(sum(f5top) / len(f5top), 3) if f5top else "",
            "eval_tag": args.tag,
            "machine": args.machine,
        })
    _write(rows, FDE_COLS, args.out)
    if not rows:
        print("no 6-mode ([MODESROW]) logs matched (subset not run yet)",
              file=sys.stderr)
    return rows


def run_topmode(args):
    """frozen1k_topmode_rows.csv: per freeze-1k headline run, the gating
    oncoming's forecast-block count and frozen count (top-mode endpoint moves
    < 1 m over 5 s while the actor moves > 10 m), plus outcome, so the paper
    macros compute the per-arm rate and the completed-vs-collided split."""
    rows = []
    for name in sorted(os.listdir(args.logdir)):
        if not name.endswith(".log"):
            continue
        stem = name[:-4]
        m = re.match(r"^hl_(?P<arm>.+)_r(?P<rep>\d+)$", stem)
        if not m:
            continue
        path = os.path.join(args.logdir, name)
        core = kde.parse_log(path)
        if core is None or core == "INVALID":
            print(f"skip ({'no RUNROW' if core is None else 'INVALID'}): {name}",
                  file=sys.stderr)
            continue
        collided = "YES" if int(core.get("eps", "0")) > 0 else "no"
        completed = core.get("completed", "no")
        d = kad.parse_log(path)
        gid = _gating_oncoming(d)
        if gid is None:
            print(f"skip (no gating oncoming): {name}", file=sys.stderr)
            continue
        ev, gt = kad.actual_timeline(d, gid)
        nb = nf = 0
        for r in sorted(d["eval"].get(gid, []), key=lambda r: r["tick"]):
            f025 = r["futs"].get(0.25)
            f5 = r["futs"].get(5.0)
            if f025 is None or f5 is None:
                continue
            a025 = kad.lookup_actual(ev, gt, r["tick"] + kad.horizon_ticks(0.25))
            a5 = kad.lookup_actual(ev, gt, r["tick"] + kad.horizon_ticks(5.0))
            if a025 is None or a5 is None:
                continue
            nb += 1  # classifiable at the 5 s window (both truths present)
            if (math.hypot(f5[0] - f025[0], f5[1] - f025[1]) < FROZEN_DISP_M
                    and math.hypot(a5[0] - a025[0], a5[1] - a025[1]) > ACTUAL_MOVE_M):
                nf += 1
        arm = {"handoversnap": "handover_snapshot"}.get(
            m.group("arm"), m.group("arm"))
        rows.append({
            "tag": stem, "arm": arm, "seed": m.group("rep"),
            "completed": completed, "collided": collided,
            "forecast_blocks": nb, "frozen_blocks": nf,
            "eval_tag": args.tag,
        })
    _write(rows, TOPMODE_COLS, args.out)
    return rows


# ---------------------------------------------------------------------------
# non-central blocks
# ---------------------------------------------------------------------------
def _age_at_tick(text, target):
    """AGEROW realized_age_ms nearest `target` tick (last AGEROW if target is
    None). The pipeline age the planner acted on at the maneuver decision."""
    best = bt = None
    for m in AGEROW_TICK_RE.finditer(text):
        t, a = int(m.group(1)), float(m.group(2))
        if target is None or bt is None or abs(t - target) < abs(bt - target):
            best, bt = a, t
    return best


def run_age_sweep(args):
    """frozen1l_age_sweep_rows.csv (schema row 47). realized_age_ms = TOTAL age
    at use = pipeline age at the maneuver decision (AGEROW realized_age_ms
    nearest the sustained do_ov launch) + the injected delay; baseline_age_ms is
    the pipeline age alone; inject_ms is the AOI_INJECT_MS knob."""
    rows = []
    for name in sorted(os.listdir(args.logdir)):
        if not name.endswith(".log"):
            continue
        stem = name[:-4]
        tm = FRESH_TAG_RE.match(stem)
        if not tm:
            continue
        path = os.path.join(args.logdir, name)
        row = std_row(path, stem, args.machine, args.oncoming_speed,
                      args.trigger_dist, args.tag)
        if row is None or row == "INVALID":
            print(f"skip ({'no RUNROW' if row is None else 'INVALID'}): {name}",
                  file=sys.stderr)
            continue
        text = _read(path)
        d = kad.parse_log(path)
        inj = int(tm.group("inj"))
        base = _age_at_tick(text, kad.launch_tick(d))
        row["scenario"] = tm.group("scn")
        row["inject_ms"] = inj
        row["baseline_age_ms"] = round(base, 1) if base is not None else ""
        row["realized_age_ms"] = round(base + inj, 1) if base is not None else ""
        rows.append(row)
    _write(rows, STD_COLS + AGESWEEP_EXTRA, args.out)
    return rows


def run_t25b(args):
    """frozen1l_t25b_rows.csv: khonsu_t25_land's row (STD + the nine T25 columns)
    plus shift_x_m. Reuses khonsu_t25_land.build_row verbatim; the t25b tag
    (t25b_{mode}_x{shift}_s{seed}) does not match its _TAG, so seed/rep and
    shift_x_m are set here (oncoming_speed falls back to LAUNCHENV 12)."""
    import khonsu_t25_land as kt25
    rows = []
    for name in sorted(os.listdir(args.logdir)):
        if not name.endswith(".log"):
            continue
        stem = name[:-4]
        tm = T25B_TAG_RE.match(stem)
        if not tm:
            continue
        path = os.path.join(args.logdir, name)
        r = kt25.build_row(path, args.machine)
        if r is None or r == "INVALID":
            print(f"skip ({'no RUNROW' if r is None else 'INVALID'}): {name}",
                  file=sys.stderr)
            continue
        r["seed"] = tm.group("seed")
        r["rep"] = tm.group("seed")
        text = _read(path)
        env = re.search(r"shift_x_m=(-?[0-9.]+)", text)
        if env and str(int(float(env.group(1)))) != tm.group("shift"):
            print(f"WARN {stem}: tag shift={tm.group('shift')} != "
                  f"RUNROW shift_x_m={env.group(1)}", file=sys.stderr)
        r["shift_x_m"] = tm.group("shift")
        rows.append(r)
    _write(rows, kt25.HEADER + ["shift_x_m"], args.out)
    return rows


def run_theta(args):
    """frozen1l_theta_rows.csv (STD + mtr_theta) and frozen1l_pdst_rows.csv (one
    row per [PDSTROW] prepare decision: tick, p_dst, mtr_theta, fired)."""
    rows, pdst = [], []
    for name in sorted(os.listdir(args.logdir)):
        if not name.endswith(".log"):
            continue
        stem = name[:-4]
        tm = THETA_TAG_RE.match(stem)
        if not tm:
            continue
        path = os.path.join(args.logdir, name)
        row = std_row(path, stem, args.machine, args.oncoming_speed,
                      args.trigger_dist, args.tag)
        if row is None or row == "INVALID":
            print(f"skip ({'no RUNROW' if row is None else 'INVALID'}): {name}",
                  file=sys.stderr)
            continue
        row["mtr_theta"] = tm.group("theta")
        rows.append(row)
        for m in PDSTROW_RE.finditer(_read(path)):
            pdst.append({"tag": stem, "tick": int(m.group(1)),
                         "p_dst": m.group(3), "mtr_theta": m.group(4),
                         "fired": m.group(5), "eval_tag": args.tag})
    _write(rows, STD_COLS + THETA_EXTRA, args.out)
    _write(pdst, PDST_COLS, args.pdst_out)
    return rows


def run_faults(args):
    """frozen1l_faults_rows.csv (schema row 58). double_emission_ms + recovered
    from [FAULTROW]; stale_consumed + fallback from the daemon's per-injection
    [FAULTPATH]. The clean-handoff control (flt_control_*) has fault=none."""
    rows = []
    for name in sorted(os.listdir(args.logdir)):
        if not name.endswith(".log"):
            continue
        stem = name[:-4]
        tm = FLT_TAG_RE.match(stem)
        if not tm:
            continue
        path = os.path.join(args.logdir, name)
        core = kde.parse_log(path)
        if core is None or core == "INVALID":
            print(f"skip ({'no RUNROW' if core is None else 'INVALID'}): {name}",
                  file=sys.stderr)
            continue
        text = _read(path)
        fr = FAULTROW_RE.search(text)
        fp = FAULTPATH_RE.search(text)
        de = fr.group(2) if fr else ""
        rec = (fr.group(3) if fr and fr.group(3) else
               (fp.group(5) if fp else ""))
        fault = tm.group("fault")
        rows.append({
            "tag": stem,
            "fault": "none" if fault == "control" else fault,
            "epochs": tm.group("ep"),
            "rep": tm.group("seed"),
            "double_emission_ms": de,
            "stale_consumed": fp.group(3) if fp else "",
            "fallback": fp.group(4) if fp else "",
            "recovered": rec,
            "eval_tag": args.tag,
        })
    _write(rows, FAULTS_COLS, args.out)
    return rows


def run_repl(args):
    """frozen1l_repl_period_rows.csv: STD + repl_period_s, bytes_per_crossing
    (RUNROW bytes / number of HANDOFFROW crossings), warm_before_first_use
    (per-run YES fraction), age_at_decision_ms (AGEROW realized at launch)."""
    rows = []
    for name in sorted(os.listdir(args.logdir)):
        if not name.endswith(".log"):
            continue
        stem = name[:-4]
        tm = REPL_TAG_RE.match(stem)
        if not tm:
            continue
        path = os.path.join(args.logdir, name)
        row = std_row(path, stem, args.machine, args.oncoming_speed,
                      args.trigger_dist, args.tag)
        if row is None or row == "INVALID":
            print(f"skip ({'no RUNROW' if row is None else 'INVALID'}): {name}",
                  file=sys.stderr)
            continue
        text = _read(path)
        d = kad.parse_log(path)
        row["repl_period_s"] = tm.group("period")
        by = int(row.get("by", "0") or 0)
        ncross = len(re.findall(r"\[HANDOFFROW\]", text))
        row["bytes_per_crossing"] = round(by / ncross, 1) if ncross else ""
        wb = [1 if m == "YES" else 0 for m in HANDOFF_WBFU_RE.findall(text)]
        row["warm_before_first_use"] = round(sum(wb) / len(wb), 3) if wb else ""
        a = _age_at_tick(text, kad.launch_tick(d))
        row["age_at_decision_ms"] = round(a, 1) if a is not None else ""
        rows.append(row)
    _write(rows, STD_COLS + REPL_EXTRA, args.out)
    return rows


# ---------------------------------------------------------------------------
def _write(rows, cols, out):
    fh = open(out, "w", newline="") if out else sys.stdout
    w = csv.DictWriter(fh, fieldnames=cols, extrasaction="ignore")
    w.writeheader()
    for r in rows:
        w.writerow(r)
    if out:
        fh.close()
        print(f"{len(rows)} rows -> {out}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    a = sub.add_parser("acc", help="ACCEL maneuvering-panel lander")
    a.add_argument("logdir")
    a.add_argument("-o", "--out",
                   default="docs/kb/data/relay_eval_2026_08/frozen1l_acc_rows.csv")
    a.add_argument("--tag", default="khonsu-eval-freeze-1l")
    a.add_argument("--machine", default="atlas")
    a.add_argument("--oncoming-speed", dest="oncoming_speed", default="12")
    a.add_argument("--trigger-dist", dest="trigger_dist", default="300")
    a.set_defaults(func=run_acc)

    h = sub.add_parser("hln", help="loaded-headline lander (per N)")
    h.add_argument("logdir")
    h.add_argument("--outdir",
                   default="docs/kb/data/relay_eval_2026_08")
    h.add_argument("--tag", default="khonsu-eval-freeze-1l")
    h.add_argument("--machine", default="atlas")
    h.add_argument("--oncoming-speed", dest="oncoming_speed", default="12")
    h.add_argument("--trigger-dist", dest="trigger_dist", default="300")
    h.set_defaults(func=run_hln)

    f = sub.add_parser("fde", help="6-mode FDE lander (min of two top modes)")
    f.add_argument("logdir")
    f.add_argument("-o", "--out",
                   default="docs/kb/data/relay_eval_2026_08/frozen1l_fde_rows.csv")
    f.add_argument("--tag", default="khonsu-eval-freeze-1l")
    f.add_argument("--machine", default="atlas")
    f.set_defaults(func=run_fde)

    t = sub.add_parser("topmode",
                       help="freeze-1k top-mode frozen-rate lander")
    t.add_argument("logdir")
    t.add_argument("-o", "--out",
                   default="docs/kb/data/relay_eval_2026_08/frozen1k_topmode_rows.csv")
    t.add_argument("--tag", default="khonsu-eval-freeze-1k")
    t.set_defaults(func=run_topmode)

    _KB = "docs/kb/data/relay_eval_2026_08"

    def _common(sp, default_out):
        sp.add_argument("logdir")
        sp.add_argument("-o", "--out", default=f"{_KB}/{default_out}")
        sp.add_argument("--tag", default="khonsu-eval-freeze-1l")
        sp.add_argument("--machine", default="atlas")
        sp.add_argument("--oncoming-speed", dest="oncoming_speed", default="12")
        sp.add_argument("--trigger-dist", dest="trigger_dist", default="300")

    ags = sub.add_parser("age_sweep", help="freshness AOI sweep lander")
    _common(ags, "frozen1l_age_sweep_rows.csv")
    ags.set_defaults(func=run_age_sweep)

    tb = sub.add_parser("t25b", help="geometry shift sweep lander (reuses t25)")
    _common(tb, "frozen1l_t25b_rows.csv")
    tb.set_defaults(func=run_t25b)

    th = sub.add_parser("theta", help="MTR-theta sweep + p_dst trace lander")
    _common(th, "frozen1l_theta_rows.csv")
    th.add_argument("--pdst-out", dest="pdst_out",
                    default=f"{_KB}/frozen1l_pdst_rows.csv")
    th.set_defaults(func=run_theta)

    fl = sub.add_parser("faults", help="fault-injection lander")
    _common(fl, "frozen1l_faults_rows.csv")
    fl.set_defaults(func=run_faults)

    rp = sub.add_parser("repl", help="repl_final REPL_PERIOD sweep lander")
    _common(rp, "frozen1l_repl_period_rows.csv")
    rp.set_defaults(func=run_repl)

    args = ap.parse_args()
    for _attr in ("out", "pdst_out"):
        _v = getattr(args, _attr, None)
        if _v:
            _d = os.path.dirname(os.path.abspath(_v))
            if _d:
                os.makedirs(_d, exist_ok=True)
    if getattr(args, "outdir", None):
        os.makedirs(args.outdir, exist_ok=True)
    args.func(args)


if __name__ == "__main__":
    main()
