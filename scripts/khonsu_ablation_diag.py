#!/usr/bin/env python3
"""Record-depth ablation diagnostics for the RELAY/Khonsu freeze-1k F-block arms.

POST-HOC log extractor. Parses existing simulation .log files only. It does not
run the simulator, touch a GPU, or modify any sim code.

One CSV row per cell (per run/log). Cells are selected by filename-stem regex
(default ^fa_ , ^fb_edgewarp , ^hl_ ; override with --glob for a shell glob or
--regex for a stem regex, e.g. to validate against the k_* cand2_1k cells).

Usage:
    python scripts/khonsu_ablation_diag.py <logdir> \
        [-o OUT.csv] [--tag TAG] [--glob 'k_*.log'] [--regex '^k_']

Output default: docs/kb/data/relay_eval_2026_08/frozen1k_ablation_diag.csv
The whole logdir is re-parsed on each run (overwrite), keyed by cell+tag.

Marker formats consumed (see task spec / idfix_wt/evaluation_outputs/cand2_1k):
    [LAUNCHENV] ... MIGRATION_MODE=warm [MIGRATION_HIST=N] ...
    [RUNROW] mode=warm ... episodes=0 ...
    [EVAL] tick=T arm=A tag=G Track K -> name (CARLA ID C) + Actual/Future block
    [GT INJECT DBG] frame=F aid=C type=... world=(x,y) yaw_anchor_rad=...
    [COASTROW] tid=T vel_mps=(vx,vy) |v|=V spf=.. steps=.. proj=(x,y)
    [EGO-DBG] tick=T pos=(x,y) spd=.. ttc=.. ... do_ov=True|False ...
    [HANDOFFROW] npc=C prepare_tick=.. first_dst_track_tick=.. first_use_tick=U ...
    [PRED COLLISION] carla_id=C track_id=T TTC=.. dist=.. obs_pos=(x,y) ego_pos=(x,y)

launch_tick is the SUSTAINED overtake commit: the first tick of the first run
of >= 4 consecutive EGO-DBG do_ov=True samples (EGO-DBG is emitted every 5
ticks). This skips the early transient do_ov that clears and returns the commit
that persists until the recheck aborts or the pass completes. All launch-derived
fields (true_gap_at_launch, gating_*, blind) are computed at this tick.

Gating / mechanism columns:
    gating_track_id             oncoming track that gates the overtake at launch
    gating_first_use_tick       HANDOFFROW first_use_tick for that track (blank if
                                the arm does not migrate, e.g. cold)
    gating_first_detection_tick first EVAL tick that track is seen at destination
    blind                       1 if launch_tick < gating_first_use_tick (cold: 1
                                by no-migration; clear-gap launch: 0)
    collision_partner_resolved  CARLA id of the obstacle contacted, resolved as the
                                non-ego actor nearest the ego at collision onset
The headline runs (--regex '^hl_') additionally emit arm and collision_partner
(the raw carla_id of the last [PRED COLLISION] line; usually the ego, kept only
for continuity - use collision_partner_resolved for the obstacle identity).
"""

import argparse
import csv
import fnmatch
import glob as globmod
import math
import os
import re
import statistics

# Physics constants from the scenario config (fixed_delta_seconds=0.05,
# edge_dt=0.2). Horizons in seconds map to ticks via /0.05.
SIM_DT = 0.05
TRUCK_X = 278.0          # conflict x, the stopped firetruck (CID carlacola)
ONCOMING_Y_LO = 196.5    # oncoming lane band (y ~ 199); ego/truck lane is y ~ 195
ONCOMING_Y_HI = 202.5
ONCOMING_Y = 199.0       # oncoming lane centre
ONCOMING_BAND = 3.0      # |Actual_y - 199| < 3 selects the oncoming lane
GATING_WINDOW = 8        # EVAL ticks around launch to sample gating candidate pose
SUSTAIN_SAMPLES = 4      # do_ov must hold this many consecutive EGO-DBG samples
COLL_TOL_TICKS = 10      # actor-position tolerance around the collision onset tick
HORIZONS = [0.25, 0.50, 1.25, 3.00, 5.00]
ACTUAL_TOL_TICKS = 4     # nearest-actual tolerance for the FDE lookup

_ANSI = re.compile(r"\x1b\[[0-9;]*m")


def strip_ansi(s):
    return _ANSI.sub("", s)


def horizon_ticks(h):
    return int(round(h / SIM_DT))


# ---------------------------------------------------------------------------
# Parsing
# ---------------------------------------------------------------------------
def parse_log(path):
    """Return a dict of parsed structures for one log file."""
    d = {
        "launchenv": {},
        "runrow": {},
        "eval": {},        # cid -> list of dicts {tick, ax, ay, px, py, speed, futs}
        "gt": {},          # aid -> {frame: (x, y)}
        "coast": [],       # list of (ctx_tick, tid, v)
        "ego": {},         # tick -> (x, y, spd, ttc, do_ov)
        "handoff": {},     # migrated npc cid -> HANDOFFROW field dict
        "pred_coll_ids": [],  # carla_id of each [PRED COLLISION] line, in order
        "collision_onset": None,  # context tick of the first ego collision-sensor hit
        "eval_tag": "",
        "eval_arm": "",
    }
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        text = fh.read()

    # LAUNCHENV (first matching line)
    m = re.search(r"\[LAUNCHENV\]([^\n]*)", text)
    if m:
        for kv in re.findall(r"(\w+)=(\S+)", m.group(1)):
            d["launchenv"][kv[0]] = kv[1]

    # RUNROW (last matching line wins; there is normally one)
    for m in re.finditer(r"\[RUNROW\]([^\n]*)", text):
        row = {}
        for kv in re.findall(r"(\w+)=(\S+)", m.group(1)):
            row[kv[0]] = kv[1]
        d["runrow"] = row

    # HANDOFFROW: one per migrated npc. first_use_tick is the tick the
    # destination edge first USED the migrated track for planning.
    for m in re.finditer(r"\[HANDOFFROW\]([^\n]*)", text):
        fields = dict(re.findall(r"(\w+)=(\S+)", m.group(1)))
        npc = fields.get("npc")
        if npc is not None:
            try:
                d["handoff"][int(npc)] = fields
            except ValueError:
                pass

    # PRED COLLISION: carla_id of the predicted-collision events, in order.
    # NOTE: carla_id here is the vehicle running the check (the ego, usually
    # 197), not the obstacle; obstacle identity lives in track_id / obs_pos.
    for m in re.finditer(r"\[PRED COLLISION\][^\n]*?carla_id=(-?\d+)", text):
        d["pred_coll_ids"].append(m.group(1))

    # GT INJECT DBG
    for m in re.finditer(
        r"\[GT INJECT DBG\] frame=(\d+) aid=(\d+) type=\S+ "
        r"world=\(([\-0-9.]+),([\-0-9.]+)\)",
        text,
    ):
        frame = int(m.group(1))
        aid = int(m.group(2))
        d["gt"].setdefault(aid, {})[frame] = (float(m.group(3)), float(m.group(4)))

    # EVAL blocks (split so each block is self-contained)
    for block in re.split(r"(?=\[EVAL\] tick=)", text):
        hm = re.match(
            r"\[EVAL\] tick=(\d+) arm=(\S+) tag=(\S+) Track (\d+) -> \S+ "
            r"\(CARLA ID (\d+)\)",
            block,
        )
        if not hm:
            continue
        tick = int(hm.group(1))
        cid = int(hm.group(5))
        if not d["eval_tag"]:
            d["eval_arm"] = hm.group(2)
            d["eval_tag"] = hm.group(3)
        am = re.search(r"Actual:\s*x=\s*([\-0-9.]+),\s*y=\s*([\-0-9.]+)", block)
        pm = re.search(r"Predicted:\s*x=\s*([\-0-9.]+),\s*y=\s*([\-0-9.]+)", block)
        sm = re.search(r"Actual Speed:\s*([\-0-9.]+)", block)
        futs = {}
        for fm in re.findall(
            r"\+([0-9.]+)s: \(([\-0-9.]+), ([\-0-9.]+)\)", block
        ):
            futs[float(fm[0])] = (float(fm[1]), float(fm[2]))
        if not am:
            continue
        rec = {
            "tick": tick,
            "ax": float(am.group(1)),
            "ay": float(am.group(2)),
            "px": float(pm.group(1)) if pm else None,
            "py": float(pm.group(2)) if pm else None,
            "speed": float(sm.group(1)) if sm else None,
            "futs": futs,
        }
        d["eval"].setdefault(cid, []).append(rec)

    # Line-ordered pass for COASTROW (needs contextual tick) and EGO-DBG.
    cur_tick = None
    for raw in text.splitlines():
        s = strip_ansi(raw)
        if "[EGO-DBG]" in s:
            m = re.search(
                r"\[EGO-DBG\] tick=(\d+) pos=\(([\-0-9.]+),([\-0-9.]+)\) "
                r"spd=([\-0-9.]+) ttc=([\-0-9.]+).*?do_ov=(True|False)",
                s,
            )
            if m:
                t = int(m.group(1))
                cur_tick = t
                d["ego"][t] = (
                    float(m.group(2)),
                    float(m.group(3)),
                    float(m.group(4)),
                    float(m.group(5)),
                    m.group(6) == "True",
                )
            continue
        if "[EVAL] tick=" in s:
            m = re.search(r"\[EVAL\] tick=(\d+)", s)
            if m:
                cur_tick = int(m.group(1))
            continue
        if "[TRACKER DBG]" in s:
            m = re.search(r"tick=(\d+)", s)
            if m:
                cur_tick = int(m.group(1))
            continue
        if "[COASTROW]" in s:
            m = re.search(r"\[COASTROW\] tid=(\d+).*\|v\|=([\-0-9.]+)", s)
            if m:
                d["coast"].append((cur_tick, int(m.group(1)), float(m.group(2))))
            continue
        # Ego collision-sensor onset: the first vehicle_manager "Collision"
        # warning. The line carries no tick, so use the running context tick.
        if d["collision_onset"] is None and "vehicle_manager" in s \
                and s.rstrip().endswith("Collision"):
            d["collision_onset"] = cur_tick
    return d


# ---------------------------------------------------------------------------
# Derived quantities
# ---------------------------------------------------------------------------
def identify_oncoming(d):
    """Oncoming = non-ego tracked CIDs starting in the oncoming lane (y~199)
    with net +x motion. Falls back to all tracked CIDs if none qualify."""
    oncoming = []
    for cid, recs in d["eval"].items():
        recs = sorted(recs, key=lambda r: r["tick"])
        y0 = recs[0]["ay"]
        x0 = recs[0]["ax"]
        xlast = recs[-1]["ax"]
        if ONCOMING_Y_LO <= y0 <= ONCOMING_Y_HI and (xlast - x0) > 5.0:
            oncoming.append(cid)
    if oncoming:
        return oncoming, False
    return list(d["eval"].keys()), True  # ambiguous


def actual_timeline(d, cid):
    """Return (eval_map tick->pos, gt_map frame->pos) for a cid."""
    ev = {}
    for r in sorted(d["eval"].get(cid, []), key=lambda r: r["tick"]):
        ev[r["tick"]] = (r["ax"], r["ay"])
    gt = d["gt"].get(cid, {})
    return ev, gt


def lookup_actual(ev, gt, target_tick):
    """Actual position at target_tick: nearest EVAL actual within tolerance,
    else GT world at frame ~= target_tick within tolerance, else None."""
    best = None
    if ev:
        t = min(ev, key=lambda k: abs(k - target_tick))
        if abs(t - target_tick) <= ACTUAL_TOL_TICKS:
            best = ev[t]
    if best is None and gt:
        f = min(gt, key=lambda k: abs(k - target_tick))
        if abs(f - target_tick) <= ACTUAL_TOL_TICKS:
            best = gt[f]
    return best


def fde_ade(d, cids):
    """Compute pooled and per-object endpoint (FDE) errors per horizon.

    Returns (pooled, per_obj) where:
      pooled[h] = list of per-tick endpoint errors at horizon h over all cids
      per_obj[cid][h] = list of per-tick endpoint errors at horizon h
    """
    pooled = {h: [] for h in HORIZONS}
    per_obj = {}
    for cid in cids:
        ev, gt = actual_timeline(d, cid)
        per_obj[cid] = {h: [] for h in HORIZONS}
        for r in sorted(d["eval"].get(cid, []), key=lambda r: r["tick"]):
            T = r["tick"]
            for h, fpos in r["futs"].items():
                if h not in pooled:
                    continue
                target = T + horizon_ticks(h)
                apos = lookup_actual(ev, gt, target)
                if apos is None:
                    continue
                err = math.hypot(fpos[0] - apos[0], fpos[1] - apos[1])
                pooled[h].append(err)
                per_obj[cid][h].append(err)
    return pooled, per_obj


def mean_or_blank(vals):
    return round(statistics.fmean(vals), 3) if vals else ""


def ade_upto(pooled, H):
    """ADE up to horizon H: mean over all sampled horizons h<=H of endpoint
    errors, pooled over ticks/objects."""
    vals = []
    for h in HORIZONS:
        if h <= H + 1e-9:
            vals.extend(pooled.get(h, []))
    return mean_or_blank(vals)


def min_over_oncoming(per_obj, H, metric):
    """metric='fde' -> per-object mean endpoint error at H;
    metric='ade' -> per-object ADE up to H. Return min across objects."""
    out = []
    for cid, hd in per_obj.items():
        if metric == "fde":
            vals = hd.get(H, [])
            if vals:
                out.append(statistics.fmean(vals))
        else:
            vals = []
            for h in HORIZONS:
                if h <= H + 1e-9:
                    vals.extend(hd.get(h, []))
            if vals:
                out.append(statistics.fmean(vals))
    return round(min(out), 3) if out else ""


def compute_depth(cell, launchenv):
    if "MIGRATION_HIST" in launchenv:
        return launchenv["MIGRATION_HIST"]
    name = cell.lower()
    if "one_frame" in name:
        return "1"
    if "hist10" in name:
        return "10"
    if "hist5" in name:
        return "5"
    if "hist2" in name:
        return "2"
    if "latest_box" in name or "latestbox" in name:
        return "latestbox"
    if "nothing" in name:
        return "cold"
    if "full" in name:
        return "none"
    return ""


def launch_tick(d):
    """Sustained-overtake launch tick.

    Criterion: the first tick of the first run of >= SUSTAIN_SAMPLES (4)
    consecutive EGO-DBG samples with do_ov=True. EGO-DBG is emitted every 5
    ticks, so this is the first do_ov commit that holds for ~4 samples instead
    of the early transient that clears (e.g. the ego briefly raising do_ov far
    from the truck, tick 105 in some cold runs). Equivalently: the do_ov run
    that persists until the recheck aborts or the pass completes. Returns None
    if do_ov never holds for SUSTAIN_SAMPLES consecutive samples."""
    run = 0
    start = None
    for t in sorted(d["ego"]):
        if d["ego"][t][4]:
            if run == 0:
                start = t
            run += 1
            if run >= SUSTAIN_SAMPLES:
                return start
        else:
            run = 0
            start = None
    return None


def min_ttc(d):
    vals = [v[3] for v in d["ego"].values() if v[3] != 1000.0]
    return round(min(vals), 3) if vals else ""


def vel_est_at_commit(d, lt):
    if not d["coast"]:
        return ""
    if lt is not None:
        cand = [c for c in d["coast"] if c[0] is not None]
        if cand:
            best = min(cand, key=lambda c: abs(c[0] - lt))
            return round(best[2], 3)
    return round(d["coast"][0][2], 3)


def true_gap_at_launch(d, lt, oncoming):
    if lt is None or lt not in d["ego"]:
        return ""
    ex, ey = d["ego"][lt][0], d["ego"][lt][1]
    dists = []
    for cid in oncoming:
        ev, gt = actual_timeline(d, cid)
        apos = lookup_actual(ev, gt, lt)
        if apos is None and ev:
            t = min(ev, key=lambda k: abs(k - lt))  # relax tol for the gap
            apos = ev[t]
        if apos is None and gt:
            f = min(gt, key=lambda k: abs(k - lt))
            apos = gt[f]
        if apos is not None:
            dists.append(math.hypot(ex - apos[0], ey - apos[1]))
    return round(min(dists), 3) if dists else ""


def pred_arrival_tick(d, oncoming):
    """First tick at which an oncoming forecast reaches the conflict x (=truck
    x). Prefer a future-prediction sample that crosses; else project from
    Actual Speed at the last approaching tick. Best-effort, may be blank."""
    arrivals = []
    for cid in oncoming:
        recs = sorted(d["eval"].get(cid, []), key=lambda r: r["tick"])
        crossed = None
        for r in recs:
            if r["ax"] >= TRUCK_X:
                break
            for h in sorted(r["futs"]):
                fx = r["futs"][h][0]
                if fx >= TRUCK_X:
                    crossed = r["tick"] + horizon_ticks(h)
                    break
            if crossed is not None:
                break
        if crossed is not None:
            arrivals.append(crossed)
            continue
        # Speed projection from the last approaching tick.
        proj = None
        for r in recs:
            if r["ax"] < TRUCK_X and r["speed"] and r["speed"] > 0.5:
                proj = r["tick"] + (TRUCK_X - r["ax"]) / (r["speed"] * SIM_DT)
        if proj is not None:
            arrivals.append(int(round(proj)))
    return min(arrivals) if arrivals else ""


def gating_track(d, oncoming, lt):
    """Oncoming track that gates the overtake sight-distance check at launch.

    At launch_tick (+/- GATING_WINDOW EVAL ticks), among oncoming-lane tracks
    (|Actual_y - 199| < ONCOMING_BAND), prefer the one AHEAD of the ego
    (Actual_x <= ego_x, since the ego heads -x) nearest to the ego (largest
    Actual_x <= ego_x). If none is ahead (all have passed to higher x), pick
    the nearest one at higher x (smallest Actual_x > ego_x) and mark it
    'approaching'. If no oncoming track is in the window, blank.

    The candidate set is the oncoming set (identify_oncoming), which excludes
    the ego and the stationary truck by lane+motion, so no CARLA id is
    hard-coded here. Returns (gating_cid_or_blank, note).
    note in {"ahead", "approaching", "no_oncoming", ""}.
    """
    if lt is None:
        return "", ""
    if lt in d["ego"]:
        ego_x = d["ego"][lt][0]
    elif d["ego"]:
        t = min(d["ego"], key=lambda k: abs(k - lt))
        ego_x = d["ego"][t][0]
    else:
        return "", ""

    cand = {}
    for cid in oncoming:
        recs = d["eval"].get(cid, [])
        if not recs:
            continue
        best = min(recs, key=lambda r: abs(r["tick"] - lt))
        if abs(best["tick"] - lt) > GATING_WINDOW:
            continue
        if abs(best["ay"] - ONCOMING_Y) >= ONCOMING_BAND:
            continue
        cand[cid] = (best["ax"], best["ay"])

    if not cand:
        return "", "no_oncoming"

    ahead = {c: v for c, v in cand.items() if v[0] <= ego_x}
    if ahead:
        gid = max(ahead, key=lambda c: ahead[c][0])
        return gid, "ahead"
    gid = min(cand, key=lambda c: cand[c][0])
    return gid, "approaching"


def first_detection_tick(d, cid):
    """First EVAL tick at which cid appears (earliest tracked observation)."""
    recs = d["eval"].get(cid, [])
    return min((r["tick"] for r in recs), default="")


def gating_first_use_tick(d, cid):
    """HANDOFFROW first_use_tick for the migrated cid; blank if not migrated."""
    hf = d["handoff"].get(cid)
    if not hf or "first_use_tick" not in hf:
        return ""
    try:
        return int(hf["first_use_tick"])
    except (TypeError, ValueError):
        return ""


def derive_arm(cell):
    """Headline arm from the cell stem hl_<arm>_r<N>."""
    m = re.match(r"hl_(.+)_r\d+$", cell)
    if not m:
        return ""
    tok = m.group(1)
    return {"handoversnap": "handover_snapshot"}.get(tok, tok)


def blind_flag(arm, lt, gid, gfu):
    """1 if the overtake launched before the gating oncoming track was usable.

    - cold never migrates a track, so any launch is unassisted -> blind=1.
    - no gating track in the launch window (clear gap) -> blind=0.
    - gating track present but never migrated (blank first_use) -> blind=1.
    - otherwise blind = launch_tick < gating_first_use_tick.
    """
    if arm == "cold":
        return 1
    if gid == "" or lt is None or lt == "":
        return 0
    if gfu == "" or gfu is None:
        return 1
    try:
        return 1 if int(lt) < int(gfu) else 0
    except (TypeError, ValueError):
        return 0


def identify_ego_cid(d):
    """CARLA id of the ego: the EVAL track whose Actual trajectory matches the
    EGO-DBG positions. Returns None if no track tracks the ego closely."""
    if not d["ego"] or not d["eval"]:
        return None
    best = None
    for cid, recs in d["eval"].items():
        ds = [
            math.hypot(r["ax"] - d["ego"][r["tick"]][0],
                       r["ay"] - d["ego"][r["tick"]][1])
            for r in recs if r["tick"] in d["ego"]
        ]
        if len(ds) >= 5:
            mean = statistics.fmean(ds)
            if best is None or mean < best[1]:
                best = (cid, mean)
    return best[0] if best and best[1] < 3.0 else None


def resolve_collision_partner(d, ego_cid=None):
    """CARLA id of the obstacle the ego contacted, from the collision onset.

    Resolved geometrically: at the first ego collision-sensor tick, take the
    non-ego actor nearest the ego position (Actual from EVAL, else GT). This is
    the physical partner and covers rear-ends of the stopped truck as well as
    oncoming hits. The [PRED COLLISION] line is not used for this: its carla_id
    is the checking vehicle (usually the ego 197), its track_id often does not
    map at the collision tick, and its last occurrence can post-date the event.
    Returns "" if there is no collision or no actor near the ego.
    """
    onset = d.get("collision_onset")
    if onset is None:
        return ""
    if onset in d["ego"]:
        ex, ey = d["ego"][onset][0], d["ego"][onset][1]
    elif d["ego"]:
        t = min(d["ego"], key=lambda k: abs(k - onset))
        ex, ey = d["ego"][t][0], d["ego"][t][1]
    else:
        return ""

    cand = []  # (cid, x, y)
    for cid, recs in d["eval"].items():
        best = min(recs, key=lambda r: abs(r["tick"] - onset)) if recs else None
        if best is not None and abs(best["tick"] - onset) <= COLL_TOL_TICKS:
            cand.append((cid, best["ax"], best["ay"]))
            continue
        gt = d["gt"].get(cid, {})
        if gt:
            f = min(gt, key=lambda k: abs(k - onset))
            if abs(f - onset) <= COLL_TOL_TICKS:
                cand.append((cid, gt[f][0], gt[f][1]))
    if not cand:
        return ""

    def dist(c):
        return math.hypot(c[1] - ex, c[2] - ey)

    # Drop the ego's own track. Prefer the trajectory-matched ego cid; fall
    # back to dropping the actor sitting on the ego position.
    if ego_cid is not None:
        others = [c for c in cand if c[0] != ego_cid]
    else:
        ego_c = min(cand, key=dist)
        others = [c for c in cand if c[0] != ego_c[0]] if dist(ego_c) < 2.0 else cand
    if not others:
        return ""
    return min(others, key=dist)[0]


# ---------------------------------------------------------------------------
# Per-cell row
# ---------------------------------------------------------------------------
def build_row(path, tag_override):
    cell = os.path.splitext(os.path.basename(path))[0]
    d = parse_log(path)
    le = d["launchenv"]
    rr = d["runrow"]

    mode = rr.get("mode") or le.get("MIGRATION_MODE") or d.get("eval_arm") or ""
    depth = compute_depth(cell, le)
    sm = re.search(r"_s(\d+)", cell)
    seed = sm.group(1) if sm else le.get("SEED", "")
    tag = tag_override or d.get("eval_tag") or ""

    episodes = rr.get("episodes", "")
    try:
        collided = "True" if int(episodes) >= 1 else "False"
    except (TypeError, ValueError):
        collided = ""

    oncoming, ambiguous = identify_oncoming(d)
    pooled, per_obj = fde_ade(d, oncoming)

    lt = launch_tick(d)

    gid, _gnote = gating_track(d, oncoming, lt)
    gating_track_id = gid if gid != "" else ""
    gfu = gating_first_use_tick(d, gid) if gid != "" else ""
    gfd = first_detection_tick(d, gid) if gid != "" else ""

    arm = derive_arm(cell)
    blind = blind_flag(arm, lt, gating_track_id, gfu)
    partner_resolved = resolve_collision_partner(d, identify_ego_cid(d))

    row = {
        "cell": cell,
        "mode": mode,
        "depth": depth,
        "seed": seed,
        "tag": tag,
        "vel_est_at_commit": vel_est_at_commit(d, lt),
        "vel_est_first_coast": round(d["coast"][0][2], 3) if d["coast"] else "",
        "ade_3s": ade_upto(pooled, 3.0),
        "fde_3s": mean_or_blank(pooled.get(3.0, [])),
        "ade_5s": ade_upto(pooled, 5.0),
        "fde_5s": mean_or_blank(pooled.get(5.0, [])),
        "ade_3s_min_oncoming": min_over_oncoming(per_obj, 3.0, "ade"),
        "fde_3s_min_oncoming": min_over_oncoming(per_obj, 3.0, "fde"),
        "ade_5s_min_oncoming": min_over_oncoming(per_obj, 5.0, "ade"),
        "fde_5s_min_oncoming": min_over_oncoming(per_obj, 5.0, "fde"),
        "oncoming_cids": "|".join(str(c) for c in sorted(oncoming)),
        "oncoming_ambiguous": "1" if ambiguous else "0",
        "pred_arrival_tick_at_conflict": pred_arrival_tick(d, oncoming),
        "launch_tick": lt if lt is not None else "",
        "true_gap_at_launch": true_gap_at_launch(d, lt, oncoming),
        "min_ttc": min_ttc(d),
        "episodes": episodes,
        "collided": collided,
        "gating_track_id": gating_track_id,
        "gating_first_use_tick": gfu,
        "gating_first_detection_tick": gfd,
        "blind": blind,
        "collision_partner_resolved": partner_resolved,
        # Headline-only extras (dropped from the ablation CSV via fieldnames).
        "arm": arm,
        "collision_partner": d["pred_coll_ids"][-1] if d["pred_coll_ids"] else "",
    }
    return row


COLUMNS = [
    "cell", "mode", "depth", "seed", "tag", "vel_est_at_commit", "vel_est_first_coast",
    "ade_3s", "fde_3s", "ade_5s", "fde_5s",
    "ade_3s_min_oncoming", "fde_3s_min_oncoming",
    "ade_5s_min_oncoming", "fde_5s_min_oncoming",
    "oncoming_cids", "oncoming_ambiguous",
    "pred_arrival_tick_at_conflict", "launch_tick",
    "true_gap_at_launch", "min_ttc", "episodes", "collided",
    "gating_track_id", "gating_first_use_tick", "gating_first_detection_tick",
    "blind", "collision_partner_resolved",
]

# Extra columns emitted only for the headline arms (all-hl_ selections).
HEADLINE_EXTRA = ["arm", "collision_partner"]

DEFAULT_REGEX = r"^(fa_|fb_edgewarp|hl_)"
DEFAULT_OUT = "docs/kb/data/relay_eval_2026_08/frozen1k_ablation_diag.csv"


def select_logs(logdir, user_glob, user_regex):
    paths = sorted(globmod.glob(os.path.join(logdir, "*.log")))
    out = []
    rx = re.compile(user_regex) if user_regex else re.compile(DEFAULT_REGEX)
    for p in paths:
        stem = os.path.splitext(os.path.basename(p))[0]
        if user_glob:
            if fnmatch.fnmatch(os.path.basename(p), user_glob):
                out.append(p)
        elif rx.search(stem):
            out.append(p)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("logdir", help="directory of .log files to parse")
    ap.add_argument("-o", "--out", default=DEFAULT_OUT, help="output CSV path")
    ap.add_argument("--tag", default="", help="eval tag string (overrides EVAL tag)")
    ap.add_argument("--glob", default="", help="shell glob over basenames to select cells")
    ap.add_argument("--regex", default="", help="regex over filename stem to select cells")
    args = ap.parse_args()

    logs = select_logs(args.logdir, args.glob, args.regex)
    rows = []
    for p in logs:
        try:
            rows.append(build_row(p, args.tag))
        except Exception as e:  # noqa: BLE001 - one bad log must not abort
            rows.append({
                "cell": os.path.splitext(os.path.basename(p))[0],
                "mode": f"PARSE_ERROR:{type(e).__name__}",
            })

    # Headline arms get the extra arm + collision_partner columns. Triggered
    # when every selected cell is an hl_ arm (e.g. --regex '^hl_'); the mixed
    # default selection keeps the ablation column set unchanged.
    stems = [os.path.splitext(os.path.basename(p))[0] for p in logs]
    headline = bool(stems) and all(s.startswith("hl_") for s in stems)
    fieldnames = COLUMNS + HEADLINE_EXTRA if headline else COLUMNS

    outdir = os.path.dirname(os.path.abspath(args.out))
    if outdir:
        os.makedirs(outdir, exist_ok=True)
    with open(args.out, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=fieldnames, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow(r)
    print(f"wrote {len(rows)} rows -> {args.out}")
    for r in rows:
        print(
            f"  {r.get('cell','')}: mode={r.get('mode','')} "
            f"vel={r.get('vel_est_at_commit','')} "
            f"fde3={r.get('fde_3s','')} fde5={r.get('fde_5s','')} "
            f"launch={r.get('launch_tick','')} "
            f"gap={r.get('true_gap_at_launch','')} "
            f"min_ttc={r.get('min_ttc','')} coll={r.get('collided','')}"
        )


if __name__ == "__main__":
    main()
