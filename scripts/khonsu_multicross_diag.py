#!/usr/bin/env python3
"""Multi-crossing diagnostics for the RELAY/Khonsu freeze-1k Table 8 arms.

POST-HOC log extractor. Parses existing simulation .log files only. It does not
run the simulator, touch a GPU, or modify any sim code.

Emits PER-TRACK rows (one row per migrated npc per run) plus the run outcome on
each row. Cells are selected by filename-stem regex (default ^q5_ , ^burst_ ;
override with --glob for a shell glob or --regex for a stem regex).

Usage:
    python scripts/khonsu_multicross_diag.py <logdir> \
        [-o OUT.csv] [--tag TAG] [--glob 'q5_*.log'] [--regex '^q5_']

Output default: docs/kb/data/relay_eval_2026_08/frozen1k_multicross_diag.csv

Marker formats consumed:
    [HANDOFFROW] npc=199 prepare_tick=53 crossing_tick=55 first_dst_track_tick=54
                 first_use_tick=75 warm_before_first_use=YES dst=locale_0
    [TRACKER DBG] tid=1 cid=200 act=.. tsu=.. pos=(x,y,z) yaw=..
    [RUNROW] mode=.. episodes=N ...
    [PRED COLLISION] carla_id=C track_id=.. TTC=.. ...
    [EVAL]/[GT INJECT DBG] as in khonsu_ablation_diag.py (for FDE at first_use).

The concurrent-duplicate rule mirrors scripts/khonsu_accept.py invariant 6:
each tid's [TRACKER DBG] line indices for a cid are split into contiguous runs
(gap > 50 lines); two runs of DIFFERENT tids overlapping by > 20 lines are a
concurrent duplicate.
"""

import argparse
import csv
import fnmatch
import glob as globmod
import math
import os
import re
import statistics

SIM_DT = 0.05
HORIZONS = [0.25, 0.50, 1.25, 3.00, 5.00]
ACTUAL_TOL_TICKS = 4
RUN_GAP = 50   # invariant-6 contiguous-run gap (lines)
OVL = 20       # invariant-6 concurrent-overlap threshold (lines)

_ANSI = re.compile(r"\x1b\[[0-9;]*m")


def strip_ansi(s):
    return _ANSI.sub("", s)


def horizon_ticks(h):
    return int(round(h / SIM_DT))


def parse_log(path):
    d = {
        "launchenv": {},
        "runrow": {},
        "handoffs": {},        # npc -> dict of handoff fields
        "eval": {},            # cid -> list {tick, ax, ay, futs}
        "gt": {},              # aid -> {frame: (x,y)}
        "cid_tid_lines": {},   # (cid, tid) -> [line indices]
        "pred_collisions": [], # list of carla_id
        "stale_lines": [],     # (carla_id_or_None, raw)
        "eval_tag": "",
    }
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        raw_lines = fh.readlines()
    text = "".join(raw_lines)

    m = re.search(r"\[LAUNCHENV\]([^\n]*)", text)
    if m:
        for kv in re.findall(r"(\w+)=(\S+)", m.group(1)):
            d["launchenv"][kv[0]] = kv[1]

    for m in re.finditer(r"\[RUNROW\]([^\n]*)", text):
        row = {}
        for kv in re.findall(r"(\w+)=(\S+)", m.group(1)):
            row[kv[0]] = kv[1]
        d["runrow"] = row

    for m in re.finditer(
        r"\[HANDOFFROW\] npc=(\d+) prepare_tick=(\d+) crossing_tick=(\d+) "
        r"first_dst_track_tick=(\d+) first_use_tick=(\d+) "
        r"warm_before_first_use=(\w+) dst=(\S+)",
        text,
    ):
        d["handoffs"][int(m.group(1))] = {
            "prepare_tick": int(m.group(2)),
            "crossing_tick": int(m.group(3)),
            "first_dst_track_tick": int(m.group(4)),
            "first_use_tick": int(m.group(5)),
            "warm_before_first_use": m.group(6),
            "dst": m.group(7),
        }

    for m in re.finditer(
        r"\[GT INJECT DBG\] frame=(\d+) aid=(\d+) type=\S+ "
        r"world=\(([\-0-9.]+),([\-0-9.]+)\)",
        text,
    ):
        d["gt"].setdefault(int(m.group(2)), {})[int(m.group(1))] = (
            float(m.group(3)),
            float(m.group(4)),
        )

    for block in re.split(r"(?=\[EVAL\] tick=)", text):
        hm = re.match(
            r"\[EVAL\] tick=(\d+) arm=(\S+) tag=(\S+) Track (\d+) -> \S+ "
            r"\(CARLA ID (\d+)\)",
            block,
        )
        if not hm:
            continue
        if not d["eval_tag"]:
            d["eval_tag"] = hm.group(3)
        am = re.search(r"Actual:\s*x=\s*([\-0-9.]+),\s*y=\s*([\-0-9.]+)", block)
        if not am:
            continue
        futs = {}
        for fm in re.findall(r"\+([0-9.]+)s: \(([\-0-9.]+), ([\-0-9.]+)\)", block):
            futs[float(fm[0])] = (float(fm[1]), float(fm[2]))
        d["eval"].setdefault(int(hm.group(5)), []).append({
            "tick": int(hm.group(1)),
            "ax": float(am.group(1)),
            "ay": float(am.group(2)),
            "futs": futs,
        })

    # Line-indexed pass: TRACKER DBG tid/cid, PRED COLLISION, stale markers.
    for i, raw in enumerate(raw_lines):
        s = strip_ansi(raw)
        if "[TRACKER DBG]" in s:
            m = re.search(r"tid=(\d+) cid=(\d+)", s)
            if m:
                key = (int(m.group(2)), int(m.group(1)))
                d["cid_tid_lines"].setdefault(key, []).append(i)
            continue
        if "[PRED COLLISION]" in s:
            m = re.search(r"carla_id=(\d+)", s)
            if m:
                d["pred_collisions"].append(int(m.group(1)))
            continue
        if re.search(r"stale", s, re.IGNORECASE):
            cm = re.search(r"(?:carla_id|cid|aid|npc)=(\d+)", s)
            d["stale_lines"].append((int(cm.group(1)) if cm else None, s.strip()))
    return d


def _runs(lns, gap=RUN_GAP):
    lns = sorted(lns)
    out = []
    s = p = lns[0]
    for x in lns[1:]:
        if x - p > gap:
            out.append((s, p))
            s = x
        p = x
    out.append((s, p))
    return out


def dup_and_concurrent(d, cid):
    """Return (num_distinct_tids, concurrent_dup_flag) for a cid using the
    khonsu_accept invariant-6 contiguous-run overlap rule."""
    tids = set()
    runs = []
    for (c2, tid), lns in d["cid_tid_lines"].items():
        if c2 != cid or not lns:
            continue
        tids.add(tid)
        for (s, e) in _runs(lns):
            runs.append((s, e, tid))
    concurrent = 0
    for i in range(len(runs)):
        for j in range(i + 1, len(runs)):
            s1, e1, t1 = runs[i]
            s2, e2, t2 = runs[j]
            if t1 != t2 and min(e1, e2) - max(s1, s2) > OVL:
                concurrent = 1
    return len(tids), concurrent


def actual_timeline(d, cid):
    ev = {}
    for r in sorted(d["eval"].get(cid, []), key=lambda r: r["tick"]):
        ev[r["tick"]] = (r["ax"], r["ay"])
    return ev, d["gt"].get(cid, {})


def lookup_actual(ev, gt, target_tick):
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


def fde_at(d, cid, at_tick, H):
    """Endpoint (FDE) error at horizon H for the forecast issued nearest
    at_tick for this cid. Blank if unavailable."""
    recs = sorted(d["eval"].get(cid, []), key=lambda r: r["tick"])
    if not recs:
        return ""
    rec = min(recs, key=lambda r: abs(r["tick"] - at_tick))
    if abs(rec["tick"] - at_tick) > ACTUAL_TOL_TICKS + 2:
        return ""
    fpos = rec["futs"].get(H)
    if fpos is None:
        return ""
    ev, gt = actual_timeline(d, cid)
    apos = lookup_actual(ev, gt, rec["tick"] + horizon_ticks(H))
    if apos is None:
        return ""
    return round(math.hypot(fpos[0] - apos[0], fpos[1] - apos[1]), 3)


COLUMNS = [
    "cell", "mode", "seed", "tag", "npc", "episodes", "collided", "dbl_ticks", "contact_actor",
    "prepare_tick", "crossing_tick", "first_dst_track_tick", "first_use_tick",
    "ready_delay", "use_delay", "tracks_same_cycle",
    "dup_tids_for_cid", "concurrent_dup", "fde_5s_at_first_use", "stale_consumed",
]

DEFAULT_REGEX = r"^(q5_|burst_)"
DEFAULT_OUT = "docs/kb/data/relay_eval_2026_08/frozen1k_multicross_diag.csv"


def build_rows(path, tag_override):
    cell = os.path.splitext(os.path.basename(path))[0]
    d = parse_log(path)
    le = d["launchenv"]
    rr = d["runrow"]
    mode = rr.get("mode") or le.get("MIGRATION_MODE") or ""
    sm = re.search(r"_s(\d+)", cell)
    seed = sm.group(1) if sm else le.get("SEED", "")
    tag = tag_override or d.get("eval_tag") or ""

    episodes = rr.get("episodes", "")
    dbl_ticks = rr.get("dbl_ticks", "")  # inv3: concurrent double publication
    try:
        collided = "True" if int(episodes) >= 1 else "False"
    except (TypeError, ValueError):
        collided = ""

    contact_actor = ""
    if d["pred_collisions"]:
        contact_actor = str(d["pred_collisions"][-1])

    run_stale_total = len(d["stale_lines"])

    # tracks sharing each prepare_tick
    prep_counts = {}
    for h in d["handoffs"].values():
        prep_counts[h["prepare_tick"]] = prep_counts.get(h["prepare_tick"], 0) + 1

    rows = []
    for npc, h in sorted(d["handoffs"].items()):
        dup_tids, concurrent = dup_and_concurrent(d, npc)
        npc_stale = sum(1 for cid, _ in d["stale_lines"] if cid == npc)
        # not_scoreable when the logs carry no stale/epoch markers at all; the
        # epoch pathology is then read from inv3 dbl_ticks (concurrent double
        # publication) instead of a zero that looks like a clean measurement.
        if run_stale_total == 0:
            stale_consumed = "not_scoreable"
        else:
            stale_consumed = npc_stale if npc_stale else run_stale_total
        rows.append({
            "cell": cell,
            "mode": mode,
            "seed": seed,
            "tag": tag,
            "npc": npc,
            "episodes": episodes,
            "collided": collided,
            "dbl_ticks": dbl_ticks,
            "contact_actor": contact_actor,
            "prepare_tick": h["prepare_tick"],
            "crossing_tick": h["crossing_tick"],
            "first_dst_track_tick": h["first_dst_track_tick"],
            "first_use_tick": h["first_use_tick"],
            "ready_delay": h["first_dst_track_tick"] - h["prepare_tick"],
            "use_delay": h["first_use_tick"] - h["crossing_tick"],
            "tracks_same_cycle": prep_counts.get(h["prepare_tick"], 1),
            "dup_tids_for_cid": dup_tids,
            "concurrent_dup": concurrent,
            "fde_5s_at_first_use": fde_at(d, npc, h["first_use_tick"], 5.00),
            "stale_consumed": stale_consumed,
        })
    return rows


def rank_corr(xs, ys):
    """Spearman rank correlation; falls back to Pearson-on-ranks by hand.
    Returns None if degenerate."""
    n = len(xs)
    if n < 2:
        return None
    def ranks(v):
        order = sorted(range(n), key=lambda i: v[i])
        r = [0.0] * n
        i = 0
        while i < n:
            j = i
            while j + 1 < n and v[order[j + 1]] == v[order[i]]:
                j += 1
            avg = (i + j) / 2.0 + 1.0
            for k in range(i, j + 1):
                r[order[k]] = avg
            i = j + 1
        return r
    rx, ry = ranks(xs), ranks(ys)
    mx = statistics.fmean(rx)
    my = statistics.fmean(ry)
    num = sum((rx[i] - mx) * (ry[i] - my) for i in range(n))
    dx = math.sqrt(sum((rx[i] - mx) ** 2 for i in range(n)))
    dy = math.sqrt(sum((ry[i] - my) ** 2 for i in range(n)))
    if dx == 0 or dy == 0:
        return None
    return num / (dx * dy)


def summarize(all_rows):
    """Print run-level / block-level summary lines."""
    by_cell = {}
    for r in all_rows:
        by_cell.setdefault(r["cell"], []).append(r)
    def _dt(r):
        try:
            return int(r.get("dbl_ticks", "") or 0)
        except (TypeError, ValueError):
            return 0
    for cell, rows in sorted(by_cell.items()):
        any_dup = any(r["concurrent_dup"] for r in rows)
        max_dbl = max((_dt(r) for r in rows), default=0)
        # Pathology = concurrent duplicate tids (assoc) OR inv3 double publication
        # (dbl_ticks>0). When stale/epoch markers are absent, dbl_ticks is the
        # authoritative concurrent-double-publication signal.
        print(f"[SUMMARY] cell={cell} tracks={len(rows)} "
              f"any_concurrent_dup={'YES' if any_dup else 'no'} "
              f"max_dbl_ticks={max_dbl} "
              f"pathology={'YES' if (any_dup or max_dbl > 0) else 'no'}")

    # block-level delay vs collided correlation
    coll = []
    ready = []
    use = []
    for r in all_rows:
        if r["collided"] not in ("True", "False"):
            continue
        c = 1.0 if r["collided"] == "True" else 0.0
        coll.append(c)
        ready.append(float(r["ready_delay"]))
        use.append(float(r["use_delay"]))
    if not coll:
        print("[SUMMARY] block: no rows with a defined collided outcome")
        return
    n_coll = sum(1 for c in coll if c == 1.0)
    if 0 < n_coll < len(coll):
        # Outcome varies -> correlation is defined.
        try:
            from scipy.stats import spearmanr  # type: ignore
            rc_ready = spearmanr(ready, coll).correlation
            rc_use = spearmanr(use, coll).correlation
            method = "spearman(scipy)"
        except Exception:  # noqa: BLE001
            rc_ready = rank_corr(ready, coll)
            rc_use = rank_corr(use, coll)
            method = "spearman(stdlib)"
        mean_ready_c = statistics.fmean([ready[i] for i in range(len(coll)) if coll[i] == 1.0])
        mean_ready_n = statistics.fmean([ready[i] for i in range(len(coll)) if coll[i] == 0.0])
        mean_use_c = statistics.fmean([use[i] for i in range(len(coll)) if coll[i] == 1.0])
        mean_use_n = statistics.fmean([use[i] for i in range(len(coll)) if coll[i] == 0.0])
        print(f"[SUMMARY] block delay vs collided ({method}): "
              f"corr(ready_delay)={rc_ready} corr(use_delay)={rc_use}")
        print(f"[SUMMARY] mean ready_delay collided={mean_ready_c:.2f} clean={mean_ready_n:.2f}; "
              f"mean use_delay collided={mean_use_c:.2f} clean={mean_use_n:.2f}")
    else:
        print(f"[SUMMARY] block: collided outcome is constant across "
              f"{len(coll)} tracks (n_collided={n_coll}); correlation undefined. "
              f"mean ready_delay={statistics.fmean(ready):.2f} "
              f"mean use_delay={statistics.fmean(use):.2f}")


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
    all_rows = []
    for p in logs:
        try:
            all_rows.extend(build_rows(p, args.tag))
        except Exception as e:  # noqa: BLE001 - one bad log must not abort
            all_rows.append({
                "cell": os.path.splitext(os.path.basename(p))[0],
                "mode": f"PARSE_ERROR:{type(e).__name__}",
            })

    outdir = os.path.dirname(os.path.abspath(args.out))
    if outdir:
        os.makedirs(outdir, exist_ok=True)
    with open(args.out, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=COLUMNS, extrasaction="ignore")
        w.writeheader()
        for r in all_rows:
            w.writerow(r)
    print(f"wrote {len(all_rows)} rows -> {args.out} (from {len(logs)} logs)")
    summarize([r for r in all_rows if "npc" in r])


if __name__ == "__main__":
    main()
