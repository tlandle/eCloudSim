# -*- coding: utf-8 -*-
"""khonsu_accept.py - executable acceptance gate for a Khonsu smoke.

Parses each smoke-cell log and prints PASS/FAIL per invariant. The tag requires
ALL PASS on ALL cells (exit 0); any FAIL -> exit 1. Every invariant is read from
lines the runs already emit (RUNROW, HANDOFFROW, AGEROW, COASTROW, [EVAL],
[LAUNCHENV], SCENB COMMIT REFRESH, [OT RECHECK], TRACKER DBG).

Usage: python scripts/khonsu_accept.py <logdir_or_files...>
Cell arm/outcome are inferred from the RUNROW + filename.
"""
import sys, os, re, glob

TRUE_SPD = 12.0   # flow ONCOMING_SPEED; accel cruises 5 then floors 16

def _num(s, pat):
    m = re.search(pat, s)
    return float(m.group(1)) if m else None

def parse(path):
    d = dict(path=path, name=os.path.basename(path).replace('.log', ''))
    txt = open(path, errors='ignore').read()
    lines = txt.splitlines()
    d['runrow'] = next((l for l in lines if '[RUNROW]' in l), '')
    d['launchenv'] = next((l for l in lines if '[LAUNCHENV]' in l), '')
    d['has'] = {m: (m in txt) for m in
                ('[RUNROW]', '[HANDOFFROW]', '[AGEROW]', '[COASTROW]', '[EVAL]')}
    d['eval_horizons'] = set(re.findall(r'\+([0-9.]+)s:', txt))
    d['episodes'] = _num(d['runrow'], r'episodes=(\d+)')
    d['dbl_ticks'] = _num(d['runrow'], r'dbl_ticks=(\d+)')
    d['completed'] = (re.search(r'completed=([A-Za-z]+)', d['runrow']) or [None, None])[1]
    d['mode'] = (re.search(r'mode=([a-z_]+)', d['runrow']) or [None, None])[1]
    d['actor_id'] = ('actor_id' in txt)
    # migrated tracks: HANDOFFROW npc + first_use; COMMIT REFRESH tick per npc
    d['handoffs'] = {}
    for l in lines:
        m = re.search(r'\[HANDOFFROW\] npc=(\d+) prepare_tick=(\d+) crossing_tick=(\d+) '
                      r'first_dst_track_tick=(\d+) first_use_tick=(\d+)', l)
        if m:
            d['handoffs'][int(m.group(1))] = dict(
                prepare=int(m.group(2)), crossing=int(m.group(3)),
                first_dst=int(m.group(4)), first_use=int(m.group(5)))
    d['commit_refresh'] = {}
    for l in lines:
        m = re.search(r'COMMIT REFRESH tick=(\d+) npc=(\d+)', l)
        if m:
            d['commit_refresh'][int(m.group(2))] = int(m.group(1))
    # tid<->cid (association)
    d['cid_tids'] = {}
    for m in re.finditer(r'tid=(\d+) cid=(\d+)', txt):
        d['cid_tids'].setdefault(int(m.group(2)), set()).add(int(m.group(1)))
    # coast |v|
    d['coast_v'] = [float(x) for x in re.findall(r'\[COASTROW\][^\n]*\|v\|=([0-9.]+)', txt)]
    # planner recheck
    d['recheck'] = len(re.findall(r'\[OT RECHECK\]', txt))
    d['do_ov_true'] = len(re.findall(r'do_ov=True', txt))
    # HOLD/ABORT with subj_ahead>0 (a stall) : ABORT with subj_ahead>=0 is the
    # designed response, so we flag only a HOLD (brake) at subj_ahead>0.
    d['hold_subj_pos'] = 0
    for m in re.finditer(r'\[OT RECHECK\] HOLD .*subj_ahead=([0-9.-]+)m', txt):
        if float(m.group(1)) > 0:
            d['hold_subj_pos'] += 1
    return d

def check(cells):
    arms = {c['name']: c for c in cells}
    rows = []
    for c in cells:
        n = c['name']
        is_accel = 'accel' in n
        is_migr_arm = c['mode'] in ('warm', 'reactive', 'kf', 'edgewarp', 'handover_snapshot')
        r = {}
        # (1) runner signature
        sig = all(c['has'][m] for m in ('[RUNROW]', '[HANDOFFROW]', '[AGEROW]',
                                        '[COASTROW]', '[EVAL]')) if is_migr_arm \
            else all(c['has'][m] for m in ('[RUNROW]', '[EVAL]'))
        horiz = {'3.00', '5.00'}.issubset(c['eval_horizons'])
        r['1.sig'] = 'PASS' if (sig and horiz) else f'FAIL(sig={sig},h={sorted(c["eval_horizons"])})'
        # (2) geometry env
        le = c['launchenv']
        g = ('ONCOMING_SPEED=12' in le and 'TRIGGER_DIST=300' in le
             and (('ONCOMING_ACCEL=1' in le) if is_accel else True))
        r['2.geom'] = 'PASS' if g else f'FAIL({le[:60]})'
        # (3) ownership dbl_ticks
        r['3.own'] = 'PASS' if c['dbl_ticks'] == 0 else f'FAIL(dbl={c["dbl_ticks"]})'
        # (4) publish gate: first_use > commit per migrated track
        if c['handoffs'] and is_migr_arm:
            bad = [npc for npc, h in c['handoffs'].items()
                   if npc in c['commit_refresh'] and h['first_use'] <= c['commit_refresh'][npc]]
            r['4.pubgate'] = 'PASS' if not bad else f'FAIL(npc {bad})'
        else:
            r['4.pubgate'] = 'NA'
        # (5) final update (warm/edgewarp): COMMIT REFRESH per migrated track before first_use
        if c['mode'] in ('warm', 'edgewarp') and c['handoffs']:
            bad = [npc for npc in c['handoffs']
                   if npc not in c['commit_refresh']
                   or c['commit_refresh'][npc] > c['handoffs'][npc]['first_use']]
            r['5.final'] = 'PASS' if not bad else f'FAIL(npc {bad})'
        else:
            r['5.final'] = 'NA'
        # (6) association: one tid per migrated cid (migrated cids >= 199)
        dup = {cid: sorted(t) for cid, t in c['cid_tids'].items()
               if cid >= 199 and len(t) > 1}
        r['6.assoc'] = 'PASS' if not dup else f'FAIL({dup})'
        # (7) coast |v| within 0.5 of true (flow) / moving (accel variable speed)
        if c['coast_v']:
            if is_accel:
                ok = all(v >= 4.5 for v in c['coast_v'][:20])  # cruise 5 or fast 16
            else:
                ok = all(abs(v - TRUE_SPD) <= 0.5 for v in c['coast_v'][:20])
            r['7.coast'] = 'PASS' if ok else f'FAIL(v={c["coast_v"][:3]})'
        else:
            r['7.coast'] = 'NA' if not is_migr_arm else 'FAIL(no COASTROW)'
        # (8) planner gate: recheck logged while latched; no HOLD at subj_ahead>0
        if c['do_ov_true'] > 0:
            r['8.gate'] = 'PASS' if (c['recheck'] > 0 and c['hold_subj_pos'] == 0) \
                else f'FAIL(recheck={c["recheck"]},hold+={c["hold_subj_pos"]})'
        else:
            r['8.gate'] = 'NA'
        # (9) velocity source: migrated |v| used (coast |v| ~ true, not ~0) for migrated arm
        if is_migr_arm and c['coast_v']:
            r['9.velsrc'] = 'PASS' if max(c['coast_v'][:20]) >= 3.0 else 'FAIL(coast~0)'
        else:
            r['9.velsrc'] = 'NA'
        rows.append((n, r))
    # (10) outcome across arms
    def clean(c): return c['episodes'] == 0
    out = {}
    aw = [c for c in cells if re.search(r'accel_warm', c['name'])]
    if aw:
        out['accel_warm_3/3_clean'] = ('PASS' if all(clean(c) for c in aw)
                                       else f'FAIL({sum(clean(c) for c in aw)}/{len(aw)})')
    ac = [c for c in cells if re.search(r'accel_cold', c['name'])]
    if ac:
        out['accel_cold_collide'] = 'PASS' if all(not clean(c) for c in ac) else 'FAIL(clean cold)'
    fc = [c for c in cells if re.search(r'flow_cold', c['name'])]
    if fc:
        out['flow_cold_collide'] = 'PASS' if all(not clean(c) for c in fc) else 'FAIL(clean)'
    return rows, out

def main():
    args = sys.argv[1:]
    files = []
    for a in args:
        files += glob.glob(os.path.join(a, '*.log')) if os.path.isdir(a) else [a]
    files = [f for f in sorted(files) if os.path.basename(f).startswith(('m_', 'fix_'))]
    cells = [parse(f) for f in files]
    rows, out = check(cells)
    inv = ['1.sig', '2.geom', '3.own', '4.pubgate', '5.final', '6.assoc',
           '7.coast', '8.gate', '9.velsrc']
    print("CELL".ljust(26) + "eps ct  " + " ".join(i.split('.')[0].rjust(2) for i in inv))
    allpass = True
    for n, r in rows:
        c = next(x for x in cells if x['name'] == n)
        marks = []
        for i in inv:
            v = r[i]
            marks.append('P' if v == 'PASS' else ('-' if v == 'NA' else 'F'))
            if v.startswith('FAIL'):
                allpass = False
        print(n.ljust(26) + f"{str(c['episodes']):>3} {'y' if c['episodes'] else 'n':>2}  "
              + "  ".join(marks))
    print("\nFAIL detail:")
    for n, r in rows:
        for i in inv:
            if r[i].startswith('FAIL'):
                print(f"  {n} {i}: {r[i]}")
    print("\nOutcome (10):")
    for k, v in out.items():
        print(f"  {k}: {v}")
        if v.startswith('FAIL'):
            allpass = False
    print(f"\nGATE: {'ALL PASS' if allpass else 'FAIL'}")
    return 0 if allpass else 1

if __name__ == '__main__':
    sys.exit(main())
