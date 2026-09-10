#!/usr/bin/env python3
# Split the corridor stale-owner-consumed reads into destination-bound violations
# (a real "two owners' streams" breach) vs source-bound legitimate reads (a CAV
# still bound to the source locale reading the source's own local track, which
# the design permits until the source tracker drops the road user).
import re, sys
SWAP = '--swap' in sys.argv
text = open(sys.argv[1], errors="ignore").read()
# locale x-ranges from the corridor config (overlap: 200-210 = 0&1, 70-80 = 1&2)
LOC = {"locale_0": (200, 330), "locale_1": (70, 210), "locale_2": (-90, 80)}
def in_loc(x, loc):
    lo, hi = LOC[loc]; return lo <= x <= hi
cross = {}
for m in re.finditer(r"CORRIDORCROSS\] npc=(\d+).*?src=(\S+) dst=(\S+).*?crossing_tick=(\d+).*?epoch=(-?\d+)", text):
    cross.setdefault(int(m.group(1)), []).append(
        (int(m.group(4)), int(m.group(5)), m.group(2), m.group(3)))
ego = {}
for m in re.finditer(r"EGO-DBG\] tick=(\d+) pos=\(([0-9.-]+),", text):
    ego[int(m.group(1))] = float(m.group(2))
egoticks = sorted(ego)
import bisect
def ego_x_at(t):
    i = bisect.bisect_right(egoticks, t) - 1
    return ego[egoticks[i]] if i >= 0 else None
viol = legit = overlap = other = total = 0
for m in re.finditer(r"CONSUMEDEPOCH\] ego=(\d+) actor=(\d+) tick=(\d+) consumed_epoch=(-?\d+)", text):
    actor = int(m.group(2)); t = int(m.group(3)); ce = int(m.group(4))
    if actor not in cross:
        continue
    prior = [(ct, ep, s, d) for (ct, ep, s, d) in cross[actor] if 0 <= ct <= t]
    if not prior:
        continue
    cur = max(ep for (ct, ep, s, d) in prior)
    if ce < 0 or ce >= cur:
        continue
    total += 1
    cc = [(ct, ep, s, d) for (ct, ep, s, d) in prior if ep == cur][-1]
    src, dst = cc[2], cc[3]
    if SWAP: src, dst = dst, src
    ex = ego_x_at(t)
    if ex is None:
        other += 1; continue
    ind, ins = in_loc(ex, dst), in_loc(ex, src)
    if ind and ins:
        overlap += 1
    elif ind:
        viol += 1
    elif ins:
        legit += 1
    else:
        other += 1
print(f"total_stale={total} dest_bound_VIOLATION={viol} source_bound_legit={legit} "
      f"overlap_band_ambiguous={overlap} ego_elsewhere={other}")
