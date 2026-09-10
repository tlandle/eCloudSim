#!/usr/bin/env python3
# Identify what the ego actually contacted in a corridor route: find where the
# ego stalls (sustained contact), then the nearest actor and whether it is
# stationary (a blocker) or a moving oncoming (cid 200/201/202).
import re, sys
text = open(sys.argv[1], errors="ignore").read()
ego = []
for m in re.finditer(r"EGO-DBG\] tick=(\d+) pos=\(([0-9.-]+),([0-9.-]+)\)", text):
    ego.append((int(m.group(1)), float(m.group(2)), float(m.group(3))))
ego.sort()
if not ego:
    print("no ego LOC"); sys.exit()
last = ego[-1]; sx, sy = last[1], last[2]
stall_start = last[0]
for (t, x, y) in reversed(ego):
    if abs(x - sx) < 1.5 and abs(y - sy) < 1.5:
        stall_start = t
    else:
        break
print(f"ego final tick={last[0]} pos=({sx:.1f},{sy:.1f}) "
      f"stall_from_tick={stall_start} stall_ticks={last[0]-stall_start}")
# actors: any line with cid=N ... pos=(x,y); collect all positions per cid
acts = {}
for m in re.finditer(r"cid=(\d+)[^\n]{0,80}?pos=\(([0-9.-]+),([0-9.-]+)\)", text):
    cid = int(m.group(1))
    acts.setdefault(cid, []).append((float(m.group(2)), float(m.group(3))))
ONC = {200, 201, 202}
print("actors within 8m of the ego stall (cid, count near, x/y range over run, class):")
for cid, pts in sorted(acts.items()):
    near = [(x, y) for (x, y) in pts if abs(x - sx) < 8 and abs(y - sy) < 8]
    if not near:
        continue
    xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
    xr = max(xs) - min(xs); yr = max(ys) - min(ys)
    moving = "MOVING" if (xr > 3 or yr > 3) else "STATIONARY"
    kind = "oncoming" if cid in ONC else "blocker/other"
    print(f"  cid={cid} near={len(near)} xrange={xr:.1f} yrange={yr:.1f} "
          f"{moving} {kind} sample_near={near[0]}")
