# Evidence scaffold and realism rules (reviewer, 2026-09-07)

Spec of record for every evaluation figure: order, paper location, RQ, axes, series, the claim each figure earns, the shape real data should have, and the variance to show. Generators write CSVs in the measured schema; plot scripts consume CSVs; `scripts/figure_rules.py` scores any CSV against the rules below. Measured data replaces generated data figure by figure when it passes its rules; when it does not, the scenario and tuned parameters are fixed until it does.

| Order | Paper | RQ | Figure | Claim |
|---|---|---|---|---|
| 1 | §2.1 | RQ1 | canvas size to fusion latency | one locale has finite extent |
| 2 | §2.3 / §5.9 | RQ1 | conflict distance to nearest boundary (ECDF) | placement cannot eliminate dynamic conflicts |
| 3 | §5.2 | RQ2 | Kalman vs SSM, one locale | temporal history earns its keep under motion |
| 4 | §5.2 | RQ2 | history depth / forced handoff | what state has to move |
| 5 | §5.3 | RQ2/RQ3 | operating regions vs T_avail | when snapshot, reactive, preparation suffice |
| 6 | §5.4 | RQ3 | controlled age to success + real age CDF | freshness the planner requires |
| 7 | §5.5 | RQ3 | trigger Pareto (warm vs wasted bytes per true crossing) | why the forecast trigger |
| 8 | §5.6 | RQ3 | overlap width to warm-before-use at 12 and 20 m/s | overlap valid but increasingly expensive |
| 9 | §5.6 | RQ3 | continuity vs cost across alternatives | where Khonsu sits |
| 10 | §5.7 | RQ4 | corridor age timeline, one paired representative seed | continuity across repeated handoffs |
| 11 | §5.7 | RQ4 | crossing index 1,2,3 and route-level dot plots | repeated handoffs do not accumulate failure |
| 12 | §5.8 | RQ4 | concurrent crossings to p50/p95/p99 first forecast | capacity |
| 13 | §5.8 | RQ3/RQ4 | transport sensitivity (delay, loss) | survives realistic backhaul |
| 14 | §5.8 / appendix | RQ3 | fault/epoch table | ownership stays correct |
| 15 | §5.8 / appendix | RQ4 | association success vs pose error | non-connected tracks move |
| 16 | §5.9 | RQ1/RQ4 | locale size feasible band (+ heatmap) | close the metropolitan story |
| 17 | §5 summary | all | operating regions + continuity vs cost side by side | when continuity is needed, cheapest way |

## Realism rules (apply to every figure)

- Binary outcomes: markers per measured point with Wilson 95% intervals; thin guide lines only; logistic fits labeled as fits.
- Time series: median with IQR band, or one paired representative trace chosen by a predeclared rule (median outcome seed), plus aggregate statistics separately.
- Tail latency: report the number of handoff samples, not runs; p99 needs hundreds of samples.
- Same seeds across methods so comparisons are paired.
- Never remove non-monotone points; never draw curves smoother than the measurements.
- Let the baselines succeed where they succeed: the result is where each works, then where Khonsu gives continuity at lower steady-state cost.

## Per-figure shape rules

1. Canvas: monotone, convex; tight spread at fixed size; 4 and 8 contributors separate with size; p50 line with light p95 band; leave small non-monotonicities.
2. Conflict distance ECDF: fixed conflicts concentrated far from boundaries; traffic-generated broad with meaningful mass within 25 to 50 m; structure by road topology, variation across maps not seeds.
3. Kalman vs SSM: constant-speed panel nearly overlapping with jitter, either may be slightly better; maneuver panel: Kalman rises after a short lag, peaks during or just after the strongest acceleration; SSM also rises but smaller and recovers faster; align t=0 at maneuver onset; median + IQR over tracks and seeds.
4. History depth: diminishing returns with a knee (0,1,2,5,10 frames), not strictly monotone; brake and turn above straight at shallow depth; paired tracks with bootstrap CI.
5. Operating regions: two panels (constant, maneuvering); markers at measured T_avail with Wilson CIs; broad transitions of several hundred ms; Kalman with velocity may show no cliff at constant speed; maneuvering moves the snapshot transition to the right (more time needed); Khonsu high but not perfect; oracle high but not perfect; regions derived from CIs.
6. Freshness: panel A controlled age sweep with a soft cliff and maneuver-specific transition, 20 to 30 trials near the cliff, tau from a fitted success threshold, not the largest all-pass bin; panel B age-at-use CDFs at 4, 12, 20, 31 contenders with long right tails and cadence quantization steps.
7. Trigger Pareto: a cloud, not a diagonal; boundary and radio low waste and low warm; aggressive fixed leads high warm and more waste; bands high warm and moderate waste; predictor near the high-warm low-waste corner; oracle upper-left; Wilson on warm rate, bootstrap on bytes per true crossing; lead CDF in the appendix.
8. Overlap warmth: low at small width, broad transition, saturation below 100%, faster actor shifted right; markers + Wilson intervals.
9. Overlap cost: primary = duplicated tracking and prediction compute per crossing measured from dual-service execution, roughly linear in w/v with stair steps from prediction cycles, variance growing with width; companion = fusion latency at coverage L + w from the canvas sweep, convex, labeled "fusion cost of wider sensing coverage" and never "total overlap cost".
10. Dual publication: two aligned mini-panels (fraction of disagreeing ticks, p95 disagreement in metres); fraction rises with width then plateaus; magnitude may be non-monotone; do not force drama.
11. Alternatives scatter: Khonsu need not be highest on continuity; replication may match or beat it; overlap is a mini curve over widths; vertical Wilson/bootstrap bars, horizontal route-to-route cost variation; two panels for network and compute cost or compute on x with KB labels.
12. Corridor timeline: sawtooth from cycles, occasional spikes, crossing transients, some points near the limit; baseline spikes at some crossings, not all; one representative paired seed by predeclared rule; aggregate separately.
13. Crossing index: Khonsu approximately stable across 1 to 3; investigate any downward trend, do not average it away; at least 20 paired seeds per design for the headline route result.
14. Route summary: compact dot plots (route success, freshness compliance); some failures remain for oracle, replication, and Khonsu; if oracle is 100% and every baseline 0%, inspect the scenario.
15. Capacity: p50 flat long, p95 widens, p99 jumps at the knee; not parallel lines; horizontal budget line; vertical marker at the observed p95 burst; report handoff samples.
16. Transport: quantized or stair-stepped latency against injected delay because of discrete edge cycles; loss sweep with little effect while retries fit inside the lead, then deterioration.
17. Faults: a table with columns fault, double-emission window, stale forecasts consumed, fallback, recovered; epochs disabled and enabled; deterministic on the isolated state machine; median/p95 emission duration on the live path.
18. Association: success near 100% at zero pose error but not literally; stays high then degrades around the effective gate; distractors move the drop earlier; broad degradation, not a cliff at 8 m; stable-ID as upper bound, geometry-only as fallback.
19. Sizing: fusion utilization increasing and convex; handoff pressure decreasing but jagged with topology; feasible band, not an exact 300 m; heatmap over size and boundary offset or density with an irregular feasible island.
20. Summary figure: operating regions on the left, continuity vs cost on the right.

## Deck order (main narrative, 22 slides; the rest is backup)

1 title; 2 Conductor solved one locale; 3 main question; 4 RQs and hypotheses; 5 canvas result; 6 architecture and conflicts at boundaries; 7 one locale Kalman vs SSM at constant speed; 8 one locale SSM advantage under acceleration and braking; 9 forced-handoff history depth; 10 central T_avail operating regions; 11 freshness cliff per maneuver; 12 Khonsu mechanism; 13 trigger Pareto; 14 overlap width to warmth; 15 overlap and replication cost; 16 alternatives continuity vs cost; 17 corridor topology; 18 corridor age timeline; 19 corridor aggregate; 20 crossing capacity; 21 locale-size feasible band; 22 done, running, missing. Backup: association, transport, faults, individual trigger bars, all-seed tables, defects.
