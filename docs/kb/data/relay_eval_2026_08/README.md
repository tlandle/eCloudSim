# relay_eval_2026_08: Khonsu closed-loop data

Provenance (2026-09-06): the paper's evaluation DATA lives here, in docs/kb/data on
the develop branch. The CODE is frozen in the git tags (khonsu-eval-freeze-1X);
from freeze-1i on, the tags are CODE-ONLY and carry no evaluation data. Each row
names the code it was produced under in its `eval_tag` column; join a row to its
code by checking out that tag. Do not read CSVs out of a tag checkout (they are
absent from 1i on, and stale on older tags that happened to carry committed data).

One CSV row per run. Binary outcome fields: `collided` (any ego contact), `completed`. Report collided-or-not and completed-without-collision only; never mean episodes.

| File | What | Harness (develop) | Status |
|---|---|---|---|
| design_sweep_v3_rows.csv | six flow arms x 10 + burst arms x 5 on the corrected scenario (ONCOMING_SPEED=12, TRIGGER_DIST=300 as runner defaults) | scripts/khonsu_rebaseline_arms.sh | citable re-baseline |
| q5_scale_rows.csv | oncoming density N=2/4/8 (FLOW_N releases the first N of 8 parked oncoming vehicles at scenario start; all N cross the boundary; unconnected actors, one migration per crossing), arms cold / edgewarp (at-crossing one-frame snapshot, paper name "handover snapshot") / warm (forecast trigger 1 s, full history), 5 runs per cell | scripts/khonsu_q5_scale.sh; FLOW_N knob in ecav/scenario_testing/scenarios/scenario_1.py | citable (collided field); completion-field format under confirmation |
| design_sweep_v1_rows.csv | 150-run uniform design sweep on the unvalidated geometry | scripts/khonsu_design_sweep.sh | relative findings only, not citable absolutely |
| fault_injection_results.csv | eight ownership-epoch scenarios through the per-track state machine | unit harness | citable |
| canvas_latency.csv | fusion latency and memory vs BEV canvas side, 8 contributors | see current_state.md 2026-08-31 | citable (Fig. 1) |
| q3_lookahead_sweep.csv, q7_locale_sizing.csv | August pilots | superseded by s5 lookahead redo (scripts/khonsu_s5_lookahead_pinned.sh, worktree khonsu_v1_wt at 9d5e1883) and the sizing study | do not cite |

Retired: q4_flow_table.csv (canonical warm arm reproduces 4/4 under the env contract, but the reactive and edgewarp arms never transferred). Provenance history in docs/kb/wiki/current_state.md (2026-08-31, 2026-09-03, 2026-09-04) and raw/sessions.

## Radio plane (platform note, 2026-09-05)
The default flow/accel runners use the ANALYTICAL C-V2X plane: SEE-V2X
measured latency traces (HybridModel: real C-V2X RTT + backhaul lognormal +
base_ms) plus SB-SPS PC5 Mode-4 contention (SbSpsMac, M=20 resources). This
plane governs the UPLINK only (sensor -> edge, via the jitter buffer). The
edge -> ego forecast DOWNLINK is instantaneous, gated only by
downlink_packet_loss_pct (no delay stamp). Age at use is therefore carried by
uplink staleness. Measured SEE-V2X RTT p50/p95: L 11.8/22.3, M 12.3/23.3,
H 18.7/23.8 ms. The 5G-LENA ns-3 co-sim (ecav/core/networking/ns3_cosim) is a
separate, unwired path. The paper's platform sentence must match whichever
T12 age source Tyler selects.

## Visible-row tag justification (2026-09-05)
5.3 visible cells (flow_visible, accel_visible) are labeled eval_tag=
freeze-1c+vis, NOT re-run under a single tag with the occluded cells. Justified:
the visible scenario files (scenario_1_flow_visible.xml, scenario_1_accel_
visible.xml + their configs/runners) are ADDITIVE - new files added after
freeze-1c. The occluded-cell arms never reference them (they load
scenario_1_flow.xml / scenario_1_accel.xml), so occluded-arm behavior is
byte-identical with or without the visible files present. The 2x2 figure
therefore pools freeze-1c (occluded) + freeze-1c+vis (visible) rows validly.

## CORRECTION to the radio-plane note (2026-09-05)
The earlier "downlink instantaneous, loss only" note was WRONG for the flow
runner. It described the BASE pluggable manager (_advance_vehicles). The flow
runner's manager (WorldFusionAdaptiveEdge -> WorldFusionEdge, the
linear_predictor) has use_ns3_lut DEFAULT TRUE and applies BOTH uplink and
downlink ns-3 LUT latency: UL sampled per-CAV (max over N) into the jitter
buffer, DL sampled (n_cav, dl_bytes) plus compute_ms into an outbound queue
delivered at deliver_tick. So realized age at use is already ns-3-LUT-derived
(ns3_uplink_lut.csv / ns3_downlink_lut.csv, N in {4,8,16,24,31} x payload,
bilinear interp). ns-3 IS part of this platform. LUT ranges: UL p50 9.6-37.8,
DL p50 5.3-304, DL p95 up to 952 ms at N=31 / 16.9KB. T12 (freeze-1e) sweeps
NS3_LUT_N over the LUT range; MAC_BG_SENDERS dropped. The paper's platform
sentence: ns-3-derived C-V2X Uu latency (payload/N-aware LUT), UL+DL.

## t12_lut_rows.csv collided-flag correction (2026-09-06)
The freeze-1f blind-overtake half (35 runs, 7 N-levels x 5 seeds) was landed
with an inline t12 extractor whose collided detection used a literal-adjacent
`- WARNING - Collision` regex. The live collision-sensor line is not
adjacent (WARNING and Collision are separated by other tokens), so the regex
matched nothing and every row got collided=0. Two runs were internally
inconsistent (collided=0 but completed=no), which flagged the bug. Authoritative
RUNROW re-check: N=31 s1 (episodes=1, contact_ticks=30, 1408 raw collision
warnings) and N=31 s4 (episodes=1, contact_ticks=30, 1396) collided; all other
33 runs clean (episodes=0). Corrected in place: N=31 s1/s4 collided 0->1. N=31
is 3/5 completed-without-collision; every other N-level is 5/5. The runs are
valid and complete (RUNROW present; the core dumps are post-run teardown
segfaults, not run failures), so this is a column re-derivation, not a re-run.
Root cause isolated to this one inline path; the general extractor
(scripts/khonsu_design_extract.py:53) derives collided from RUNROW episodes and
is unaffected.

## tau(blind overtake) statistic (2026-09-06, final)
NS3_LUT_N overrides only the lookup-table index; the scene (FLOW_N, spawns,
speeds) is identical at every level, so there is no density difference between
levels. What differs is the sampled latency distribution.

The safe-age limit is defined on age_at_maneuver_ms, NOT on whole-run p50/p95:
  age_at_maneuver_ms(W) = max realized age of the forecasts the planner consumed
  over the W-second window of AGEROW ending at the maneuver tick (first-contact
  tick if the run contacts, else the conflict tick; the conflict tick is 236 in
  every run since the scene is identical). tau(u) = the largest 100ms bin of
  age_at_maneuver below which no run fails.
Whole-run statistics do NOT work here: the clean N=31 runs take an age spike
early in the approach (tick ~447-538, actor still far / ego not committed) that
they recover from, so whole-run/full-pre-contact max is 1400-2000ms for clean
runs, ABOVE the failures at 1200-1250; that reading is non-monotone and yields
no threshold. Bounding to the maneuver window removes that spike.

W=2s is the planner's collision look-ahead horizon, not an arbitrary constant:
config collision_time_ahead=2 feeds CollisionChecker(time_ahead=2), which
projects the consumed obstacle forecast 2s ahead (lookahead_interp) and runs the
collision-circle check over that window (ecav/core/plan/collision_check.py:245,
643-644). The overtake commit gate consumes the forecast to decide over a 2s
horizon, so the relevant freshness is the age over the 2s window at the commit.

Result (realized_age_maneuver_ms in t12_lut_rows.csv, W=2s): every run with
age_at_maneuver <= 1000ms completes (all of N<=24, and N=31 s2/s3 at 1000); at
1200ms N=31 s5 completes while s1/s4 fail. tau(blind overtake) = 1.0s, onset
1.2s. Sensitivity (extraction only, no runs): tau=1.0s and onset=1.2s are STABLE
across W=1s, 2s, 3s. The only W-dependence is the onset bin: at W=1s no clean run
reaches 1200 (max clean 1000, pure-fail onset), at W>=2s the clean s5 also
reaches 1200 (mixed onset). The 800-1150ms band is unsampled (N=24 maneuver
<=1000 jumps to N=31 >=1000), so NS3_LUT_N=28 x5 is queued to sample it.

Checker rule: keep collided from the general extractor's eps>0; read tau off
realized_age_maneuver_ms with the largest-all-clean-bin rule. Accel scenario
uses the same planner (collision_time_ahead=2), so W=2s carries over; its
conflict tick and window land with the accel T12 half.

## AGEROW availability + adaptive-lineage LUT (2026-09-05, restart-4 rescinded)
CONFIRMED (peer + code): edge_manager_worldfusion_ab3dmot_mtr_adaptive.run_step
calls super().run_step at line 171; neither the adaptive nor the mamba
subclass overrides delivery/latency, so WorldFusionEdge's ns-3 LUT sampling
(UL at ingest, DL into the latest-wins outbound queue) is LIVE in every
Khonsu run including frozen1c. The adaptive lineage was created after the
Apr-30 LUT commit but inherits it; the earlier "grep per file missed
inheritance" scare (restart 4) was wrong and RESCINDED - no code change, batch
not stopped (beyond an accidental kill I immediately resumed).
frozen1c rows: NO AGEROW (realized age at use not extractable for the headline
batch); the extractable freshness fields are HANDOFFROW warm_before_first_use
and bytes. AGEROW (realized age = delivery_tick - source_frame_tick, ms) is
added at freeze-1e (outbound drain) and carried by T12 and all later tags.

## Medium mismatch: inter-locale transfer accounted as radio (2026-09-05)
frozen1c/1e: InterLocaleLink(edge.latency_model) sampled the transfer's
network cost from the C-V2X RADIO model, but inter-locale transfer is WIRED
backhaul between edge servers. SCOPE: accounting only (link.py never delays
the mechanism) + the computed-trigger EMA seed (40ms was a radio number).
The LEADROW/XFERROW transfer times and computed-lead EMA are affected; the
lead EFFECT is <0.05s, and warm/predictive/mtr/oracle arms do not use the
EMA, so frozen1c is NOT invalidated (Tyler's ruling). FIX (develop, for
freeze-2/T8/later): InterLocaleLink wired model = base 2ms + payload/1Gbps +
0.5x queueing; computed EMA reseeded to ~2ms. TRANSFER_MEDIUM=wired default.
NETEM matrix HELD until citable 5G-MOBIX wired-backhaul figures arrive.

## Wired backhaul figures: 5G-MOBIX D5.2 (2026-09-05)
Source: 5G-MOBIX D5.2 v3.0 "Report on technical evaluation" section 4.5.4,
CS_14 inter-MEC exchange. Table 30 (MEC-to-MEC over fibre, 100km ES-PT
corridor): one-way network latency avg 1.8-3.6 ms, median 1.6-3.6, p95
2.4-3.7, max 3.6-5.2, stdev 0.1-0.8 ms; 11km NL federation added no
measurable delay; executive-summary bullet: direct inter-MEC over leased line
across operators adds 15-20 ms per hop; wired packet loss zero (TCP).
InterLocaleLink wired model uses base 3 ms + 0.5 ms jitter (stdev) +
serialization + payload/1Gbps + 0.5x queue; computed-trigger EMA reseeded
~3 ms. netem matrix (Table 31 equivalent): (delay,jitter) = (0,0),(3,0.5),
(20,1),(50,2) ms x loss {0,0.1,1}%. 3ms=fibre op-point, 20ms=leased-line
hop (both MEASURED); 50ms and 1% loss = stress beyond measurement (labeled
_stress in row tags); 0.1% = nominal wired loss.

## Age-at-use floor is a result (2026-09-05)
Delta_use (t_c - t_o, total age of the consumed forecast's newest observation
at use) has a MINIMUM of ~300 ms at the lightest load, quantized in ~150 ms
steps, set by the edge cadence (edge_dt=0.2s) + jitter buffer + DL queue, NOT
network. The freshness axis starts at this floor; tau(u) is defined on total
Delta_use (AGEROW realized_age_ms). network_age_ms (UL+DL LUT only, added
freeze-1f=7c28b753) is the radio contribution per load for the load-to-age
table. T12 reruns on 1f (N=4 on 1e was pilot). 50ms bins kept; levels land on
300,450,600,... (sparse bins expected).

## Lead metric correction (2026-09-05)
HANDOFFROW crossing_tick = destination ENTRY (locale_0.contains, x>240), but
the predictive trigger targets source EXIT (locale_1 exit, x>250). The 10m
locale overlap [240,250] makes dest-entry ~0.1s after prepare, so the naive
prepare->crossing "lead" reads ~0.1s. The TRUE predictive lead (prepare ->
source-exit) is ~0.88s at LOOKAHEAD_S=1 and SCALES with config: prepare fires
at npc x=239/227/215/202 for look 1/2/3/4 = (250-x)/12 = 0.9/1.9/2.9/4.0s.
So the lead axis DOES vary; the earlier "detection-capped ~0.1s" note was a
metric artifact, RETRACTED. Source RSU tracks the oncoming continuously from
spawn (frame 14) through the crossing (not occluded from source; paper's
"source observes up to crossing" holds). freeze-1 look2/3/4 -> 0/3 is the
staleness x lead interaction: longer true lead = more stale record WITHOUT the
final update; the final-update fix removes it. True-lead per arm:
frozen1c_truelead_summary.csv. Metric fix for freeze-2: log source_exit_tick
in HANDOFFROW.

## "Launch tail" reframed: it is the safety signal, not noise (2026-09-05)
Every frozen1c collision is ego-vs-STOPPED-TRUCK (x=278) during the overtake
launch (first-contact ego_x 278-293, speed 0-6 m/s). But the RATE is strongly
ARM-DEPENDENT, not an arm-independent artifact:
  warm 1/10, reactive 1/10  (full latent -> good oncoming forecast -> safe gap
                             found -> clean overtake)
  handover_snapshot 7/10, cold 9/10, kf 10/10, edgewarp 10/10  (snapshot/none
                             -> poor forecast -> no safe gap -> ego grinds the
                             truck it is trying to pass)
So the truck contact is the DOWNSTREAM safety consequence of migration
quality; it IS the paper's result (Table 5), not a scenario defect to fix.
The only arm-independent piece is the ~10% residual on the GOOD arms
(warm 1/10) = a genuine launch-imperfection floor capping them at 9/10; small,
Tyler's call whether to reduce it (release stream / delay ego). Prior
"residual launch tail as noise floor for all arms" framing was WRONG and is
corrected here. Headline Table 5 (frozen1c, success = completed & no contact):
warm 9/10, reactive 9/10, handover_snapshot 2/10, cold 1/10, kf 0/10,
edgewarp 0/10.

## look-4 collapse diagnosed: ownership shadow not enforced in publish (2026-09-06)
Block B: look 1 9/10, 2 8/10, 3 9/10, 4 4/10 clean (final update in place).
(a) CONFIRMED protocol gap: the T7 ownership shadow (ownership.py:
publishable/epoch) is NOT wired into any edge publish path (no
OwnershipManager import in edge_manager_*). The destination publishes the
imported (prepared) track immediately, so a consumer receives it BEFORE
commit. Evidence: look4 npc200 prepare=91, first_use=105, crossing/commit=156
-> ego consumes the migrated track 51 ticks (2.5s) BEFORE the final
update/commit. First oncoming (199) is used after commit (57>55), so the leak
bites the longer-lead / second-crossing cases. This violates the paper's
"IMPORT does not publish until final update + commit."
(d) Collapse is between 3s (9/10) and 4s (4/10). computed/mtr/oracle leads
~1s (< the 2.5s cap), so their shadow-leak window is short -> safe regime,
rows likely unaffected. The 2.5s computed cap already guards this.
MECHANISM: at 4s lead the ego acts on the migrated forecast 2.5s early (sees
the oncoming far), its overtake gate mis-times, and it commits into a closing
gap -> truck grind. Gating publish on ownership.publishable (commit) would
deliver the migrated forecast at commit (oncoming near) -> WAIT -> safe.
DECISION FOR TYLER: (i) wire the shadow gate into the publish path (the T7
protocol enforced end-to-end; freeze-2; rerun the look arms; touches the core
publish path for ALL arms - side-effect risk) OR (ii) report look4 as the
demonstration that leads beyond the 2.5s cap are unsafe without the shadow
gate (motivates both the cap and the gate; no rerun). Headline (warm=look1)
and trigger arms (~1s) are unaffected either way.

## freeze-1g: publish gate live, smoke PASSED (2026-09-06)
Gate: imported track is a SHADOW, not published until COMMIT (physical
crossing; arm-independent). freeze-1g = 515136be. Smoke (warm look1/look4,
reactive) assertion PASSED - first_use_tick > commit(crossing) for EVERY
migrated track: look4 npc199 61>55, npc200 161>156 (was 105 pre-gate -> leak
fixed), npc201 263>257; zero pre-commit consumption. PUBGATE suppressed 21x
each on the long look4 leads. warm_look1 clean, reactive clean (PUBGATE=0,
at-crossing has no pre-commit window). (b) no age-out/re-association: single
first_dst_track_tick right after prepare, warm_before_first_use=YES, track
persists prepare->use. look4 improved 4->2 episodes at n=1 (residual launch
tail remains; 5 seeds will confirm if look4 now ~9/10). Batch restarted from
block A on 1g; tail (faults->netem->T19b) + block-lander re-armed on 1g.
T12 stays on 1f (no migration); 5.3+visible re-tag to 1g.

## look4 second mechanism: coast-drift -> duplicate track (2026-09-06)
Gate held (1g: first_use>commit, 0 violations, all collisions are ego-vs-TRUCK
during overtake, not the oncoming). Second mechanism, confirmed on collided vs
clean look4:
- The migrated obstacle track COASTS (dead-reckons at migrated vel_mps) during
  the lead. At 4s it OVER-SHOOTS: collided r1 npc200 migrated tid=1 coasted to
  x=175 while the TRUE oncoming (native re-detection) is at x=156 -> ~18m drift,
  >> tracker match gate (lost_match_dist_m=8m).
- At the crossing the destination natively re-detects the oncoming at its true
  x; association to the drifted migrated track FAILS -> a DUPLICATE track is
  created (tid=7 alongside migrated tid=1, both stamped cid=200). Clean runs
  have only tid=1 (drift < 8m, association succeeds).
- The ego forecast cache keys per carla_id ('c',cid) latest-wins, so two
  same-cid tracks flip-flop the oncoming forecast -> overtake gate mis-decides
  -> ego grinds the truck. (2) compute is secondary; (5) source keeps tracking
  + publishing the NPC through the window (2 OBSTACLE_HANDOFF, share not move);
  (4) inject_latent_into_tracker APPENDS (never dedups) but the duplicate here
  is native-vs-migrated association failure, both rooted in the coast drift.
This is a REAL systems result and a better justification for the 2.5s computed
cap than staleness: leads beyond ~2.5-3s drift past the association gate.
Capped arms (computed/mtr/oracle ~1s) are in the safe regime. FIX (optional,
Tyler's call): identity-aware association - merge a native detection with an
existing migrated track by beacon carla_id, not position, so long leads do not
duplicate. Not a protocol violation (unlike the shadow gate); the cap already
covers the paper's operating point, so reporting look4 as the cap's
justification needs no rerun.

## idfix (identity-aware assoc) PREPARED on branch (2026-09-06)
Branch idfix-assoc (worktree idfix_wt). (a) inject_latent_into_tracker
REPLACEs the same-tid tracklet instead of append. (b) _merge_duplicate_
carla_ids: post-track pass - when >1 tracklet shares a carla_id (migrated +
native, position-assoc failed after coast drift), keep the longer-history
track, adopt the freshest observed position, drop the duplicate from
trajectories/id-map/tracker; logs [IDMERGE]. ARCHITECTURAL NOTE: the mamba
tracker takes raw bboxes (no id), so §3.5 "associate by stable ID" cannot
live in the associator without threading ids through; (b) is the post-track
identity merge instead. Smoke ARMED (idfix_smoke_then_tail.sh): after the
frozen1g batch _all_done, runs warm look1/look4 + reactive from the worktree
(expect no dup tid for cid200 on look4; warm/reactive identical to 1g), then
the normal tail - no batch contention. NOT tagged/restarted; awaiting Tyler's
fix-vs-report pick. Record answered: warm look1 final update has NO persistent
duplicate (only tid=2 for cid199); the look4 dup is native(tid7)-vs-
migrated(tid1), a different phenomenon than the inject re-append.

## Tyler: FIX + timestamp projection (2026-09-06)
Decision FIX (identity-aware assoc = paper 3.5 step 4). Drift explained
(code-verified): the shadow coast PROJECTS from the record (tracklet.predict:
pred=memo_bank[-1]+mig_vel*spf*(tsu+1), memo[-1] never mutated while coasting
-> re-anchored each tick, NOT compounding). So the ~18m over-shoot is a
mig_vel magnitude / spf-steps scaling error, not integration. mig_vel_mps was
unlogged and TRACKER DBG can't split source-native vs dest-shadow (both
tid=1 cid=200 on different edges) -> added [COASTROW] logging; exact magnitude
+ residual drift come from the smoke, not asserted from current logs.
FIX on branch idfix-assoc (844ea3fe): (a) explicit record-anchored projection
+ COASTROW; (b) position gate compares detections to the projected
predicted_last_bbox; (c) identity merge PRIMARY (_merge_duplicate_carla_ids,
same carla_id -> keep history, adopt fresh pose, drop dup) - load-bearing for
the drift since coast already projects; (d) inject REPLACE same-tid.
PLAN (Tyler, 20 seeds): Table 8 (burst+density) stays 1g. Stop 1g before
block D. freeze-1h: Atlas A 6x20 + B 7x20 + E theta x5 (~285 runs ~22h) +
tail; cetus 5.3 matrix accel+flow_visible+accel_visible 6x10 on 1h (headline
cell from Atlas 1h); block D dropped. Corridor Sep 8-9, go/no-go Sep 9.
Smoke armed post-blockC; tag 1h after smoke verified.

## STALE accel runner found (2026-09-06) - major
The accel crash on cetus led to the real root cause: openscenario_1_accel_gt.py
was a stale 565-line file with NONE of the flow runner's instrumentation
(only SCENB; no RUNROW/HANDOFFROW/AGEROW/PUBGATE/COASTROW) and NONE of the
freeze-1b..1g fixes (no final update, no publish gate, no projection). So every
accel-based result (T12 accel, 5.3 accel cells, accel_visible) ran OLD code.
- T12 accel realized_age was never captured (no AGEROW) -> NOT salvageable ->
  must RERUN on the fixed runner (migration-free, tag-immaterial, needs AGEROW).
- The GIL teardown crash on cetus is orthogonal (fires in the ego destroy race
  after the eval dict); data before it was valid but had no AGEROW/RUNROW.
FALLBACK VALIDATED: SCENB tx/by reproduces RUNROW exactly, 120/120 flow logs;
other fields RUNROW-independent. But moot for accel (rerun needed for the fix).
FIX (idfix-assoc 18b6b9a8): regenerated openscenario_1_accel_gt.py +
openscenario_1_accel_visible_gt.py from the flow runner (accel_visible was also
stale). accel differs only by configFile (scenario_1_accel.xml). Smoke now
includes accel_warm to validate the regenerated runner + accel-with-both +
whether flow-based teardown clears the cetus GIL crash.
REVISED CHAINS: Atlas 1h = A6x20 -> B7x20 -> E theta x5 -> faults -> netem
(T19b removed). Cetus 1h = 5.3 matrix -> T19b platoon sweep (100 runs) + T12
accel rerun folded in. Lesson: burst/visible were regenerated from flow;
accel was the one runner never regenerated - check ALL scenario runners share
the flow runner's instrumentation before trusting their rows.

## Runner-signature rule + corridor runner is stale (2026-09-06)
Sign-off now requires, per scenario runner a batch uses: the smoke log shows
RUNROW+HANDOFFROW+AGEROW+PUBGATE+COASTROW present in one run, and the runner
is generated from the flow runner at the tag (record gen commit). A runner
missing any marker does not run.
CORRIDOR (freeze-2) CHECKED NOW: openscenario_1_corridor_gt.py (t13-corridor)
is STALE - RUNROW+HANDOFFROW+SCENB only, NO AGEROW/PUBGATE/COASTROW, and lacks
the freeze-1e..1g + idfix runner-side changes (publish-gate wiring
committed=True + t19_crossing shadow-clear, computed EMA seed, mtr lead gate,
wired backhaul). MUST be regenerated from the freeze-1h flow runner with the
N-locale surgery re-applied (dest resolver _dest_locale_for, crossing_idx
HANDOFFROW, per-crossing rows) before the corridor smoke. Corridor files:
openscenario_1_corridor_gt.py (regenerate), openscenario_1_corridor_gt.yaml
(4-locale), scenario_1_corridor.xml. Note: AGEROW/COASTROW/PUBGATE come from
shared edge/tracker code so a current-code run emits them; RUNROW+HANDOFFROW
are runner-side, so the runner must be current. Cetus 1h order: T12 accel
(7x5 ~3h) FIRST, then 5.3 matrix, then T19b.

## flow_visible ALSO stale; strong currency test (2026-09-06)
Burst 0/5 root: STALE burst runner (813 PUBGATE suppressions, 0 shadow-clears
-> platoon suppressed forever -> ego never sees it), NOT the source/dest
forwarding gap. Same class as accel/corridor. Both burst ego and flow ego are
in locale_0 (destination); flow works only because its runner is current.
flow_visible was ALSO stale re: the gate - it HAD HANDOFFROW but shadow_clear=0
(regenerated after 1f, before the 1g gate). So the 5-MARKER SIGN-OFF IS
INSUFFICIENT: a runner can have all markers yet lack the gate wiring. STRONG
currency test adopted: a scenario runner must be a PURE SCENARIO_NAME diff from
the flow runner at the tag (diff modulo the SCENARIO_NAME line = 0 lines).
Regenerated + verified CURRENT (0 diff lines vs flow): flow_visible, accel,
accel_visible, burst (idfix-assoc 71c9f37e). corridor still to regenerate at
freeze-2. Forwarding fix HELD pending the burst-warm smoke on the fixed runner
(>=4/5 -> at-commit delivery suffices, no forwarding; <4/5 -> forwarding).
Burst smoke adds per-platoon (first_use - crossing) tick + ego min-TTC.

## Process-mgmt lesson: flock chains; stop batch before block D (2026-09-06)
When block C completed on 1g, the batch auto-advanced to block D (accel) while
the smoke chain also started -> 2+ simultaneous ecav (CARLA contention).
Compounded by repeated kill+relaunch leaving duplicate smoke chains (pkill -f
+ setsid did not reliably clear them). FIX: (1) wrap long chains in flock
(exec 9>lock; flock -n 9 || exit) so duplicates exit immediately - only one
runs regardless of relaunch mistakes; (2) a stop-before-block-D marker must
gate the batch when a mid-batch stop is planned, so it does not start the next
block during the handoff. pgrep -f / ps grep self-match the checking command
and setsid doubles the process line - count ecav/CARLA (the contention risk),
not the bash waiters. Landed: frozen1g_trigger_lead_summary.csv (true lead +
warm-before-use per trigger arm: look2/3/4 = 1.88/2.83/3.91s, computed/mtr/
oracle ~0.8s). Smoke running flock-guarded; T9/T22/corridor-regen queued CPU-side.

## Acceptance gate
`scripts/khonsu_accept.py <logdir>` parses each smoke-cell log and prints PASS/FAIL per invariant (runner signature, geometry env, ownership, publish gate, final update, association, coast, planner gate, velocity source, outcome); the freeze tag requires ALL PASS on ALL cells (exit 0). Invariants 7/9 are proxy-checked from COASTROW; 4/5 from HANDOFFROW vs COMMIT REFRESH. Invariant 6 is gated to the migrated cid and SKIPs on cold arms (no migration window); tracker tid churn over an episode is not a protocol property. Invariant 7 is a 15% physical band on coast speed. accel maneuvering (warm) is deferred to freeze-1j and reported non-blocking.

## Known limitation (freeze-1i)
Coast velocity is the mean over the exported record's frames, lagging under acceleration; fix in 1j. The final update at commit replaces the prepared record, so this bias only affects the shadow track between prepare and commit.

## Invalid / held cetus 1j blocks (do not use in figures)
- frozen1j_cetus_hl_n20_rows.csv (loaded-headline N=20): INVALID. Every arm 0/5 including warm and reactive. Runs the NS3_LUT_N=20 override, whose realized age (p50 350-400 ms, max 600-1000 ms) is a forced-N operating point, not the deployed live load (block A live path realizes ~200 ms at the scene count). This measures the override, not the deployed headline. Blocked on item 1 (make the forced-N override realize the live per-message distribution; reproduce ~200 ms at N=scene, pin the diverging line, fix), which runs right after the freeze-1k tag and BEFORE any loaded-headline rerun. warm/reactive/cold arms here are also invalid (0/5). Observation to keep for later: at realized p50 350-400 ms every arm failed, while block A at ~200 ms gives warm 9/10. That is consistent with a freshness cliff near 300 ms for this maneuver. Once the override realizes the live distribution, the loaded block and the controlled T_obs sweep should agree on that cliff; if they disagree, the override is still not reproducing the deployed load.
- frozen1j_cetus_t8_rows.csv (Table 8): HELD pending diagnosis. burst_warm 2/5 (1g 5/5) and density n=8 warm 2/5 (1g 5/5) regressed. Diagnosis: NOT the 1h identity merge and NOT the publish gate: dbl_ticks=0 in every run (no concurrent double-count), and cids_with_multi_tid=8 (burst) appears in BOTH clean and collided runs, so the sequential-reID churn does not correlate with collisions. Clean and collided share the same association signature; only the outcome differs -> planner/seed noise at n=5, not a tracking defect. No fix warranted to the identity merge. Table 8 held from figures until reproduced at 10 seeds on the 1k rerun: run seeds 6-10 for burst_warm and q5_n8_warm (10 more runs) inside the Table 8 rerun. If the 10-seed clean rate sits within noise of the single-crossing warm arm, the n=5 drop is noise and Table 8 is reportable at 10 seeds. If it stays low, diagnose further. Neither cell enters a figure until this resolves. Note: freeze-1g burst_warm is 0/5 (all collided); there is no tagged 5/5 burst baseline. The untagged 5/5 lived in design_sweep_v3_rows.csv, now skipped in load_rows.

## freeze-1k (tag bc26cd3e)
Planner: per-tick overtake recheck with a safe response (complete if past the subject; return behind it only when a following gap back and the return path is clear; otherwise brake and hold, never steer into the subject; no timer-driven state advance). The freeze-1i abort-to-lane truck-rejoin is fixed and validated by acceptance invariant 8 (zero unsafe aborts) on every smoke cell.

Estimator: the migrated velocity spans the last three memo frames over the EXACT per-frame time, using source ticks now stamped per memo frame and carried in the record (memo_tick, int32, ~34 B/transfer added, measured in the byte table). The tracker feed cadence is runtime-variable (jitter-buffer drain gives 1/2/4-tick gaps within and across runs), so the earlier EMA/median/edge_dt constants mis-scaled velocity by ~2-4x (edgewarp 6.86, flow 24.13, flow 3.0); exact ticks give coast |v| 10.9-11.6 (within 5% of 12) on every arm, inv7 passes. The coast PROJECTION keeps the smoothed cadence (EMA): the exact per-frame gap flips 0.1<->0.2 tick-to-tick and jitters the re-anchored projection (measured: warm_s1 do_ov 129 and a collision), so only the one-shot exported velocity uses exact ticks.

Acceptance smoke (cand2_1k, 19 cells): all 9 protocol invariants PASS. warm/kf/edgewarp/handover/one_frame 3/3 clean, t12bo_n4 1/1.

accel note (non-blocking, measured in the campaign at 10 seeds): accel warm is 2/3 in the smoke. The one collision (accel_warm_s3) is a SAFE abort (subj_ahead=20 m) followed by a stop behind the truck, not a truck-rejoin. Cause: the accelerating oncoming outpaces the three-frame velocity estimate, so the ego holds one beat too long before the safe abort. This is the maneuvering-target limitation the paper's maneuvering panel projects (Khonsu ~8/10, not 10/10), not a protocol defect. Gated by inv8, not by an accel outcome threshold.

### 1j data superseded by 1k
All freeze-1j rows (frozen1j_rows.csv and every frozen1j_cetus_*.csv) are SUPERSEDED by the freeze-1k campaign: the planner safety fix (recheck safe response) and the exact-per-frame-tick migrated velocity change every arm's inputs, so 1j numbers are not comparable. Do not read any 1j table into a figure. The stale cetus_1j_tail.sh run (loaded-headline, 5.3 matrix, netem, bisect) was stopped mid-run when cetus was brought onto 1k; its partial 1j logs are superseded and not landed. 1k data lands in frozen1k_rows.csv (Atlas) and frozen1k_cetus_rows.csv (cetus).
