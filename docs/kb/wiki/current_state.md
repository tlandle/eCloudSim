---
updated: 2026-07-27
---
# Current State

Primary context-switching artifact. Read this first after a gap.

---

# Follow Up (Tyler Sync)

Items to raise at next Tyler sync:

- **Phase 2 `-eo` distribution plan (Step 8)** — separate plan needed before building. Discuss scope and what Tyler has already scaffolded (registration server, edge-fusion client) vs. what needs wiring.
- **Curved-road suppression bug `behavior_agent.py:1573` — CONFIRMED and FIXED (`b954dbc9`).** Previously flagged from a code read only; now empirically confirmed as the proximate cause of a real collision. Re-running Scenario B sequential after the RSU-refresh fix (`fef25d41`) produced a sustained (13s, hundreds of `Collision` warnings) ego/stationary-ambulance creep-collision. `curved=True` latched continuously from tick 440 through the collision window in that run, vs flickering ~15-20% of samples with no sustained run in the two immediately-preceding non-colliding runs. Once latched, the step-8 guard's `potential_curved_road` term forced `car_following_flag=True` regardless of `overtake_allowed`/`overtake_counter`, permanently blocking the step-9 overtake path. Fixed by removing that term from the guard — the other two conditions (`not overtake_allowed`, `overtake_counter>0`) are sufficient on their own. This is shared planning logic, not Phase-2-scoped; flag for Tyler sync as a heads-up (already applied) and re-run the regression matrix against it.
- **Track-birth timing findings** — Scenario B debugging established that AB3DMOT needs `min_hits=3` confirmed detections before a track is published. At 50ms world_dt that's 3 ticks. Relevant for Phase 2 advance-warning window claims.
- **Warm import gate (Phase 1.5) — RAISE THIS FIRST.** `handoff_warm_import` defaults False and is settable from nowhere (not YAML, not any `__init__`). It gates both vehicle and obstacle import. So run 12 injected nothing at edge 1 and the 82-tick window is a geometric proxy, not a realized benefit. Needs import-side reconciliation to avoid stale-duplicate ghosts. This is now Step 1 of Phase 2 and gates all distribution work — if the warm-vs-cold delta measures ~0, the framing needs rework.
- **`_PluggableEdgeBase` MRO change** — the 2026-07-27 develop merge removed `AB3DMOTStateTransferMixin` from `_PluggableEdgeBase`'s MRO and replaced it with inline dual-backend dispatch. Check whether any of Tyler's test fixtures do `isinstance(edge, AB3DMOTStateTransferMixin)` — those would now fail.
- **Shared TTC threshold conflict, fixed `653082ed`, confirmed + cleaned up `2635fa03`: `collision_time_ahead` was reused for two physically different checks.** After the two fixes above, Scenario B sequential still produced a single clip against the ambulance. Root cause (confirmed via temporary instrumentation, `ebd5e18d`/`325e9b31`): `collision_manager()`'s `self._collision_check.time_ahead` was shared between (1) the ambulance-braking trigger on ego's own path (correctly tuned for a stationary hazard, fires ~44m out) and (2) the candidate lane-change path safety check inside `overtake_management` (checked against the scenario's fast NPC, 18 m/s constant). The same 4s TTC threshold demands ~72m clearance from an 18 m/s hazard vs. a much shorter braking distance from a stationary one — confirmed exactly: ego committed to the overtake with the NPC 70.7m away, almost precisely 18×4. By then ego had burned nearly all of the ambulance's 44m buffer, leaving ~10m — not enough to complete the lateral shift. Fixed by giving the adjacent-lane candidate-path check ([behavior_agent.py](../../ecav/core/plan/behavior_agent.py), `collision_manager`) its own threshold, `overtake_lane_safety_time_ahead` (default 2.0s — `collision_time_ahead`'s own value before the run-10/11 stationary-obstacle-specific bump to 4), selected via the existing `adjacent_check` flag with no call-site changes needed. **Confirmed across 3 consecutive sequential runs**: 0 collisions, `ghost_brake_events=0`, `MEASURED=29 ticks` identical each time; position-logged dry-run rejections showed the trigger was always the NPC (occasionally logged `carla_id=-1` on an unresolved-identity tick, same track by position) — the ambulance never moved and was never the trigger. Temporary diagnostic logging removed once confirmed.
- **`46216a12` merge regression, fixed `fef25d41`: RSU perception never refreshed in sequential mode (`PredictionLateFusionEdge` only).** The same merge that removed the mixin from `_PluggableEdgeBase`'s MRO also extracted `PredictionLateFusionEdge.run_step()`'s per-tick advance block into `_advance_actors()` and dropped the RSU loop (`rsu.update_info(); rsu.run_step()`) that existed inline at run-12 (`6cefda1a`) — the new method's own docstring still promised it. Any RSU-only edge (edge 0 once its managed vehicle hands off) got zero live tracker output for the rest of any sequential run. Surfaced when re-running Scenario B for Phase 2 Step 1: obstacle handoff never fired in three consecutive runs, and one run ended in an ego/ambulance collision. Fixed by restoring the loop. **Checked `WorldFusionEdge` and `_PluggableEdgeBase` — both correctly gate an RSU refresh under `if not self.run_distributed:` in their sequential paths (`_run_fusion` and `_advance_vehicles` respectively). Confirmed isolated to the late-fusion class.**
- **Open diagnostics in scenario file** — `[EGO-DBG]` and `[SCENB-DBG]` still at WARNING level in `openscenario_multi_edge_right_merge.py`. `[TRACK-DBG]` was resolved in the merge. Remove before Phase 2.
- **PACE MTR training status** — job 10846208 (8x H200, two-stage clean data) submitted 2026-07-07. Check if it completed and what minADE landed.
- **Docker cleanup** — `docker system df` showed build cache ~80GB with ~41GB reclaimable. Run `docker system prune` when convenient.
- **Scenario B MAX_STEP** — ego reaches x=384 at tick=695 but destination is x=600; scenario terminates before full route completion. Not a research blocker (merge sequence is the contribution) but worth noting.
- **LOW PRIORITY — after his defense, not before:** `1f970053` (Tyler's own July 8 commit) introduced a third bug in the same file as the RSU regression: `_ab3d_history_to_trajs`'s zombie filter (`live_tids = {trk.id + 1 ...}`) never matched `output()`'s actual row format (`[trk.id]`, no offset) — every genuinely-alive track was treated as a zombie unless some other track's `id - 1` happened to also be alive. Fixed in `22f455a0`, same commit as a `max_age` grace-period fix for warm-imported tracks. **Bounded check (not a full audit) found no evidence this affects any of his reported dissertation numbers** — Table 6.1 comes from `migration/harness.py`, a fully synthetic microbenchmark that never touches `_ab3d_history_to_trajs`; the two-locale MTR/SSM live-handoff result uses the Mamba/`_PluggableEdgeBase` path, confirmed bug-free; B0.1's vehicle-handoff validation is dated 2026-07-05, three days before the bug was introduced. Only caveat: can't rule out a *later* re-run against buggy `develop` between July 8 and today. Worth a two-minute confirmation with Tyler sometime, not urgent.
- **Reconciliation limitation, left unfixed by design (see Phase 2 plan, "Step 1 conclusion"):** even with the above bugs fixed, a warm-imported KF's constant-velocity-coasted position carries a persistent ~6-15m bias from ground truth that doesn't close over time, well outside AB3DMOT's own ~2m association gate — so the destination's eventual real detection reliably fails to reconcile with the injected track and spawns a new one instead. Tyler's Khonsu design (dissertation Section 6.3.3) already has the right answer — an explicit prepare-commit ownership/epoch protocol that doesn't depend on organic re-association — so this wasn't chased further; the primitive KF-injection mechanism in the current eCAV codebase just doesn't have that layer yet.

---

## Active Branch

`develop` → PR target: `ecav_2_distributed`

Branch is 10 commits ahead of `origin/develop` (includes 2026-07-27 merge of origin/develop's multi-edge predictive latent migration work).

---

## DONE: Phase 1 Scenario B — Right-Merge Obstacle Handoff (2026-07-27)

**Result:** All Phase 1 Scenario B requirements met. Run 12 (commit `6cefda1a`).

> **Scope caveat (established 2026-08-15).** Run 12 validated the *export* half.
> `handoff_warm_import` defaults False and is settable from nowhere — not the
> YAML, not any `__init__` — and the gate covers both `import_vehicle_state`
> (`ab3dmot_state_transfer.py:162`) and `import_tracked_obstacle_state`
> (line 204). Scenario B's edges are `manager_type: late_fusion` →
> `PredictionLateFusionEdge(AB3DMOTStateTransferMixin, …)`, so **no KF was
> injected at edge 1 in run 12**. The 82-tick advance-warning window is tick
> arithmetic over a 60 m geometric proxy computed in the scenario file; it never
> reads edge 1's tracker and would print 82 with the import call deleted. It
> measures opportunity, not realized benefit. Phase 1.5 (enable warm import +
> import-side reconciliation) is now Step 1 of the Phase 2 plan and gates
> everything downstream.

| Event | Tick | Detail |
|---|---|---|
| Vehicle handoff | 63 | vid, 986 bytes, full KF state |
| Ambulance detected by edge predictions | ~305 | dist=42.3m (before local sensor range) |
| **Predictive obstacle handoff** | 161 | NPC at x=98.3, 16.7m before locale boundary |
| RSU1 first detects NPC | 243 | **advance-warning window = 82 ticks (4.1s)** |
| Ego merges right | ~500 | 4.3m lateral clearance, no crash |

**Three bugs fixed to reach this:**
1. `if not beacons:` guard in jitter buffer drain silently dropped all RSU detections after ego handoff (vehicle_manager_list empty → beacons dict empty → guard fired). Fixed: `if not beacons and not objects.get('vehicles'):`.
2. `KeyError` in `_collect_ab3d_detections` — VM in list but not in beacons snapshot (timing race at handoff boundary). Fixed: `if vm.vehicle.id not in beacons: continue`.
3. `_find_obstacle_kf` position fallback always returned None — used `kf.x[1]` (CARLA_z height ≈ 0) instead of `kf.x[2]` (CARLA_y lateral ≈ 140). KF state layout: `[x, y, z, theta, l, w, h, dx, dy, dz]` → `x[0]=CARLA_x`, `x[2]=CARLA_y`. Made NPC-to-KF distance ≈ 140m always >> `max_dist_m=15`. Fixed index to `[2]`.

**Key geometry:** Town06 right-merge (CARLA left-handed frame; +y is the vehicle's RIGHT heading east). Ego + ambulance in lane y≈137, NPC in lane y≈140. RSU0 at x=55 (moved from x=75 to clear near-field blind donut). Locale boundary x=115.

**Plan:** `docs/agent_plans/edge_handoff_phase1_state_transfer.md` (Steps 0–7 checked off).

---

## Next: Phase 2 — Distributed Edges + gRPC State Transfer (2026-08-16)

**Plan:** `docs/agent_plans/edge_handoff_phase2_distributed_state_transfer.md`
**Status (2026-08-22):** Sequential baseline (Step 0 two-edge bring-up + Step 1 cold measurement) is solid — 3 consecutive clean runs, 0 collisions, `MEASURED=29 ticks` reproducible. Step 1's warm-vs-cold comparison ran and its "delta = 0" result was traced to the actual mechanism, not just measured — see "Step 1 conclusion" in the plan for the full writeup. Short version: two real bugs fixed (off-by-one zombie filter, `22f455a0`; `max_age` grace period for warm-imported tracks, same commit), one deeper structural limitation identified and left unfixed (constant-velocity coasting doesn't stay within AB3DMOT's tight ~2m association gate over a long unobserved window) because Tyler Landle's dissertation (Khonsu, Chapter 6) already specifies the correct fix — an explicit prepare-commit ownership protocol that doesn't depend on organic re-association at all. A bounded regression-scope check (not a full audit) found no evidence either bug affects Tyler's reported dissertation numbers (Table 6.1's harness bypasses the buggy code path entirely; the two-locale MTR/SSM result uses an unaffected tracker backend; B0.1's vehicle-handoff validation predates the bug by 3 days) — flagged as worth a quick confirmation with Tyler if it comes up, not urgent given his defense is this week.

Scope: Scenario B only. Decisions D-8 … D-15 recorded in the plan.
Hub transport first, then peer; cost gating config-flagged default off.

**Sequencing is Phase-1.5-first, not distribution-first.** Distributing the
edges before warm import works builds a real gRPC hop carrying a real payload
that the destination discards. Step 1 (sequential, in-process, fast iteration)
must show a non-zero warm-vs-cold delta before Steps 2–6 are worth doing.

### What landed in 983644ae

**Step 0 code (no CARLA run yet):**
- `edge_registration_server.py`: D-15 edge binding from `container_name` not arrival order; collision/parse errors now fatal.
- `openscenario_multi_edge_right_merge.py`: `zip(edge_list, fusion_clients)` → explicit `fc_by_edge_idx` map; NPC uniqueness check; `fc_by_edge_idx = {}` guard at top of `run_scenario`.

**Step 1 code (no CARLA run yet):**
- `openscenario_multi_edge_right_merge.yaml`: `handoff_warm_import: false` in `edge_base` (set true to enable warm import).
- `PredictionLateFusionEdge.__init__`: `self.handoff_warm_import = bool(cfg.get(...))` + `self._first_track_publish_tick: Dict[int,int] = {}`.
- `PredictionLateFusionEdge.import_vehicle_state` (class override): added `_warm_import_enabled()` gate (the override was bypassing the mixin's gate).
- `WorldFusionEdge.__init__`, `_PluggableEdgeBase.__init__`: same `handoff_warm_import` read.
- `_PluggableEdgeBase`: added `_warm_import_enabled()` + gated `_import_track_latent` (F8 reconciliation).
- `daemon.py`: `MigrationPayload.deserialize(payload.serialize())` round-trip in both `request_handoff` and `transfer_obstacle_state`. Confirmed: `payload_bytes()=1050` ≠ `len(pickle)=1696` — D-14 parity invariant intact.
- `_ab3d_history_to_trajs`: optional `tick` parameter; `[TRACK_PUBLISH]` log on first per-cid trajectory appearance. Scenario closing log adds `MEASURED` window beside `PROXY` window.

**Import-side reconciliation analysis:** `max_age=6` → warm track pruned ~24 world ticks after injection (tick ~185). RSU1 first detects NPC at tick ~243. No live warm track at re-detection → no ghost-duplicate risk at current operating point. Confirm with run.

### What still needs CARLA

- [x] Step 0: single-edge `-eo` regression (`openscenario_3_edge_late_fusion`) — clean, no regression.
- [x] Step 0: two-edge `-eo` right-merge bring-up — clean after the `[EDGE-ONLY]` print fix (`0767ed3a`).
- [x] Step 1 cold baseline: sequential right-merge, `handoff_warm_import: false` — **`MEASURED=29 ticks`, confirmed reproducible across 3 runs** (`XuwE8k`, `exyzDi`, `vG76bj`), 0 collisions, `ghost_brake_events=0` each time.
- [ ] Step 1 warm run: same scenario with `handoff_warm_import: true` on edge 1. Compare `[TRACK_PUBLISH]` tick for the NPC against the cold baseline.
- **Gate:** if measured warm-vs-cold delta ~0, re-plan before Steps 2–6.

**Three unrelated regressions found and fixed while chasing a clean sequential baseline** (none were Phase-2/warm-import bugs — all were pre-existing, surfaced by finally re-running Scenario B sequential after a long gap):
1. `fef25d41` — RSU perception never refreshed in sequential mode (`46216a12` merge dropped a loop `_advance_actors()`'s docstring still promised).
2. `b954dbc9` — curved-road suppression bug (previously flagged, Tyler-scope, now confirmed via a real collision and fixed).
3. `653082ed` — `collision_time_ahead` shared between two physically different hazards (stationary ambulance vs. 18 m/s NPC); split into `overtake_lane_safety_time_ahead`.

Architectural findings behind the plan:

- **F1** — under `-eo` the tracker lives in the container (`Edge_PerformFusion` overwrites the VM/RSU lists with `_FeatureStub`s each tick); the base-process edge is a perception+planning shell whose tracker never advances.
- **F2** — consequently Scenario B's per-tick snapshot loop under `-eo` exports `kf_state=None` and the handoff silently degrades to nothing. Nothing raises.
- **F3** — the handoff decomposes into *routing* (`relinquish`/`accept` on base shells — unchanged from Phase 1) and *tracker state* (must cross the wire). Only the second needs RPCs.
- **F4** — the container sees only tracks, so **one** RPC pair keyed on `carla_id` + position hint covers both vehicle and obstacle transfer.
- **F5** — `edge_id` is assigned by registration arrival order; `zip(edge_list, fusion_clients)` holds only because `start_actors.sh` serializes container launch. Wrong-locale binding would present as a geometry bug. Bind from `EdgeRegistrationInfo.container_name` instead.
- **F6** — both `collect_features` impls call `update_information()`, running the comm model (latency, packet loss, beacon temp-ids, jitter push) in the base process where nothing drains it. `BeaconIdManager._random_temp_id()` uses `random.randint`, so base and container temp ids diverge — fine, since the RPC contract is `carla_id`-keyed, but base-side beacon state must never be consulted for identity.
- **F7** — Single-edge `-eo` is validated (Phases 3 and 4 complete in `edge_only_distributed_mode.md` — late fusion confirmed 2026-05-31, WorldFusion confirmed same period). Two-edge `-eo` with ScenarioRunner, handoff block active, and two separate containers is the new, unexercised combination; that is the Step 0 risk.
- **F8** — `AB3DMOTStateTransferMixin._import_track_latent` honors `handoff_warm_import`; `_PluggableEdgeBase._import_track_latent` has no gate and always injects. Mamba and AB3DMOT results are not comparable until this is reconciled.

**Parity invariant:** the cost model must keep using `MigrationPayload.payload_bytes()`, never `len(pickled_payload)` — the pickle wire size is larger and host-dependent, and switching would break the Phase 1 ↔ Phase 2 byte comparison.

---

## NSDI Paper: scale_out_nsdi (2026-07-07)

Branch `scale_out_nsdi` (commit `33c8a07`). Systems positioning complete: related-work problem-class abstraction, camera-networks subsection, ClairvoyantEdge distinction, system model corrected (locale = CONFLICT ZONE not RSU). Boundary framing: "static canvas, dynamic assignment, ASYMMETRIC elasticity." 74 `\ptag` intent tags, AV-term glosses for systems reviewers. 15pp clean build.

Tyler reading both manuscripts. Professor email drafted.

---

## Dissertation Proposal (DONE, 2026-07-05)

Five research thrusts, chapter order fixed, P2–P6 complete, 60 pages clean build, 0 warnings. `\ptag` intent tags visible until advisors approve; flip one-line macro in `main.tex` to hide.

---

## Scale-out B0 (2026-07-05)

**B0.3 DONE — live full-latent migration in CARLA.** `openscenario_3_multi_edge_mamba`. At tick 60: full memo bank (10 frames, 776 B) exported from edge0, injected warm at edge1. Tracklet survived; no post-handoff exceptions. Known gaps: warm-vs-cold DELTA needs B4 metrics; `SOTAEdge.evaluate()` NotImplementedError at cleanup (non-fatal).

**B0.2 DONE — Mamba latent through edge dispatch.** Mamba3DMOT in tracker registry. `_PluggableEdgeBase` dispatches: Mamba → full latent via factories; AB3DMOT → KFState. Both verified under `opencda310` (`test_mamba_edge_migration.py`): banks byte-identical, id preserved, ~1.3 KB.

**B0.1 DONE — live edge migrates KF state.** `PredictionLateFusionEdge` export/import wired with real AB3DMOT KF snapshot (mean, covariance, hits≥min_hits, velocity). Round-trip verified.

---

## WF→MTR Training (PACE, status as of 2026-07-07)

Goal: train MTR on WorldFusion fused BEV features (CMP-style) for Multi-V2X.

**Current status:** Retrain job 10846208 submitted 2026-07-07 (8x H200, two-stage, ~12h). Used clean re-export (174 GB, 52 zones) after discovering the prior 8-GPU run (10804312) used `train=True` in export → per-frame random augmentation scrambled trajectories → model plateau at ADE ~31m (static baseline). Fixed: `export_wf_for_mtr.py` now uses `train=False`.

**Infrastructure validated** (2x H200 job 10802480): DDP, eval after every epoch, best_model save, clean exit, finite metrics (minADE=nan fix: skip objects with `final_valid_idx<1`).

**Correct model:** `worldfusion_multiv2x_translaug_finetune` (epoch 27/45 local at `ecav/ml_manager/models/`). NOT `caronly_aug` (wrong pick; scene-overfits).

**Key design decision:** everything in RSU-EGO frame. GT boxes + detection positions both in ego frame. No AB3DMOT, no rekey — GT-anchored Hungarian association (gate 2.0m) for building training data. Verified rsu_66: det pos error mean 0.42m, per-frame recall 44%, 1086 trainable samples/zone.

**PACE artifacts:** `$PROJECT/wf_mtr_translaug.tar` (174 GB, READY marker dropped after upload), `$PROJECT/mtr_code.tar`. Sbatches in `ecav/core/prediction/mtr/tools/scripts/`. Lock file pattern: `$PROJECT/mtr_wf_smoke.lock` — always `rm -f` before resubmitting.

**Known issue:** intermittent RSU detections (44% recall) give <2 valid past obs for some records — loader skips those (BatchNorm crash on single-point sequences). CMP-strict would drop these; MTR masks missing past via `obj_trajs_mask`, so lenient selection is correct.

---

## Phase 1 State Transfer — Steps 0–6 Reference

All committed. See `docs/agent_plans/edge_handoff_phase1_state_transfer.md` for full checklist.

- `migration/payload.py`: `KFState`, `TrackLatent`, `MigrationPayload`.
- `migration/binding.py`: `HandoffManager`, `evaluate()` → `Optional[HandoffEvent]`.
- `edge_manager_base.py`: `export/import_vehicle_state`, `relinquish`, `accept` stubs.
- `ab3dmot_state_transfer.py`: `AB3DMOTStateTransferMixin` — AB3DMOT-aware export/import with position fallback for unmanaged obstacles; mixed into `PredictionLateFusionEdge`.
- `edge_manager_pluggable_base.py`: dual-backend inline dispatch (Mamba + AB3DMOT) — replaces mixin inheritance as of 2026-07-27 merge.
- `sim_api.py`: `_vehicle_state_store`, `store/retrieve_vehicle_state`.
- `migration/link.py`: `TransferCost`, `InterLocaleLink.model_transfer`.
- `migration/daemon.py`: `SequentialMigrationDaemon.request_handoff`, `transfer_obstacle_state`.
- Metrics sink: `record_handoff_cost` on `ScenarioManager`; `handoff_eval` in `EvaluationManager`; `HAND-OFF MIGRATION` block in evaluation report.
- Scenario A validated (2026-06-13): tick=60, 986 bytes, ghost_brake_events=0.

**AB3DMOT warm import is OFF by default** (`handoff_warm_import = False`). Payloads are exported and logged; destination tracker is untouched until Phase 1.5.

---

## Paper 1 (safety_envelope_sensys) — SUBMITTED

Branch `paper-closed-loop-recreate`. Full analysis in session logs 2026-05-30 through 2026-06-02. Short summary: SBA (anchoring) expands safety envelope from ~100ms logic cliff to ~400ms physics limit. Oracle confirms architecture works when detection is clean. Detailed eval and taxonomy analysis archived in session logs; not recapped here.

---

## Key Invariants

**RSU = agent 0 in WorldFusion.** Fusion layer warps all agents' BEV features into agent 0's frame. RSU must always be first in `rsu_manager_list` and first in `IntermediateFeaturesBatch`. See commit `2a9db949`.

**AB3DMOT detection format = 8 columns.** `_collect_ab3d_detections` produces `[h,w,l,x,y,z,theta,confidence]` — 8 columns. Empty early-exit arrays must be `np.empty((0, 8), np.float32)`. `(0, 7)` is a recurring mistake (2026-05-31 root cause: `Box3D.__init__` defaults `s=0.0`; see session log). `array2bbox_raw` handles 8-column input gracefully (`data[-1]` → `bbox.s`); the `[:7]` strip in `model.py output()` is for the KF-state output path only.

**AB3DMOT KF state vector:** `[x, y, z, theta, l, w, h, dx, dy, dz]` (dim=10). KITTI camera convention: `x[0]=CARLA_x`, `x[1]=CARLA_z (height)`, `x[2]=CARLA_y (lateral)`. Position fallback in `_find_obstacle_kf` must use `x[0]` and `x[2]`, NOT `x[1]`.

**Clean CARLA session required for testing.** `ActorTransformSetter` teleport fails in a dirty session. Always restart CARLA before standalone testing.

**Late fusion vs WorldFusion: different feature field.** Late fusion: `VehicleUpdate.pickled_agent_objects` (YOLO detections). WorldFusion: `IntermediateFeatures.spatial_features` (BEV).

**AB3DMOT Kalman filter uses Joseph form** (commit `902aef96`). Simple `(I-KH)P` causes covariance collapse; Joseph form `(I-KH)P(I-KH)^T + KRK^T` is numerically stable.

**`collision_time_ahead: 4`** in `openscenario_multi_edge_right_merge.yaml`. At 2, hazard fires at ~22m → ego stops 5m from ambulance → collision on creep. At 4, fires at ~44m → ego stops with clearance.

---

## Regression Matrix

| # | Fusion | `-l` | `-d` | Status |
|---|---|---|---|---|
| 1 | WorldFusion | no | no | ✓ 2026-04-26 |
| 2 | WorldFusion | yes | no | ✓ 2026-04-29 |
| 3 | WorldFusion | no | yes | ✓ 2026-05-03 |
| 4 | WorldFusion | yes | yes | ✓ 2026-05-03 |
| 5 | Late fusion | no | no | ✓ 2026-06-01 (re-validated post-develop-merge) |
| 6 | Late fusion | yes | no | pending |
| 7 | Late fusion | no | yes | pending |
| 8 | Late fusion | yes | yes | pending |

Tests 6–8: pipeline believed working from `96ab86c0` fixes; not yet run. Always use clean CARLA session.

---

## Backlog

- **`isEdge_` dead code** (`ecloud_server.cc`) — gates `pendingReplies_` path irrelevant in edge mode. Legacy. Remove in a cleanup PR.
- **`vehicle_count`/`num_completed_vehicles` as class vars** (`sim_api.py:276–278`) — mutated on instance; works but unclear. Fix when `ScenarioManager.__init__` is touched for another reason.
- **`PlanningMetrics` invalid fields** — `distance_traveled_m` skips first 100 ticks; `edge_ticks_total` + 5 siblings never incremented. Fix or delete.
- **`print` → `logger` driveby** — any file touched gets a sweep. Progressive only.
- **multiv2x_mtr placeholder cache** — `ecav/ml_manager/models/multiv2x_mtr/multiv2x_fused_features_placeholder/` removed from git; Tyler to confirm intended workflow.

---

## Related

- [WorldFusion Performance](worldfusion_performance.md)
- [Architectural Decisions](decisions.md) — D12 (gRPC migration), D13 (standalone servers), D14 (log-based readiness)
- [Architecture](architecture.md) — process topology, ML server ports
- [Plans Index](plans_index.md)
- [Research](research.md)
- [Phase 1 Plan](../../agent_plans/edge_handoff_phase1_state_transfer.md)
- [Scale-out Eval Plan](../../agent_plans/scale_out_evaluation.md)
