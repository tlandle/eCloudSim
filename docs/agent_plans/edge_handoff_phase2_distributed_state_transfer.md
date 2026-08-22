# Edge Handoff Phase 2 — Distributed Edges + gRPC State Transfer

**Branch:** `develop` → PR target `ecav_2_distributed`
**Status:** Planning
**Created:** 2026-08-15
**Predecessor:** [edge_handoff_phase1_state_transfer.md](edge_handoff_phase1_state_transfer.md) (Steps 0–7 DONE, run 12 / `6cefda1a`)
**Depends on:** [edge_only_distributed_mode.md](edge_only_distributed_mode.md) (Phases 1–2 built, Phase 3–4 **unvalidated**)

---

## Goal

Lift Scenario B (Town06 right-merge, obstacle KF handoff) out of the single
process. Both edges run as standalone containers; the migration payload
crosses a real gRPC hop; the modeled transfer cost is measured on that hop and,
under a config flag, gates when the destination applies the state.

Scope is **Scenario B only**. Scenario A's mechanism is a strict subset (vehicle
ownership move, no obstacle KF share) and lifts for free once the transport
exists; it is deferred to a later regression pass.

Actor distribution (`-d`, Docker vehicle clients) remains a distinct Phase 3.
`-eo` and `-d` are orthogonal axes.

---

## The finding that reorders this plan

**In run 12, no KF state was ever injected at edge 1.**

`AB3DMOTStateTransferMixin._warm_import_enabled()`
([ab3dmot_state_transfer.py:151](../../ecav/core/application/edge/edge_manager/ab3dmot_state_transfer.py#L151))
reads `getattr(self, 'handoff_warm_import', False)`. That attribute is set
nowhere — not in `openscenario_multi_edge_right_merge.yaml`, not in any edge
manager `__init__`, not in any scenario file. The gate covers **both**
`import_vehicle_state` (line 162) and `import_tracked_obstacle_state` (line
204). Scenario B's edges are `manager_type: late_fusion` →
`PredictionLateFusionEdge(AB3DMOTStateTransferMixin, _BaseEdgeManager)`, so both
edges take the gated path.

So what run 12 actually established:

| Claim | Status |
|---|---|
| Predictive trigger fires at the right tick/geometry | **Real** |
| Source edge locates the NPC's KF and exports it | **Real** (this was bug 3's fix) |
| Payload is well-formed, sized, and cost-modeled | **Real** |
| Destination edge receives a warm track | **Never happened** — `tracker untouched` |
| "advance-warning window = 82 ticks" | **Opportunity, not realized benefit** |

The window number is tick arithmetic: `rsu1_first_detect_tick - handoff_tick`,
computed in the scenario file from a 60 m geometric proxy
([openscenario_multi_edge_right_merge.py:484-488](../../ecav/scenario_testing/openscenario_multi_edge_right_merge.py#L484-L488)).
It never reads edge 1's tracker. It would print 82 with the import call deleted
entirely.

This does not contradict Phase 1 — Phase 1 explicitly deferred warm import to
"Phase 1.5" and `current_state.md:124` records the gate correctly. But
`current_state.md:36` ("All Phase 1 Scenario B requirements met") and the run-12
table read as a mechanism result, and they are not one yet.

**Consequence for sequencing:** distributing the edges before enabling warm
import builds an elaborate no-op — a real gRPC hop carrying a real payload that
the destination discards. Phase 1.5 therefore comes *first*, in sequential mode,
where iteration is fast and there is no container rebuild in the loop. Prove the
state does something, then prove it survives a process boundary. Debugging "is
the state crossing?" and "does crossed state help?" at the same time, across a
container boundary, is the failure mode to avoid.

---

## Architectural findings

### F1 — In `-eo`, the tracker lives in the container

`Edge_PerformFusion`
([edge_process.py:318-322](../../ecav/ecav2/edge_process.py#L318-L322))
overwrites `rsu_manager_list` and `vehicle_manager_list` with `_FeatureStub`s
each tick, then calls `run_step()`. Everything stateful in the tracking
pipeline — `self.tracker`, `track_to_carla`, `beacon_id_mgr`, `_jitter_buffer`,
`_tracker_output_history` — is container-side and persists across ticks.

The base-process edge object is a **perception + planning shell**: it owns real
`VehicleManager`s and `RSUManager`s (CARLA handles, localizers, planners,
controllers), runs `collect_features` and `apply_predictions`, and never
advances a tracker.

### F2 — Scenario B's handoff silently degrades to nothing under `-eo` today

The per-tick snapshot loop
([openscenario_multi_edge_right_merge.py:254-258](../../ecav/scenario_testing/openscenario_multi_edge_right_merge.py#L254-L258))
calls `edge.export_vehicle_state()` on the *base-process* edge, whose
`tracker.trackers` is empty. `_kf_for_carla_id` returns `None`,
`_kf_to_payload` produces a payload with `kf_state=None`, and the destination
logs "no KF state — cold start". Nothing raises. Combined with the finding
above, `-eo` Scenario B would report handoffs and costs while moving zero state
through zero mechanism.

### F3 — The handoff decomposes into two independent moves

This is the load-bearing simplification.

| Move | Where it happens in `-eo` | Changes from Phase 1 |
|---|---|---|
| **Routing** — which fusion client sees this vehicle's features and gets its predictions | Base process: `relinquish` / `accept` on the edge shells, which is exactly what `collect_features` iterates | **None.** Phase-1 code works as-is. |
| **Tracker state** — the KF snapshot | Container: `raw.trackers`, `track_to_carla` | **New RPCs required.** |

Only the second move crosses the wire.

### F4 — One RPC pair covers both vehicles and obstacles

The container sees only tracks; the vehicle/obstacle distinction is purely a
base-process ownership concept. Container-side, `import_vehicle_state` and
`import_tracked_obstacle_state` both reduce to `_inject_warm_kf`. The obstacle
variants are strictly more general: `export_tracked_obstacle_state` has no
VM-list gate and supports the position fallback that unmanaged NPCs need.

So the container exposes **`Edge_ExportTrackState` / `Edge_ImportTrackState`,
keyed on `carla_id` with an optional position hint** — used by both
`request_handoff` and `transfer_obstacle_state`. The daemon decides whether to
additionally do `relinquish`/`accept`.

### F5 — Process IDs must be deterministic for all actor types

For the handoff to be debuggable and reproducible, the mapping from logical role
(edge 0, ego vehicle, RSU 1) to process/CARLA ID must be stable across runs and
independent of spawn/registration order.

**Edges:** `_EdgeRegistrationServicer.Edge_Register`
([edge_registration_server.py:85-88](../../ecav/scenario_testing/utils/edge_registration_server.py#L85-L88))
assigns `edge_id = self._next_edge_id++`. The scenario then does
`zip(edge_list, fusion_clients)`, which assumes `fusion_clients[i]` ↔
`edge_cfgs[i]` ↔ `locale_i`. With one edge that is vacuously true. With two, it
holds only because `start_actors.sh:495` serializes container launch behind
`wait_for_container_log "$container_name" "edge-only ready"`. That is an
implicit, load-bearing coupling with a wrong-locale failure mode that would look
like a geometry bug, not a wiring bug. `EdgeRegistrationInfo` already carries
`container_name` (field 6) and containers run with `-e "HOSTNAME=edge_$e"`. Bind
the index explicitly from that and fail loudly on collision.

**Vehicles and RSUs:** CARLA assigns actor IDs at spawn time. Spawn order and
server-side ID assignment can vary between runs (connection timing, server load,
prior state). The scenario must bind its logical roles (ego, NPC, RSU 0/1) by
attribute or role name at actor-discovery time — not by the order actors appear
in `world.get_actors()`. This is especially critical for the payload's
`persistent_vehicle_id` and for `export_tracked_obstacle_state`'s `carla_id`
argument; a mis-bound ID produces a missed-KF-export that logs as a warning, not
an exception. Audit the actor-discovery loop in the right-merge scenario file and
assert that each logical role is resolved by identity, not position in a list.

### F6 — `collect_features` runs the comm model in the wrong process

Both `collect_features` implementations call `self.update_information(step)`
([late fusion:1184](../../ecav/core/application/edge/edge_manager/edge_manager_prediction_late_fusion_ab3dmot_linear_predictor.py#L1184),
[worldfusion:727](../../ecav/core/application/edge/edge_manager/edge_manager_worldfusion_ab3dmot_linear_predictor.py#L727)),
which drives `fusion.collect_and_push(...)` → latency model, packet-loss draws,
`beacon_id_mgr` temp-id assignment, jitter-buffer push. The base process never
drains that buffer; the container runs the identical pipeline again on the
shipped detections and *its* result is authoritative. The WorldFusion docstring
calls this "harmless — the jitter buffer just accumulates unused frames."

For WorldFusion it is close to harmless: `update_information` is also what
triggers BEV feature extraction, which `collect_features` genuinely needs. For
late fusion it is not obviously needed — `vm.agent.objects` is populated by
`vm.update_info()` inside `apply_predictions`, not by `collect_and_push`. The
duplicate call burns `random` draws in two processes and double-counts profiler
comm metrics.

`BeaconIdManager._random_temp_id()` uses `random.randint`
([beacon_id_manager.py:117-119](../../ecav/core/application/edge/beacon_id_manager.py#L117-L119)),
so the two processes assign *different* temp ids for the same `carla_id`. That
is fine for the RPC contract — everything is keyed on `carla_id` and
`_resolved_carla_id` runs container-side against the container's own map — but
it means the base-side beacon state is meaningless and must never be consulted
for identity. Worth confirming, not assuming, in Step 0.

### F7 — Single-edge `-eo` is validated; two-edge `-eo` is new territory

`edge_only_distributed_mode.md` Phases 3 and 4 are complete. Late fusion `-eo`
(`openscenario_3_edge_late_fusion`) was confirmed working in the 2026-05-31
session (JRapp ran it: `ghost_brake_events=0`, `avg_speed_mps=9.94`, profiler
written) and re-validated post-develop-merge in 2026-06-01. WorldFusion `-eo`
was validated in the same period.

What has *not* been exercised is the combination that Scenario B requires under
`-eo`: two edge containers, a ScenarioRunner subprocess, and the handoff block
active. That is a narrower claim than "never run end to end," but it is still
the dominant Step 0 risk.

### F8 — Two state-transfer implementations have diverged

`AB3DMOTStateTransferMixin._import_track_latent` honors `handoff_warm_import`;
`_PluggableEdgeBase._import_track_latent`
([edge_manager_pluggable_base.py:215-262](../../ecav/core/application/edge/edge_manager/edge_manager_pluggable_base.py#L215-L262))
has no such gate and always injects. Scenario B uses the mixin family;
SOTAEdge/AdaptiveEdge use the pluggable family. Reconcile the gate semantics in
Step 1 so "warm import on" means the same thing in both, or the Mamba and
AB3DMOT results are not comparable.

---

## Design

### Transport abstraction

The daemon should not know whether the destination edge is a Python object or a
container. Introduce `migration/transport.py`:

```python
class StateTransport(Protocol):
    def export(self, edge, carla_id, position=None) -> Optional[MigrationPayload]: ...
    def import_(self, edge, carla_id, payload, apply_after_ticks: int = 0) -> None: ...
```

Three implementations:

| Transport | export / import | Phase |
|---|---|---|
| `InProcessTransport` | direct `edge.export_tracked_obstacle_state()` / `import_tracked_obstacle_state()` | 1 (current behavior, no change) |
| `GrpcHubTransport` | base dials `Edge_ExportTrackState` on src, `Edge_ImportTrackState` on dst | 2 step 4 |
| `GrpcPeerTransport` | base dials `Edge_MigrateTrack` on src; src dials dst | 2 step 5 |

`SequentialMigrationDaemon.__init__(transport=InProcessTransport())`. The
scenario file's handoff block becomes mode-agnostic — the only `-eo` branch is
which transport is constructed. Phase 3 (C++ shared buffer) becomes a fourth
implementation rather than a fourth code path.

`relinquish` / `accept` stay direct member calls on the base shells (F3) and do
**not** go through the transport.

### Sequencing within a tick

Sequential and `-eo` must have identical ordering or the parity check is
meaningless. Current sequential order per tick:

```
1. scenario_manager.tick()          CARLA advances
2. snapshot upload                  export → store
3. handoff evaluate + execute       relinquish / export / import / accept
4. NPC predictive + reactive trigger
5. edge.run_step(step)              perceive → fuse → track → predict → plan
```

`-eo` replaces only step 5 with `collect_features` → `fuse` → `apply_predictions`.
Steps 2–4 keep their position, so at handoff time the source container still
holds tick N−1's stubs and its full tracker — the export sees exactly the state
sequential mode would have seen. This ordering is a **contract**, not an
accident; assert it in a comment and in the smoke test.

### Proto surface

```proto
message TrackStateRequest {
  int32  carla_id     = 1;
  bool   has_position = 2;
  double position_x   = 3;
  double position_y   = 4;
  int32  tick         = 5;
}

message TrackStateResponse {
  int32 carla_id       = 1;
  bool  found          = 2;
  bytes pickled_payload= 3;   // MigrationPayload.serialize()
  int32 model_bytes    = 4;   // MigrationPayload.payload_bytes() — NOT len(pickled_payload)
}

message TrackStateImport {
  int32 carla_id        = 1;
  bytes pickled_payload = 2;
  int32 tick            = 3;
}

message MigrateTrackRequest {          // peer step only
  int32  carla_id      = 1;
  string dst_edge_ip   = 2;
  int32  dst_edge_port = 3;
  bool   has_position  = 4;
  double position_x    = 5;
  double position_y    = 6;
  int32  tick          = 7;
}

rpc Edge_ExportTrackState(TrackStateRequest)  returns (TrackStateResponse);
rpc Edge_ImportTrackState(TrackStateImport)   returns (Empty);
rpc Edge_MigrateTrack(MigrateTrackRequest)    returns (TrackStateResponse);
```

**Parity invariant:** `InterLocaleLink.model_transfer` must keep using
`MigrationPayload.payload_bytes()`, never `len(pickled_payload)`. The pickle
wire size is larger and host-dependent; switching to it would make Phase 2
report different bytes than Phase 1 for the same handoff and destroy the
comparison. `model_bytes` is carried in the response purely so the base process
can cross-check the container agrees.

Handlers are sync `def` like `Edge_PerformFusion`, so they serialize against
fusion on the same execution path. The base process is single-threaded and
blocks on each RPC in order, so there is no concurrent-mutation window in
practice — but state a guard rather than rely on it.

### Cost gating

Default **off**; enabled by `handoff_cost_gates_apply: true` in the edge YAML.

Timing logic stays base-side, where sim time is known. The daemon computes
`apply_after_ticks = ceil(total_ms / (world_dt * 1000))` and queues the import
in a `_pending_imports` list drained at the top of each tick. The container
stays dumb — it applies whatever `Edge_ImportTrackState` it is handed,
immediately. This keeps the container free of sim-clock assumptions and makes
the delay trivially inspectable in the base-process log.

At `world_dt = 50 ms` and a ~986 B payload, `total_ms` ≈ 2.4 ms → 0 tick delay.
The gate is therefore invisible at the current operating point; its value is the
sensitivity sweep in Step 6, where `serialize_rate_ms_per_byte` and the latency
model are scaled up to find the link budget at which the advance-warning window
collapses. That is the result worth reporting.

---

## Hypotheses

**H1 (mechanism).** A warm-imported KF at the destination lets edge 1 publish a
confirmed track for the NPC materially earlier than a cold start.

*Counter-H1a:* AB3DMOT's `min_hits=3` dwell is only 3 ticks (150 ms at
`world_dt=50 ms`). If the destination's own detections are available within
~3 ticks of the handoff, the warm import saves almost nothing and the 82-tick
window is an artifact of the 60 m geometric proxy, not of tracking.
*Counter-H1b:* Injecting a track for an object the destination cannot yet see
produces a free-running prediction that drifts, and reconciliation on first
detection creates a duplicate ghost — a net harm. This is exactly the risk that
motivated gating warm import off in Phase 1.

*Verification (Step 1, sequential, no CARLA changes):* instrument the tick at
which edge 1 first publishes a track for the NPC's `carla_id`. Run warm-import
on vs off, same seed, ≥3 runs each. The delta is the realized window. Replace
the geometric proxy in the scenario's closing log with this measured value.
**If the delta is ~0, H1 is falsified and the research framing needs rework
before any distribution work is worth doing.**

**H2 (transport fidelity).** Moving the payload across a gRPC hop changes
nothing observable except where the bytes travel.

*Counter-H2:* the `-eo` split introduces a one-tick lag — `collect_features` at
tick N ships `objects` populated by `apply_predictions` at tick N−1 — which
shifts detection timing and therefore handoff and first-track ticks.

*Verification (Step 4):* same scenario, same seed, sequential vs `-eo`. Compare
handoff tick, `payload_bytes` (must be **exactly** equal), first-warm-track
tick, advance-warning window, ego merge tick, collision count. Tolerance ±2
ticks on timing, exact on bytes. A systematic 1-tick offset confirms
Counter-H2 and is acceptable if characterized; a larger or non-constant offset
is a bug.

**H3 (cost sensitivity).** There is a link budget above which the modeled
transfer cost consumes the advance-warning window.

*Verification (Step 6):* sweep `serialize_rate_ms_per_byte` and the latency
model with gating on; plot realized window vs `total_ms`. The crossing point is
the result.

---

## Implementation checklist

### Step 0 — `-eo` baseline (no handoff)

The infrastructure risk. Do this on a single-edge scenario first so no handoff
logic is in the picture.

- [x] Run `openscenario_3_edge_late_fusion` under `-eo`, single edge, clean CARLA session — ran clean, no regression (2026-08-22).
- [ ] Confirm the vehicle drives on container-fused predictions (not on empty prediction lists silently falling back to local planning)
- [ ] Confirm `edge_profiler_<ts>.json` is written from the container via `Edge_EndScenario`
- [ ] **F6 check:** determine whether `collect_features` needs `update_information()` for late fusion. Instrument `vm.agent.objects` population order across `apply_predictions` → `collect_features`. If not needed, drop the call for late fusion and note the WorldFusion path still requires it.
- [ ] **F6 check:** confirm base-side and container-side `beacon_id_mgr` temp ids diverge, and that no identity path in the base process consults them
- [ ] Characterize the one-tick lag (Counter-H2) explicitly: which tick's detections does the container fuse at tick N?
- [ ] Two-edge `-eo` bring-up on the right-merge scenario, handoff block disabled — first attempt (2026-08-22) hit a false failure: `start_actors.sh`'s `-eo` readiness gate polls the base log for the literal string `[EDGE-ONLY]`, which only existed in `openscenario_3_edge_worldfusion.py`/`openscenario_3_edge_late_fusion.py` — nobody added it to the right-merge scenario file because two-edge `-eo` had never been run on it (exactly the F7 gap). Log showed both edges registered, fusion clients connected, actors spawned, RSUs/locales built — the simulation was healthy; the shell script's 60s timeout killed a working run. Fixed: added the matching print. Retry pending.
- [x] **D-15 / edge binding:** bind `edge_id` from `EdgeRegistrationInfo.container_name` (`edge_<n>`), not arrival order; fail loudly on collision or unparseable name — `983644ae`
- [x] Replace the implicit `zip(edge_list, fusion_clients)` with an explicit `edge_index → client` map, asserted against `edge_cfgs` length — `983644ae`
- [x] **D-15 / vehicle+RSU binding:** audit the actor-discovery loop in `openscenario_multi_edge_right_merge.py` — ego resolved by `role_name='hero'` (correct); NPC by velocity filter with multi-vehicle uniqueness check + warning (sufficient for single-NPC scenario); `other_vehicles` list is empty (no list-position binding). — `983644ae`

### Step 1 — Phase 1.5: make warm import real (sequential, proto round-trip)

Everything here is in-process. Fast iteration, no container rebuild.

Even in sequential mode the handoff should go through the actual proto
serialization contract — `MigrationPayload.serialize()` → bytes →
`MigrationPayload.deserialize()` → inject — rather than calling
`import_tracked_obstacle_state` directly on the in-memory object. The bytes
never cross a socket, but the round-trip:

1. validates that `KFState` and `TrackLatent` survive pickle intact
2. locks in the byte count that `InterLocaleLink.model_transfer` reports (which
   must stay `payload_bytes()`, never `len(pickle)` — see D-14)
3. is the *same* call path that Step 3's container-side RPCs will use; any
   discrepancy between in-process and wire results is a serialization bug, not a
   network bug

Wire the serialization in `SequentialMigrationDaemon.request_handoff` and
`transfer_obstacle_state`: `payload = MigrationPayload.deserialize(payload.serialize())`
before the import call. One line per method. This is not a no-op; it is the
protocol validation step.

- [x] Add `handoff_warm_import` to the edge YAML schema and read it in `__init__` — `PredictionLateFusionEdge`, `WorldFusionEdge`, `_PluggableEdgeBase` all read from cfg; `getattr` in mixin finds it — `983644ae`
- [x] **F8:** reconcile gate semantics between `AB3DMOTStateTransferMixin` and `_PluggableEdgeBase` — `_PluggableEdgeBase` now has `_warm_import_enabled()` and gates `_import_track_latent`; `PredictionLateFusionEdge.import_vehicle_state` (class override) also gated — `983644ae`
- [ ] Import-side reconciliation: on the first detection that associates to an injected track, merge rather than spawn a duplicate. **Analysis:** `max_age=6` in late fusion (6 AB3DMOT steps = 24 world ticks). Handoff at tick ~161; RSU1 detects NPC at tick ~243. Warm track is pruned ~tick 185 — 58 ticks before RSU1 detection. No live warm track at re-detection, so no ghost-duplicate risk at the current operating point. If `max_age` is increased significantly, reconciliation logic must be added. Confirm with run.
- [x] Add the serialize → deserialize round-trip to both daemon methods — `983644ae` (confirmed: `payload_bytes()=1050`, `len(pickle)=1696`)
- [x] Instrument `first_track_publish_tick[carla_id]` on the edge manager — `_ab3d_history_to_trajs(tick=tick)` added; `[TRACK_PUBLISH]` log on first per-cid appearance — `983644ae`
- [x] Replace the geometric-proxy window in the scenario's closing log — proxy now labelled `PROXY`; `MEASURED` line added from `edge1._first_track_publish_tick` — `983644ae`
- [ ] Negative control harness: run warm-import on vs off, ≥3 runs each, same seed
- [ ] **Gate:** report the measured warm-vs-cold delta. If ~0, stop and re-plan — Counter-H1a holds and distribution work is premature.
- [ ] Check for duplicate-ghost regressions: `ghost_brake_events`, ego merge tick, collisions
- [ ] Commit immediately on a validated run, before touching anything else

### Step 2 — Transport abstraction (no behavior change)

- [ ] `migration/transport.py`: `StateTransport` protocol + `InProcessTransport`
- [ ] `SequentialMigrationDaemon` takes a `transport`; route `export_vehicle_state` / `export_tracked_obstacle_state` / both imports through it
- [ ] `request_handoff`: make the store lookup conditional (`use_store`); in `-eo` the store is unpopulated and the export fallback is the primary path — downgrade the "store has no snapshot" warning accordingly
- [ ] Re-run Step 1's validated Scenario B. Byte-identical results, or the refactor is wrong.
- [ ] Drop the per-tick snapshot upload loop under `-eo` (it would be an RPC per vehicle per tick for a store nothing reads)

### Step 3 — Container-side export/import

- [ ] Add the four messages + `Edge_ExportTrackState` / `Edge_ImportTrackState` to `ecloud.proto`; `python ecav.py --build`
- [ ] `EdgeServer.Edge_ExportTrackState`: call `edge_manager.export_tracked_obstacle_state(carla_id, position=...)`, return `found` + `serialize()` + `payload_bytes()`
- [ ] `EdgeServer.Edge_ImportTrackState`: `deserialize()` → `import_tracked_obstacle_state(carla_id, payload)`
- [ ] Handle "no edge_manager yet" and "no KF found" as `found=False`, never an exception
- [ ] Extend `EdgeFusionClient` with `export_track_state()` / `import_track_state()`
- [ ] **Two-process gRPC smoke test, no CARLA** (mirrors `migration/smoke_test.py`): synthetic tracks, real containers, export → wire → import, assert `state_vector` and `covariance` round-trip byte-identical and `model_bytes` matches on both sides

### Step 4 — Hub transport, Scenario B under `-eo`

- [ ] `GrpcHubTransport`; scenario constructs it when `opt.edge_only`
- [ ] Re-enable the handoff block under `-eo`
- [ ] Verify the tick-ordering contract holds: export RPC lands before the source container's next `Edge_PerformFusion`
- [ ] **H2 parity run:** sequential vs `-eo`, same seed. `payload_bytes` exact; handoff tick, first-warm-track tick, window, merge tick within ±2.
- [ ] Commit on a validated run

### Step 5 — Peer transport

- [ ] `Edge_MigrateTrack` message + RPC
- [ ] `EdgeServer.Edge_MigrateTrack`: export locally, dial `dst_edge_ip:dst_edge_port`, call `Edge_ImportTrackState`, return the `TrackStateResponse` for base-side cost modeling
- [ ] Container-side peer channel: lazy-connect + cache per destination
- [ ] `GrpcPeerTransport`; select via YAML (`handoff_transport: hub | peer`)
- [ ] Re-run the H2 parity check against the hub numbers — must be identical, since the cost is modeled either way
- [ ] Confirm the orchestrator is off the state path: base process sees only `carla_id` + `TransferCost`, never payload bytes

### Step 6 — Cost gating + sensitivity sweep

- [ ] `handoff_cost_gates_apply` in the edge YAML, default `false`
- [ ] `_pending_imports` queue in the daemon; `apply_after_ticks = ceil(total_ms / (world_dt * 1000))`; drained at the top of each tick
- [ ] Confirm gating off reproduces Step 5 exactly
- [ ] **H3 sweep:** scale `serialize_rate_ms_per_byte` and the latency model; plot realized window vs `total_ms`; find the collapse point
- [ ] Record the sweep in the eval output, not just the log

### Step 7 — Metrics and documentation

- [ ] Extend the `HAND-OFF MIGRATION` eval block with: transport mode, gating on/off, measured warm-vs-cold delta, realized window
- [ ] Remove the `[EGO-DBG]` / `[SCENB-DBG]` WARNING-level diagnostics from the scenario file (already on the Tyler-sync list)
- [ ] `print` → `logger` driveby on every file touched
- [ ] Update `docs/kb/wiki/current_state.md` after each validated step, not at the end
- [ ] Correct the run-12 entry in `current_state.md` to distinguish exported-payload from applied-state
- [ ] Session log `docs/kb/raw/sessions/YYYY-MM-DD.md`
- [ ] Note the container rebuild requirement whenever `ecav/` changes precede a Docker test

---

## Decisions

Continuing the Phase 1 table (D-1 … D-7).

| ID | Question | Decision |
|----|----------|----------|
| D-8 | Where does the payload travel in `-eo`? | **Hub first, then peer.** Step 4 routes through the base process (reuses existing base→edge channels, zero new connectivity); Step 5 adds `Edge_MigrateTrack` so the source dials the destination directly. Both share the same export/import handlers, so the peer step is a thin dialer. Peer is the claim the paper wants; hub is the validation ladder. |
| D-9 | Does modeled cost gate application? (reopens D-7) | **Yes, config-flagged, default off.** Default off so Phase 2 reproduces Phase 1 exactly; the sweep with it on is the H3 result. |
| D-10 | Scope | **Scenario B only.** Scenario A is a strict subset and lifts for free later. |
| D-11 | Snapshot cadence | **On-demand at trigger.** Per-tick snapshotting under `-eo` would be an RPC per vehicle per tick feeding a store nothing reads. On-demand also matches what a real deployment does. |
| D-12 | One RPC pair or separate vehicle/obstacle RPCs? | **One pair**, keyed on `carla_id` + optional position hint. The container sees only tracks; ownership is a base-process concept (F4). |
| D-13 | Where does the delay live when gating is on? | **Base process.** The container has no sim clock. The daemon queues imports and releases them `apply_after_ticks` later. |
| D-14 | Which byte count feeds the cost model? | **`MigrationPayload.payload_bytes()`**, never the pickle wire size. Parity with Phase 1 depends on it. |
| D-15 | Process ID determinism | All actor process IDs must be deterministic regardless of spawn/registration order (F5). Edges: bind from `container_name`, not arrival order; fail loudly on collision. Vehicles and RSUs: bind by CARLA role/attribute at discovery, not by list position. |

---

## Risks

| Risk | Severity | Mitigation |
|---|---|---|
| H1 falsified — warm import buys ~0 ticks | **High.** Invalidates the framing, not just the plan. | Step 1 gates everything downstream. Measure before building. |
| Two-edge `-eo` is new territory (F7) | **Medium** for schedule; single-edge is validated, the combination is not | Step 0 starts on a single-edge scenario before adding the second edge |
| Duplicate-ghost regression from warm import | Medium | Reconciliation is the explicit content of Step 1; `ghost_brake_events` is the existing detector |
| One-tick lag shifts all `-eo` timing (Counter-H2) | Medium | Characterize in Step 0, before it can be confused with a handoff bug |
| Wrong-locale edge binding (F5) | Medium — presents as a geometry bug | Explicit `container_name` binding, asserted map |
| Container rebuild in the iteration loop | Medium | Steps 1–2 are entirely sequential/in-process by design |
| Base/container beacon-id divergence (F6) | Low — contract is `carla_id`-keyed | Confirm in Step 0 rather than assume |

---

## Out of scope

- Actor distribution (`-d`) — Phase 3
- Mamba / full-latent migration under `-eo` — the transport is backend-agnostic, but validation is separate work
- Scenario A lift
- Piecemeal fusion mode (`Edge_SendIntermediateFeatures` + `Edge_GetFusionResult`) — batch is sufficient while the base tick loop is serial
- Replacing the C++ orchestrator for `-d` mode

---

## Related

- [Phase 1 plan](edge_handoff_phase1_state_transfer.md) — Steps 0–7, D-1 … D-7
- [Edge-only distributed mode](edge_only_distributed_mode.md) — `-eo` infrastructure
- [Multi-edge locale handoff](multi_edge_locale_handoff.md) — Model A/B/C framing
- [Scale-out evaluation](scale_out_evaluation.md)
- [Current state](../kb/wiki/current_state.md)
