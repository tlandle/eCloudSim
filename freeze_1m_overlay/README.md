# freeze-1m overlay (settled non-scenario code)

Version-controlled snapshot of the settled freeze-1m code fixes so the set is
reproducible from the repo rather than living only in a scratchpad on one host.
These files OVERLAY the frozen base (commit `71c9f37e`, freeze-1h) at runtime:
the campaign/probe scripts `cp` each overlay onto its base path, run, then
`git checkout` the base back (trap on EXIT). The frozen base is not modified.

## Overlay map (overlay file -> base path it replaces)

| overlay file | base path |
|---|---|
| `edge_manager_merged_1l_epoch.py` | `ecav/core/application/edge/edge_manager/edge_manager_worldfusion_ab3dmot_linear_predictor.py` |
| `factories_1l.py` | `ecav/core/application/edge/migration/factories.py` |
| `mtr_edge_predictor_1m.py` | `ecav/core/prediction/mtr_edge_predictor.py` |
| `behavior_agent_1m.py` | `ecav/core/plan/behavior_agent.py` |
| `pluggable_base_wired.py` | `ecav/core/application/edge/edge_manager/edge_manager_pluggable_base.py` |
| `runner_wired.py` | `ecav/scenario_testing/openscenario_1_flow_gt.py` |
| `wrapper_wired.py` | `ecav/core/tracking/mamba3dmot/wrapper.py` |
| `tracker_wired.py` | `ecav/core/tracking/mamba3dmot/tracker.py` |
| `tracklet_1l.py` | `ecav/core/tracking/mamba3dmot/tracklet.py` |
| `payload_wired.py` | `ecav/core/application/edge/migration/payload.py` |
| `daemon_1l.py` | `ecav/core/application/edge/migration/daemon.py` |
| `scenario_1_1l.py` | `ecav/scenario_testing/scenarios/scenario_1.py` |

## Settled fixes captured here

- Defect B (migrated forecast invisible during the destination-blind window):
  coast-through-blind-window rebuild of the imported track's trajectory from the
  memo_bank translated to the coasted pose, sourced from tracked+lost, recreate
  if pruned (`edge_manager_merged_1l_epoch.py`).
- Ghost-filter fix: `_filter_ghost_tracks` falls back to `obstacle.kf_speed_mps`
  for Mamba tracks (AB3DMOT `.trackers` absent) so moving migrated tracks are not
  deleted as static.
- Acceleration-consumption fix: the gate uses the transferred `_migrated_accel_mps2`
  directly while the destination has `< LOCAL_ACCEL_THRESH` of its own local
  frames, then hands over to the local estimate; `[GATE_ACCEL]` logs used/src/
  migrated/local_est and the handover (`edge_manager_merged_1l_epoch.py`,
  `factories_1l.py`).
- Locale tag on `[MODESROW]`/`[MIG_SEAM]`, identity-settling `[MIG_IDENT]`.
- Round-trip relay transport (loud `[XFER_FALLBACK]` guard, `[XFERROW]`) in
  `daemon_1l.py`.

`scenario_1_1l.py` carries the paired spawn-phase seed (settled; SEEDROW,
deterministic per-index oncoming spawn-x offset, paired across arms) and an
endogenous acceleration-onset branch (ONCOMING_CRUISE + ONCOMING_ACCEL_ONSET).
The onset branch is inert unless ONCOMING_ACCEL_ONSET is set, so the file is safe
for the headline / record-ablation / corridor blocks that do not set it.

## Panel runner (parked)

`atlas_maneuver_panel.sh` is the maneuvering success-rate / FDE panel runner
(across-seed situational spread plus a same-seed repetition cell for run-to-run
jitter; log names match ACC_TAG_RE so `scripts/khonsu_1l_land.py acc` lands it).
It is versioned here so it is not lost, but it is NOT wired to a finalized
geometry. The closed-loop maneuvering collision panel was found not to be
load-bearing for the record-depth claim: that claim rests on the closed-loop
record ablation (the large separation is carrying velocity at all) and the
offline forced-handoff depth benchmark, not on a closed-loop collision. Whether
the projected-acceleration panel stays in the paper is Tyler's call.
