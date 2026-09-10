#!/bin/bash
set +u
# CETUS corridor RUN-ONLY. The worktree ~/cetus_1m_wt is already set up (frozen
# base 71c9f37e + freeze_1m_overlay applied + corridor files + proto stubs + REAL
# models placed by the peer). This script does NOT recreate the worktree and does
# NOT touch the models (doing so would clobber the peer's real checkpoints with
# dangling symlinks). It only verifies the setup then runs the 70 routes.
WT=$HOME/cetus_1m_wt
CARLA_ROOT=$HOME/carla-0.9.15
exec 9>/tmp/cetus_1m.lock
flock -w 600 9 || { echo "[cetus-corridor] lock busy"; exit 1; }
echo "[cetus-corridor-run] start $(date +%H:%M:%S)"
cd "$WT" || { echo "ABORT no worktree"; exit 1; }
source $HOME/anaconda3/etc/profile.d/conda.sh && conda activate opencda310
export PYTHONPATH=/tmp/torch_compat_1m:$CARLA_ROOT/PythonAPI/carla:${PYTHONPATH:-}
[ "$(git rev-parse --short HEAD)" = "71c9f37e" ] || { echo "ABORT worktree not at base"; exit 1; }
# model-integrity guard (peer 57af285a): full size+hash check against the manifest,
# not just a size floor - catches the silent-divergence case (untracked Swin backbone
# quietly fetched from HuggingFace at a different revision than atlas).
git fetch origin develop -q 2>/dev/null || true
git checkout origin/develop -- scripts/verify_models.sh scripts/models.manifest 2>/dev/null || true
bash scripts/verify_models.sh "$(pwd)" || { echo "ABORT: model artefacts differ from manifest; results from this host are not comparable"; exit 1; }
grep -q "PUBGATE_SRC" ecav/core/application/edge/edge_manager/edge_manager_worldfusion_ab3dmot_linear_predictor.py || { echo "ABORT epoch edge missing"; exit 1; }
grep -q "CONSUMEDEPOCH" ecav/core/plan/behavior_agent.py || { echo "ABORT epoch behavior missing"; exit 1; }
grep -q "CORRIDORCROSS" ecav/scenario_testing/openscenario_1_corridor_gt.py || { echo "ABORT corridor runner missing"; exit 1; }
pkill -9 -f "CarlaUE4/Binaries" 2>/dev/null; sleep 3
R="$WT/evaluation_outputs/frozen_1m_corridor"; mkdir -p "$R"
carla_restart () { pkill -9 -f "CarlaUE4/Binaries" 2>/dev/null; sleep 6; ( cd "$CARLA_ROOT" && setsid nohup ./CarlaUE4.sh -RenderOffScreen >/dev/null 2>&1 9>&- & ); sleep 55; }
_degenerate () { local lg="$1"
  grep -q "is not found in your CARLA repo\|has no attribute 'world'" "$lg" 2>/dev/null && return 0
  grep -q "\[EGO-DBG\]" "$lg" 2>/dev/null || return 0
  return 1; }
run () { local tag="$1"; shift; local log="$R/${tag}.log"
  if [ -f "$log" ] && grep -q RUNROW "$log" 2>/dev/null && ! _degenerate "$log"; then echo "[skip] $tag"; return; fi
  local attempt
  for attempt in 1 2; do
    carla_restart; echo "[$(date +%H:%M:%S)] $tag attempt $attempt : $@"
    echo "[LAUNCHENV] ONCOMING_SPEED=12 TRIGGER_DIST=300 $@" > "$log"
    ( env "$@" ONCOMING_SPEED=12 TRIGGER_DIST=300 EVAL_TAG=khonsu-eval-freeze-1m timeout -k 30 900 python ecav.py -t openscenario_1_corridor_gt --apply_ml >> "$log" 2>&1 ) || true
    _degenerate "$log" || break
    echo "[retry] $tag degenerate"
  done; }
SEEDS=${SEEDS:-"1 2 3 4 5"}
if [ "${SMOKE:-0}" = "1" ]; then run "co_warm_n1_r1" MIGRATION_MODE=warm EPOCH_FENCE=1; echo "smoke $(date +%H:%M:%S)"; pkill -9 -f "CarlaUE4/Binaries" 2>/dev/null; exit 0; fi
for rep in $SEEDS; do
  for arm in cold reactive warm replication repl_final kf_final; do
    run "co_${arm}_n1_r${rep}" MIGRATION_MODE=$arm EPOCH_FENCE=1
  done
done
for rep in $SEEDS; do run "co_khonsu_ef0_r${rep}" MIGRATION_MODE=warm EPOCH_FENCE=0; done
pkill -9 -f "CarlaUE4/Binaries" 2>/dev/null
echo "[cetus-corridor-run] DONE $(date +%H:%M:%S)" > "$R/_corridor_1m_done"
echo "[cetus-corridor-run] ALL DONE $(date +%H:%M:%S)"
