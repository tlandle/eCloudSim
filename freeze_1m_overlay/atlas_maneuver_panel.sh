#!/bin/bash
set +u
# freeze-1m MANEUVERING PANEL (endogenous-onset scenario). Runs the success-rate /
# FDE panel at a CONFIRMED operating point (from atlas_maneuver_v2_probe.sh), over
# paired spawn-phase seeds, PLUS a same-seed repetition cell so the peer can
# decompose run-to-run jitter (fixed seed, repeated) from situational spread
# (across seeds). Do NOT launch until the probe confirms kf=GO / warm=WAIT with a
# slow oncoming at the decision and time_avail ~ +1.5 s. Geometry is passed by env
# so this file is unchanged if the conflict/onset is retuned.
#
#   ONSET=<x>  CRUISE=5 ACCEL=2 VCAP=16 SHIFT=-10 CONFLICT_X=278 \
#   SEEDS="0 1 2 3 4" REPS=5 bash atlas_maneuver_panel.sh
#
# Log names match ACC_TAG_RE (acc_<arm>_a<accel>_s<seed>) so
#   scripts/khonsu_1l_land.py acc --logdir <dir>
# lands the panel directly (collided from eps>0 = binary collision rule; FDE from
# the fde lander). The rep cell lands from its own subdir (its "seed" column is the
# repeat index; KHONSU_SEED is held at REP_SEED for every repeat).
MAIN=/home/atlas/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox
WT=$MAIN/idfix_wt
SP=/tmp/claude-1000/-home-atlas-TrafficSimulator-eCloud-ecloudsim-distributed-sandbox/88f56d4f-259d-44e9-bdb2-c8ace70b6018/scratchpad
CARLA_ROOT=/home/atlas/carla-0.9.15
CRUISE=${CRUISE:-5}; ACCEL=${ACCEL:-2}; VCAP=${VCAP:-16}; SHIFT=${SHIFT:--10}
CONFLICT_X=${CONFLICT_X:-278}
ONSET=${ONSET:?set ONSET to the confirmed operating point from the probe}
SEEDS=${SEEDS:-"0 1 2 3 4"}
REPS=${REPS:-5}; REP_SEED=${REP_SEED:-0}
ARMS=${ARMS:-"kf warm"}
exec 9>/tmp/atlas_1l_central.lock
flock -w 7200 9 || { echo "[panel] lock wait timed out"; exit 1; }
echo "[panel] lock acquired $(date +%H:%M:%S) onset=${ONSET} seeds='${SEEDS}' reps=${REPS}"
cd "$WT" || exit 1
pkill -9 -f "CarlaUE4" 2>/dev/null; sleep 3
source /home/atlas/anaconda3/etc/profile.d/conda.sh && conda activate opencda310
export PYTHONPATH=$CARLA_ROOT/PythonAPI/carla:${PYTHONPATH:-}
[ "$(git rev-parse --short HEAD | cut -c1-8)" = "71c9f37e" ] || { echo "ABORT not freeze-1h"; exit 1; }
COMMON=(
  "ecav/core/application/edge/edge_manager/edge_manager_pluggable_base.py|pluggable_base_wired.py"
  "ecav/scenario_testing/openscenario_1_flow_gt.py|runner_wired.py"
  "ecav/core/tracking/mamba3dmot/wrapper.py|wrapper_wired.py"
  "ecav/core/tracking/mamba3dmot/tracker.py|tracker_wired.py"
  "ecav/core/tracking/mamba3dmot/tracklet.py|tracklet_1l.py"
  "ecav/core/application/edge/migration/factories.py|factories_1l.py"
  "ecav/core/application/edge/migration/payload.py|payload_wired.py"
  "ecav/scenario_testing/scenarios/scenario_1.py|scenario_1_1l.py"
  "ecav/core/plan/behavior_agent.py|behavior_agent_1m.py"
  "ecav/core/application/edge/edge_manager/edge_manager_worldfusion_ab3dmot_linear_predictor.py|edge_manager_merged_1l_epoch.py"
  "ecav/core/prediction/mtr_edge_predictor.py|mtr_edge_predictor_1m.py"
)
DESTS=()
_backup () { local i=0; for p in "${COMMON[@]}"; do d=${p%%|*}; cp "$d" "/tmp/pnl_${i}.bak"; DESTS+=("$d"); i=$((i+1)); done; }
restore () { local i=0; for p in "${COMMON[@]}"; do d=${p%%|*}; cp "/tmp/pnl_${i}.bak" "$d"; i=$((i+1)); done
  git checkout -q "${DESTS[@]}" 2>/dev/null; pkill -9 -f "CarlaUE4/Binaries" 2>/dev/null; }
trap restore EXIT
_backup
for p in "${COMMON[@]}"; do d=${p%%|*}; s=${p##*|}; cp "$SP/$s" "$d"; done
grep -q "ONCOMING_ACCEL_ONSET" ecav/scenario_testing/scenarios/scenario_1.py || { echo "ABORT endogenous onset not applied"; exit 1; }
R="$WT/evaluation_outputs/frozen_1m_panel"; mkdir -p "$R" "$R/rep"
carla_restart () { pkill -9 -f "CarlaUE4/Binaries" 2>/dev/null; sleep 5; ( cd "$CARLA_ROOT" && setsid nohup ./CarlaUE4.sh -RenderOffScreen >/dev/null 2>&1 9>&- & ); sleep 55; }
run () { local log="$1"; shift; carla_restart; echo "[$(date +%H:%M:%S)] $(basename $log) : $*"
  echo "[LAUNCHENV] $*" > "$log"
  ( eval "$* EVAL_TAG=khonsu-eval-freeze-1m timeout -k 30 900 python ecav.py -t openscenario_1_accel_gt --apply_ml" >> "$log" 2>&1 ) || true; }
COM="ONCOMING_SPEED=${CRUISE} ONCOMING_CRUISE=${CRUISE} ONCOMING_ACCEL=${ACCEL} ONCOMING_ACCEL_ONSET=${ONSET} ONCOMING_VCAP=${VCAP} ONCOMING_SHIFT_X=${SHIFT} LOOKAHEAD_S=1"
# --- across-seed panel: situational spread ---
for md in $ARMS; do for sd in $SEEDS; do
  run "$R/acc_${md}_a${ACCEL}_s${sd}.log" MIGRATION_MODE=${md} KHONSU_SEED=${sd} $COM
done; done
# --- same-seed repetition cell: run-to-run jitter (seed held at REP_SEED) ---
for md in $ARMS; do r=0; while [ $r -lt $REPS ]; do
  run "$R/rep/acc_${md}_a${ACCEL}_s${r}.log" MIGRATION_MODE=${md} KHONSU_SEED=${REP_SEED} $COM
  r=$((r+1))
done; done
# --- land both with the acc lander (binary collision, per-arm FDE) ---
python "$MAIN/scripts/khonsu_1l_land.py" acc --logdir "$R"     --tag panel     --machine atlas --out "$R/_panel.csv"     2>>"$R/_land.err" || true
python "$MAIN/scripts/khonsu_1l_land.py" acc --logdir "$R/rep" --tag panel_rep --machine atlas --out "$R/_panel_rep.csv" 2>>"$R/_land.err" || true
python "$MAIN/scripts/khonsu_1l_land.py" fde --logdir "$R"     --tag panel     --machine atlas --out "$R/_fde.csv"       2>>"$R/_land.err" || true
python "$MAIN/scripts/khonsu_1l_land.py" fde --logdir "$R/rep" --tag panel_rep --machine atlas --out "$R/_fde_rep.csv"   2>>"$R/_land.err" || true
{ echo "===== MANEUVER PANEL ($(date +%H:%M:%S)) onset=${ONSET} cruise=${CRUISE} a=${ACCEL} cap=${VCAP} shift=${SHIFT} ====="
  echo "-- across-seed (situational spread): _panel.csv / _fde.csv"
  echo "-- same-seed x${REPS} (run-to-run jitter, seed=${REP_SEED}): _panel_rep.csv / _fde_rep.csv"
  echo "collision rule: binary collided/completed per episode (eps>0). FDE named, min of two top modes."; } | tee "$R/_report.txt"
echo "[panel] DONE $(date +%H:%M:%S)" > "$R/_panel_done"
pkill -9 -f "CarlaUE4/Binaries" 2>/dev/null
echo "[panel] ALL DONE $(date +%H:%M:%S)"
