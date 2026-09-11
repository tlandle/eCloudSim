#!/bin/bash
set +u
# Byte-field inert proof gate (peer condition on the HANDOFFROW bytes= addition).
# Runs on cetus AFTER capacity finishes and BEFORE flow-arms. Same host, same seed,
# one cell before (currently-applied runner, no bytes field) and one after (the
# staged byte-field runner). If the outcome fields (episodes/collided/completed)
# are identical the field is inert: apply the new runner and write _proof_ok, which
# releases the flow-arms gate (it waits on WAIT_FILE=this marker). If they differ,
# restore the old runner, write _proof_fail (flow-arms stays blocked), and print
# both outcomes for the human to read. Proof is behavior-inert by construction (a
# dict store of an already-computed value + an emit-time log field); this verifies
# it rather than asserting it.
WT=$HOME/cetus_1m_wt; CARLA_ROOT=$HOME/carla-0.9.15
BASE="$WT/ecav/scenario_testing/openscenario_1_flow_gt.py"
NEW=/tmp/runner_wired_bytes.py
CAP_DONE="$WT/evaluation_outputs/frozen_1n_capacity/_done"
R="$WT/evaluation_outputs/frozen_1n_proof"; mkdir -p "$R"
cd "$WT" || exit 1
source $HOME/anaconda3/etc/profile.d/conda.sh && conda activate opencda310
export PYTHONPATH=/tmp/torch_compat_1m:$CARLA_ROOT/PythonAPI/carla:${PYTHONPATH:-}
[ -f "$NEW" ] || { echo "[proof] ABORT staged runner $NEW missing"; exit 1; }

echo "[proof] waiting for capacity _done ($(date +%H:%M:%S))"
for i in $(seq 1 17280); do [ -f "$CAP_DONE" ] && break; sleep 10; done
[ -f "$CAP_DONE" ] || { echo "[proof] ABORT capacity never finished"; exit 1; }
for i in $(seq 1 60); do pgrep -f "ecav.py -t openscenario" >/dev/null 2>&1 || break; sleep 10; done

carla_restart () { pkill -9 -f "CarlaUE4/Binaries" 2>/dev/null; sleep 6; ( cd "$CARLA_ROOT" && setsid nohup ./CarlaUE4.sh -RenderOffScreen >/dev/null 2>&1 9>&- & ); sleep 55; }
_outcome () { grep -oE "episodes=[0-9]+ contact_ticks=[0-9]+" "$1" 2>/dev/null | head -1; }
_run () { local log="$1"
  echo "[LAUNCHENV] MIGRATION_MODE=warm TRIGGER_MODE=predictive LOOKAHEAD_S=1 ONCOMING_SPEED=12 TRIGGER_DIST=300 KHONSU_SEED=1" > "$log"
  ( env MIGRATION_MODE=warm TRIGGER_MODE=predictive LOOKAHEAD_S=1 ONCOMING_SPEED=12 \
      TRIGGER_DIST=300 KHONSU_SEED=1 EVAL_TAG=khonsu-eval-freeze-1n \
      timeout -k 30 900 python ecav.py -t openscenario_1_flow_gt --apply_ml >> "$log" 2>&1 ) || true; }

# BEFORE: current applied runner (no bytes field)
grep -q "npc_handoff_bytes" "$BASE" && { echo "[proof] ABORT base already has the byte field; nothing to compare"; exit 1; }
carla_restart; echo "[proof] before-cell (current runner) $(date +%H:%M:%S)"; _run "$R/proof_before.log"
grep -q RUNROW "$R/proof_before.log" || { echo "[proof] ABORT before-cell no RUNROW"; exit 2; }
B=$(_outcome "$R/proof_before.log")

# swap in the new runner, AFTER cell
cp "$BASE" "$R/_flow_gt.bak"
cp "$NEW" "$BASE"
carla_restart; echo "[proof] after-cell (byte-field runner) $(date +%H:%M:%S)"; _run "$R/proof_after.log"
grep -q RUNROW "$R/proof_after.log" || { cp "$R/_flow_gt.bak" "$BASE"; echo "[proof] ABORT after-cell no RUNROW; reverted"; exit 2; }
A=$(_outcome "$R/proof_after.log")

echo "[proof] BEFORE: $B"
echo "[proof] AFTER : $A"
if [ "$B" = "$A" ]; then
  echo "[proof] PASS: outcome identical, byte field inert. New runner APPLIED."
  grep -m1 "HANDOFFROW" "$R/proof_after.log" | sed "s/\x1b\[[0-9;]*m//g" | grep -oE "HANDOFFROW.*bytes=[0-9]+" | head -1
  echo "PASS before=$B after=$A $(date +%H:%M:%S)" > "$R/_proof_ok"
else
  cp "$R/_flow_gt.bak" "$BASE"
  echo "[proof] FAIL: outcome DIFFERS (before=$B after=$A). Reverted runner; flow-arms stays blocked."
  echo "FAIL before=$B after=$A $(date +%H:%M:%S)" > "$R/_proof_fail"
fi
pkill -9 -f "CarlaUE4/Binaries" 2>/dev/null
echo "[proof] done $(date +%H:%M:%S)"
