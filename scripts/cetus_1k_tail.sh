#!/bin/bash
set +u
# Cetus 1k TAIL: blocks on cetus_1k.lock behind seeds 6-10, then runs, IN ORDER:
#   (1) multi-crossing (Table 8, section 5.8): burst 5-veh platoon + density
#       FLOW_N={2,4,8}, arms {warm,edgewarp,cold}, TEN seeds -> frozen1k_cetus_t8_rows.csv
#   (2) visible matrix: 6 arms x5 -> frozen1k_cetus_53_rows.csv
#   (3) netem: lat/jit/loss grid, warm grpc -> frozen1k_cetus_netem_rows.csv
# Tags are q5_n{fn}_{m}_r{rep} / burst_{m}_r{rep}: make_floats.py:133 reads
# tag.startswith('q5_n') and khonsu_multicross_diag.py regex is ^(q5_|burst_).
# NO re-checkout (seeds already set the tree to freeze-1k with LFS models restored).
exec 9>/tmp/cetus_1k.lock
echo "[1k tail] waiting for cetus_1k.lock (seeds 6-10 to finish)..."
flock -w 72000 9 || { echo "[1k tail] lock wait timed out (20h)"; exit 1; }
echo "[1k tail] lock acquired $(date +%H:%M:%S)"
cd $HOME/ecloudsim_distributed_sandbox || exit 1
export PYTHONPATH=/tmp/torch_compat:$HOME/carla-0.9.15/PythonAPI/carla:${PYTHONPATH:-}
source $HOME/anaconda3/etc/profile.d/conda.sh && conda activate opencda310
# Verify the freeze-1k tree the batch/seeds set up; do NOT re-checkout (would clobber LFS models).
echo "HEAD=$(git rev-parse --short HEAD)"
grep -q "memo_tick" ecav/core/application/edge/migration/payload.py || { echo "ABORT no memo_tick"; exit 1; }
grep -q "OT RECHECK" ecav/core/plan/behavior_agent.py || { echo "ABORT no recheck"; exit 1; }
grep -q "weights_only=False" ecav/core/prediction/mtr_edge_predictor.py || { echo "ABORT mtr load not patched"; exit 1; }
_mtrsz=$(stat -c%s ecav/ml_manager/models/mtr_wf_mamba/best_model.pth 2>/dev/null)
[ "${_mtrsz:-0}" -gt 1000000 ] || { echo "ABORT mtr checkpoint a pointer (${_mtrsz:-0})"; exit 1; }
_mbsz=$(stat -c%s ecav/core/tracking/mamba3dmot/mamba3dmot_weights.pth 2>/dev/null)
[ "${_mbsz:-0}" -gt 1000000 ] || { echo "ABORT mamba weights a pointer (${_mbsz:-0})"; exit 1; }
python -c "import torch,sys; sys.exit(0 if torch.load.__module__!='torch.serialization' else 1)" && echo "torch.load shim active" || echo "WARN shim inactive"
KB=$HOME/ecloudsim_distributed_sandbox/docs/kb/data/relay_eval_2026_08
pkill -9 -f "CarlaUE4/Binaries" 2>/dev/null; sleep 3
carla_restart () { pkill -9 -f "CarlaUE4/Binaries" 2>/dev/null; sleep 6; ( cd $HOME/carla-0.9.15 && setsid nohup ./CarlaUE4.sh -RenderOffScreen >/dev/null 2>&1 9>&- & ); sleep 55; }
_degenerate () { local lg="$1"
  grep -q "is not found in your CARLA repo\|has no attribute 'world'" "$lg" 2>/dev/null && return 0
  grep -q "\[GT INJECT DBG\]" "$lg" 2>/dev/null || return 0
  grep -q "\[EGO-DBG\]" "$lg" 2>/dev/null || return 0
  return 1; }
run () { local tag="$1"; local scn="$2"; local dir="$3"; shift 3; mkdir -p "$dir"; local log="$dir/${tag}.log"
  if [ -f "$log" ] && grep -q RUNROW "$log" 2>/dev/null && ! _degenerate "$log"; then echo "[skip] $tag (done)"; return; fi
  local attempt
  for attempt in 1 2; do
    carla_restart; echo "[$(date +%H:%M:%S)] $tag ($scn) attempt $attempt"
    echo "[LAUNCHENV] defaults(ONCOMING_SPEED=12 TRIGGER_DIST=300) then $@" > "$log"
    ( env ONCOMING_SPEED=12 TRIGGER_DIST=300 "$@" EVAL_TAG=khonsu-eval-freeze-1k timeout -k 30 900 python ecav.py -t "$scn" --apply_ml >> "$log" 2>&1 ) || true
    _degenerate "$log" || break
    echo "[retry] $tag degenerate (startup transient)"
  done
}
land () { local dir="$1"; local out="$2"; python scripts/khonsu_design_extract.py "$dir" -o "$KB/$out" --machine cetus --oncoming-speed 12 --trigger-dist 300 --tag khonsu-eval-freeze-1k 2>&1 | tail -1 || true; }

# ---- (1) multi-crossing / Table 8: burst + density, 3 arms, 10 seeds ----
# EdgeWarp arm carries the FULL record (edgewarp_full + predictive trigger), the
# same as the headline/trigger blocks (reviewer: no baseline weakening between
# sections). Tag token stays "edgewarp" so make_floats.py:133 / the multicross
# diag read it; RUNROW mode=edgewarp_full is relabeled EdgeWarp downstream.
_mode_args () { case "$1" in
  edgewarp) echo "MIGRATION_MODE=edgewarp_full TRIGGER_MODE=predictive LOOKAHEAD_S=1";;
  *) echo "MIGRATION_MODE=$1";;
esac; }
T8=$HOME/ecloudsim_distributed_sandbox/evaluation_outputs/frozen1k_t8; mkdir -p $T8
for rep in $(seq 1 10); do for m in warm edgewarp cold; do
  run "burst_${m}_r${rep}" openscenario_1_burst_gt "$T8" $(_mode_args $m)
done; done
for rep in $(seq 1 10); do for fn in 2 4 8; do for m in warm edgewarp cold; do
  run "q5_n${fn}_${m}_r${rep}" openscenario_1_flow_gt "$T8" $(_mode_args $m) FLOW_N=$fn
done; done; done
echo "[Table8] DONE $(date +%H:%M:%S)" > $T8/_done; land "$T8" frozen1k_cetus_t8_rows.csv

# ---- (2) visible matrix: 6 arms x5 (visible scenario is in the 1k tag) ----
M=$HOME/ecloudsim_distributed_sandbox/evaluation_outputs/frozen1k_53_visible; mkdir -p $M
for s in 1 2 3 4 5; do for m in warm kf cold reactive edgewarp handover_snapshot; do
  run "viscon_${m}_s${s}" openscenario_1_flow_visible_gt "$M" MIGRATION_MODE=$m
done; done
echo "[5.3 visible] DONE $(date +%H:%M:%S)" > $M/_done; land "$M" frozen1k_cetus_53_rows.csv

# ---- (3) netem: lat/jit/loss grid, warm grpc (needs passwordless sudo tc on lo) ----
N=$HOME/ecloudsim_distributed_sandbox/evaluation_outputs/frozen1k_netem; mkdir -p $N
for dj in "0 0" "3 0.5" "20 1" "50 2"; do set -- $dj; lat=$1; jit=$2
  for loss in 0 0.1 1; do
    lbl=_meas; { [ "$lat" = "50" ] || [ "$loss" = "1" ]; } && lbl=_stress
    for s in 1 2 3; do
      sudo tc qdisc del dev lo root 2>/dev/null
      sudo tc qdisc add dev lo root netem delay ${lat}ms ${jit}ms loss ${loss}% 2>/dev/null
      run "netem_l${lat}_j${jit}_p${loss}${lbl}_s${s}" openscenario_1_flow_gt "$N" MIGRATION_MODE=warm TRANSFER_MODE=grpc
      sudo tc qdisc del dev lo root 2>/dev/null
    done
  done
done
sudo tc qdisc del dev lo root 2>/dev/null
echo "[netem] DONE $(date +%H:%M:%S)" > $N/_done; land "$N" frozen1k_cetus_netem_rows.csv

pkill -9 -f "CarlaUE4/Binaries" 2>/dev/null
echo "[cetus 1k tail: t8 + visible + netem] ALL DONE $(date +%H:%M:%S)" > $HOME/ecloudsim_distributed_sandbox/evaluation_outputs/_cetus_1k_tail_done
