#!/bin/bash
set -u
cd /home/atlas/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox
source /home/atlas/anaconda3/etc/profile.d/conda.sh && conda activate opencda310
R=evaluation_outputs/frozen1b
CARLA_ROOT=$HOME/carla-0.9.15
# wait for the ordered batch to finish all blocks
while [ ! -f $R/_all_done ]; do sleep 120; done
carla_up () { pgrep -f "CarlaUE4/Binaries" >/dev/null && timeout 5 bash -c "exec 3<>/dev/tcp/localhost/2000" 2>/dev/null; }
carla_restart () { pkill -9 -f "CarlaUE4/Binaries" 2>/dev/null; sleep 5; ( cd $CARLA_ROOT && setsid nohup ./CarlaUE4.sh -RenderOffScreen >/dev/null 2>&1 & ); for i in $(seq 1 24); do sleep 5; carla_up && { sleep 15; return 0; }; done; return 1; }
run () { local tag="$1"; local scn="$2"; local dir="$3"; shift 3
  mkdir -p $dir; local log="$dir/${tag}.log"
  [ -f "$log" ] && grep -q RUNROW "$log" && grep -q actor_id "$log" && { echo "skip $tag"; return; }
  carla_restart || exit 1; echo "[$(date +%H:%M:%S)] $tag"
  env "$@" ONCOMING_SPEED=12 TRIGGER_DIST=300 timeout -k 30 900 python ecav.py -t "$scn" --apply_ml > "$log" 2>&1
  grep -q actor_id "$log" || { carla_restart; env "$@" ONCOMING_SPEED=12 TRIGGER_DIST=300 timeout -k 30 900 python ecav.py -t "$scn" --apply_ml > "$log" 2>&1; }
}
# FAULT ARMS: 6 faults x fencing{on,off} x5 on flow warm
F=$R/fault
for rep in $(seq 1 5); do for flt in lost_prepare lost_ack lost_commit dup_commit reorder dst_crash; do
  run "fault_${flt}_fenceon_r${rep}"  openscenario_1_flow_gt $F MIGRATION_MODE=warm FAULT_MODE=$flt FENCING=on
  run "fault_${flt}_fenceoff_r${rep}" openscenario_1_flow_gt $F MIGRATION_MODE=warm FAULT_MODE=$flt FENCING=off
done; done
echo "fault done" > $R/_fault_done
# NETEM MATRIX: real gRPC transfer under tc netem (NOPASSWD sudo; cleaned each cell)
N=$R/netem
( setsid nohup python -m ecav.core.application.edge.migration.relay_server 50771 >/tmp/relay.log 2>&1 & )
sleep 3
for rep in 1 2 3; do for lat in 0 5 20 50; do for loss in 0 1 5; do
  sudo tc qdisc add dev lo root netem delay ${lat}ms loss ${loss}% 2>/dev/null
  run "netem_l${lat}_p${loss}_r${rep}" openscenario_1_flow_gt $N MIGRATION_MODE=warm TRANSFER_MODE=grpc
  sudo tc qdisc del dev lo root 2>/dev/null
done; done; done
echo "netem done" > $R/_netem_done
# T19b LOAD SWEEP: platoon {1,2,4,8,16} x arms x5 on burst scenario (FLOW via burst variant)
T=$R/t19b
for rep in $(seq 1 5); do for m in warm edgewarp handover_snapshot cold; do
  run "t19b_${m}_r${rep}" openscenario_1_burst_gt $T MIGRATION_MODE=$m
done; done
echo "t19b done" > $R/_t19b_done
echo "[ATLAS TAIL] ALL DONE" > $R/_tail_done
