#!/bin/bash
set +u
# Cetus->atlas corridor landing bridge. Runs on ATLAS. Every 10 min it lands the
# cetus corridor logs (khonsu_corridor_land runs on cetus where the logs are),
# then scp's the rows+crossings into the atlas paper repo, renamed frozen1l->
# frozen1m so the watcher reads them next to frozengen_corridor_*. Exits after a
# final land once cetus writes _corridor_1m_done.
CETUS=tlandle@143.215.184.49
WT=/home/tlandle/cetus_1m_wt
R=$WT/evaluation_outputs/frozen_1m_corridor
OUT=/tmp/cetus_corr_land
MOCK=$HOME/repos/scale_out_nsdi/mock_data
LOG=/tmp/atlas_corridor_bridge.log
echo "[bridge] start $(date +%H:%M:%S)" > "$LOG"
land_and_pull () {
  ssh -o ConnectTimeout=15 "$CETUS" "mkdir -p $OUT; cd $WT && source ~/anaconda3/etc/profile.d/conda.sh && conda activate opencda310 && python scripts/khonsu_corridor_land.py $R --outdir $OUT --tau-ms 500 --complete-m 350 --tag khonsu-eval-freeze-1m --machine cetus" >>"$LOG" 2>&1 || true
  scp -o ConnectTimeout=15 "$CETUS:$OUT/frozen1l_corridor_rows.csv"      "$MOCK/frozen1m_corridor_rows.csv" >/dev/null 2>&1 || true
  scp -o ConnectTimeout=15 "$CETUS:$OUT/frozen1l_corridor_crossings.csv" "$MOCK/frozen1m_corridor_crossings.csv" >/dev/null 2>&1 || true
  local n=$(tail -n +2 "$MOCK/frozen1m_corridor_rows.csv" 2>/dev/null | wc -l)
  echo "[$(date +%H:%M:%S)] corridor rows landed=$n" >> "$LOG"
}
for i in $(seq 1 300); do
  land_and_pull
  if ssh -o ConnectTimeout=15 "$CETUS" "test -f $R/_corridor_1m_done" 2>/dev/null; then
    land_and_pull; echo "[bridge] corridor done, final land $(date +%H:%M:%S)" >> "$LOG"; break
  fi
  sleep 600
done
echo "[bridge] exit $(date +%H:%M:%S)" >> "$LOG"
