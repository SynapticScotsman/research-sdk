#!/usr/bin/env bash
# Two long jobs, in order, with nothing overlapping.
#
# Job 1 is real time and timing-sensitive, so it runs alone. Job 2 saturates
# eight cores; running it alongside job 1 would inflate the planning times job 1
# exists to measure.
#
# Job 1  paired three-planner comparison on one recorded clip. Same clip, same
#        start and goal, same controller, same trigger, same noise and delay.
#        Only the planner varies. Planners alternate run to run so that any
#        drift in the simulator lands on all three equally rather than on
#        whichever went last. Answers RQ1 under recorded motion and checks
#        whether the offline ordering survives closed-loop dynamics.
#
# Job 2  path stability against obstacle speed, offline. The committed stability
#        result is measured across triggers at a single obstacle speed of
#        1.0 m/s; this is the stated gap. Both scenarios, six speeds.
#
# Every output filename is unique and existing files are skipped, so an
# interrupted night can be resumed by rerunning this.
#
# Usage:  bash scripts/overnight_runs.sh [repeats]
set -uo pipefail

REPEATS="${1:-15}"
W="/mnt/c/Users/30068379/OneDrive - Western Sydney University/Code/TurtleRabbit/research-sdk-plannertest"
E="/mnt/c/Users/30068379/OneDrive - Western Sydney University/Code/TurtleRabbit/research-sdk"
PY="$HOME/rsdk-venv/bin/python"
LOG="$W/results/overnight.log"
GRSIM_DIR="$W/results/grsim/paired"
STAB_DIR="$E/results/stability-speed"
LOGFILE="$HOME/ssl-gamelogs/2024-07-19_TurtleRabbit-vs-NAMeC.log.gz"

mkdir -p "$GRSIM_DIR" "$STAB_DIR"
say() { echo "[$(date '+%H:%M:%S')] $*" | tee -a "$LOG"; }

say "starting, $REPEATS repeats per planner"

# --- job 1 ------------------------------------------------------------------
if ! pgrep grSim >/dev/null; then
  say "grSim not running, starting it"
  (cd "$HOME/ssl-software/grSim" && nohup ./bin/grSim >"$HOME/grsim_run.log" 2>&1 &)
  sleep 10
fi
pgrep grSim >/dev/null && say "grSim pid $(pgrep grSim | head -1)" || say "WARNING: grSim not up, job 1 will fail"

for i in $(seq 1 "$REPEATS"); do
  for P in voronoi visibility prm; do
    OUT="$GRSIM_DIR/${P}_run$(printf '%02d' "$i").json"
    if [ -e "$OUT" ]; then say "skip $(basename "$OUT")"; continue; fi
    # --trigger-centre-threshold-mm 90 matches the offline distance test rather
    # than the driver's 180 mm default. It does not equalise every other setting.
    PYTHONPATH="$W/src" timeout 120 "$PY" "$W/scripts/drive_grsim.py" \
      --planner "$P" --robots 1 --obstacles 6 \
      --opponents log --log "$LOGFILE" \
      --log-skip 180 --log-seconds 30 --duration 30 \
      --trigger geometric --trigger-centre-threshold-mm 90 \
      --out-json "$OUT" >>"$LOG" 2>&1 \
      && say "job1 $P run $i ok" || say "job1 $P run $i FAILED"
  done
done
say "job 1 done: $(ls "$GRSIM_DIR" | wc -l) result files"

# --- job 2 ------------------------------------------------------------------
say "job 2: stability against obstacle speed, offline"
for SC in "scenario_1 (7 obstacles)" "scenario_2 (5 obstacles)"; do
  TAG=$(echo "$SC" | grep -oE '^scenario_[0-9]+')
  for SPEED in 0.5 1.0 1.5 2.0 2.5 3.0; do
    OUT="$STAB_DIR/${TAG}_speed${SPEED}.txt"
    if [ -e "$OUT" ]; then say "skip $(basename "$OUT")"; continue; fi
    # rsdk-venv, not the research-sdk .venv: that one is a Windows layout with
    # Scripts/python.exe and has no bin/python for WSL to run.
    PYTHONPATH="$E/src:$E/scripts" "$PY" "$E/scripts/report_stability.py" \
      --scenario "$SC" --samples 200 --obstacle-speed "$SPEED" --workers 8 \
      >"$OUT" 2>&1 \
      && say "job2 $TAG at $SPEED m/s ok" || say "job2 $TAG at $SPEED m/s FAILED"
  done
done
say "job 2 done: $(ls "$STAB_DIR" | wc -l) result files"
say "all finished"
