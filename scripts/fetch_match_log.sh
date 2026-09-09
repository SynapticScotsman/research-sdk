#!/usr/bin/env bash
# Fetch the recorded SSL match every log-based measurement in this repo uses.
#
# 2024-07-19, TurtleRabbit against NAMeC, Division B group phase, from the
# TIGERS Mannheim public game-log archive. Real match motion is the point: two
# earlier attempts to stir grSim robots into a plausible scene produced jitter
# in place rather than travel, and even a working stirrer would only have
# measured the stirrer. Roadmap churn is a property of how robots actually
# move, so it has to come from a real game.
#
# Downloaded to $HOME rather than into the repo. It is 159 MB compressed, the
# working copy sits in OneDrive, and a file that size would sync forever.
#
# Usage:  bash scripts/fetch_match_log.sh
# Then:   python scripts/measure_search_reuse.py --source log --log "$LOG"
set -euo pipefail

DIR="${SSL_GAMELOG_DIR:-$HOME/ssl-gamelogs}"
FILE="$DIR/2024-07-19_TurtleRabbit-vs-NAMeC.log.gz"
URL='https://seafile.tigers-mannheim.de/d/e85851d9bc9944bf95bb/files/?p=/gamelogs/2024/div-b/2024-07-19_11-30_GROUP_PHASE_TurtleRabbit-vs-NAMeC.log.gz&dl=1'

mkdir -p "$DIR"

if [ -s "$FILE" ]; then
  echo "already have $(du -h "$FILE" | cut -f1) at $FILE"
else
  echo "downloading 159 MB to $FILE ..."
  curl -L --fail --max-time 1800 -o "$FILE" "$URL"
fi

# A truncated download decodes for a while and then fails mid-run, minutes into
# a measurement. Both checks below are cheap and catch it here instead.
echo "--- gzip integrity ---"
gzip -t "$FILE" && echo "gzip OK"

echo "--- SSL log header ---"
# `|| true` is load-bearing: head closes the pipe after 12 bytes, zcat takes
# SIGPIPE, and pipefail would otherwise abort the script before it reports.
MAGIC=$(zcat "$FILE" 2>/dev/null | head -c 12 || true)
if [ "$MAGIC" = "SSL_LOG_FILE" ]; then
  echo "magic OK: SSL_LOG_FILE"
else
  echo "NOT an SSL log: magic was '$MAGIC'" >&2
  exit 1
fi

echo
echo "Ready. Pass it with:"
echo "  --log \"$FILE\""
