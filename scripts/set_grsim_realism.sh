#!/usr/bin/env bash
# Turn off grSim's perfect world model before running anything that claims to
# validate the offline results.
#
# Out of the box grSim reports exact positions with no latency, which is the
# same idealisation the offline kinematic runs already make. A closed-loop run
# against a perfect world model therefore cannot falsify anything the offline
# runs assumed, which is the entire reason for running it.
#
# Both numbers below come from the 2024-07-19 TurtleRabbit vs NAMeC Division B
# log rather than from a plausible-sounding default:
#
#   noise 23 mm   frame-to-frame jitter in reported robot positions, measured
#                 while calibrating the trigger sweeps. grSim ships with 3 mm.
#   delay  9 ms   median of t_sent - t_capture over 4000 detection frames
#                 (p95 10.2, max 19.7). grSim ships with 0.
#
# grSim reads ~/.grsim.xml at startup only, so it has to be restarted after
# this runs. A backup is written alongside.
#
# Usage:  bash scripts/set_grsim_realism.sh [--noise-mm 23] [--delay-ms 9] [--off]
set -euo pipefail

XML="${GRSIM_CONFIG:-$HOME/.grsim.xml}"
NOISE_MM=23
DELAY_MS=9
ENABLE=true

while [ $# -gt 0 ]; do
  case "$1" in
    --noise-mm) NOISE_MM="$2"; shift 2 ;;
    --delay-ms) DELAY_MS="$2"; shift 2 ;;
    --off)      ENABLE=false; NOISE_MM=3; DELAY_MS=0; shift ;;
    *) echo "unknown argument: $1" >&2; exit 1 ;;
  esac
done

[ -f "$XML" ] || { echo "no grSim config at $XML. Run grSim once first." >&2; exit 1; }
cp "$XML" "$XML.bak.$(date +%s)"

python3 - "$XML" "$ENABLE" "$NOISE_MM" "$DELAY_MS" <<'PY'
import re, sys
path, enable, noise, delay = sys.argv[1], sys.argv[2] == "true", sys.argv[3], sys.argv[4]
xml = open(path).read()

def set_var(text, name, value):
    # The value sits on its own line between the named Var tag and its close.
    # Matching the name keeps this from hitting the identically-shaped blocks
    # for ball noise and the vanishing probabilities.
    pattern = re.compile(
        r'(<Var name="%s"[^>]*>\s*)([^<\s]+)(\s*</Var>)' % re.escape(name)
    )
    out, n = pattern.subn(lambda m: m.group(1) + value + m.group(3), text, count=1)
    if n != 1:
        raise SystemExit(f"could not find exactly one <Var name={name!r}>")
    return out

xml = set_var(xml, "Noise", "true" if enable else "false")
xml = set_var(xml, "Deviation for x values", f"{float(noise):.6f}")
xml = set_var(xml, "Deviation for y values", f"{float(noise):.6f}")
xml = set_var(xml, "Sending delay (milliseconds)", str(int(delay)))
open(path, "w").write(xml)
print(f"noise {'on' if enable else 'off'}, deviation {noise} mm, sending delay {delay} ms")
PY

echo
echo "Restart grSim for this to take effect:"
echo "  pkill grSim; cd ~/ssl-software/grSim && ./bin/grSim &"
