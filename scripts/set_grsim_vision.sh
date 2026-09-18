#!/usr/bin/env bash
# Point grSim's vision output at the right consumer and restart it.
#
#   bash scripts/set_grsim_vision.sh multicast        # drivers running inside WSL
#   bash scripts/set_grsim_vision.sh unicast 172.19.176.1   # driver running on Windows
#   bash scripts/set_grsim_vision.sh show
#
# WHY THIS EXISTS. WSL2 sits behind a NAT, and the multicast group grSim
# normally publishes vision on (224.5.23.2:10020) never reaches Windows, so
# every grSim-driven experiment used to have to run inside WSL. Windows 11's
# mirrored networking removes the NAT; this machine is Windows 10.
#
# What does cross the NAT is unicast, both ways. Measured 18/09/2026: a UDP
# probe from WSL to the Windows side of the WSL adapter (172.19.176.1:10020)
# arrived with no firewall rule and no admin rights, and drive_grsim.py then
# ran on Windows Python against grSim in WSL, receiving vision by unicast and
# sending commands to the WSL address (--grsim-host). One 30 s run matched the
# WSL-native set for the same configuration.
#
# The cost: grSim has ONE vision destination. While it is set to the Windows
# address, nothing inside WSL sees vision, so the overnight scripts that run
# drive_grsim.py from WSL will report "Only saw 0 blue and 0 yellow robots".
# Switch back with `multicast` before using them. `show` tells you which you
# are on. The setting lives in ~/.grsim.xml and is read at grSim start, hence
# the restart.
#
# The Windows-side address is the default gateway as WSL sees it:
#     ip route show default | cut -d" " -f3
# and the WSL address the driver needs for --grsim-host is:
#     hostname -I | cut -d" " -f1
set -euo pipefail

CFG="$HOME/.grsim.xml"
GRSIM_DIR="$HOME/ssl-software/grSim"

current() {
  grep -A1 "Vision multicast address" "$CFG" | tail -1 | tr -d '[:space:]'
}

case "${1:-show}" in
  show)
    echo "vision destination: $(current)"
    echo "grSim running:      $(pgrep -x grSim >/dev/null && echo yes || echo no)"
    echo "WSL address:        $(hostname -I | cut -d' ' -f1)"
    echo "Windows address:    $(ip route show default | cut -d' ' -f3)"
    exit 0
    ;;
  multicast)
    TARGET="224.5.23.2"
    ;;
  unicast)
    TARGET="${2:-$(ip route show default | cut -d' ' -f3)}"
    ;;
  *)
    echo "usage: $0 {show|multicast|unicast [ip]}" >&2
    exit 2
    ;;
esac

[ -f "$CFG.multicast-backup" ] || cp "$CFG" "$CFG.multicast-backup"
# The value sits on the line after the tag; replace whatever is there.
sed -i "/Vision multicast address/{n;s#[0-9.]\+#$TARGET#}" "$CFG"
echo "vision destination: $(current)"

if pgrep -x grSim >/dev/null; then
  pkill -x grSim
  sleep 1
fi
(cd "$GRSIM_DIR" && nohup ./bin/grSim >"$HOME/grsim_run.log" 2>&1 &)
sleep 8
if pgrep -x grSim >/dev/null; then
  echo "grSim restarted, pid $(pgrep -x grSim | head -1)"
else
  echo "grSim did not start; see ~/grsim_run.log" >&2
  exit 1
fi
