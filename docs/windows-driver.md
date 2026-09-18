# Running the grSim driver on Windows

grSim stays in WSL. The driver, `scripts/drive_grsim.py`, runs under Windows
Python. Verified 18 September 2026 on Windows 10 Enterprise 19045, no admin
rights, no firewall rule.

## Why this was not possible before

WSL2 sits behind a NAT. grSim publishes vision to the multicast group
`224.5.23.2:10020`, and multicast does not cross that NAT, so every
grSim-driven experiment had to run inside WSL. Windows 11's mirrored
networking removes the NAT; this machine is Windows 10.

Unicast crosses the NAT in both directions. A UDP probe from WSL to the
Windows side of the WSL adapter arrived at a Windows Python socket with no
firewall change, so the fix is to tell grSim to send vision to that address
instead of the multicast group, and to tell the driver to send commands to the
WSL address instead of `127.0.0.1`.

## Setup, once per grSim start

The two addresses, from inside WSL:

```bash
hostname -I | cut -d" " -f1          # WSL address, for --grsim-host   (172.19.186.220 here)
ip route show default | cut -d" " -f3   # Windows address, for grSim   (172.19.176.1 here)
```

They can change when WSL restarts; check before a batch.

Point grSim's vision at Windows and restart it:

```bash
bash scripts/set_grsim_vision.sh unicast
bash scripts/set_grsim_vision.sh show
```

`unicast` with no address uses the default gateway, which is the Windows side.
The script backs up `~/.grsim.xml` the first time and restarts grSim, because
the setting is read at start.

## Run

From PowerShell, in the worktree, with the Windows interpreter that has the
SDK's dependencies:

```powershell
$env:PYTHONPATH = "src"
.\..\research-sdk\.venv\Scripts\python.exe scripts\drive_grsim.py `
  --planner voronoi --robots 1 --obstacles 11 --opponents log `
  --log \\wsl$\Ubuntu\home\paulk\ssl-gamelogs\2024-07-19_TurtleRabbit-vs-NAMeC.log.gz `
  --log-skip 180 --log-seconds 30 --duration 30 `
  --trigger geometric --trigger-centre-threshold-mm 180 `
  --replay-team both --traverse-x 2400 --lane-centre-mm 750 --force-full-build `
  --grsim-host 172.19.186.220 --out-json results\grsim\windows-voronoi-30s.json
```

The match log is read straight from the WSL filesystem over `\\wsl$`; gzip
handles the UNC path.

## What it produced, against the WSL-native reference

Same configuration as `results/grsim/div-b-11obs-y750-180/` (15 runs, driven
from inside WSL). One 30 s run from Windows:

| quantity | Windows-driven | WSL-native, median [min, max] |
|---|---|---|
| crossings | 4 | 4 [4, 5] |
| replans | 5 | 5 [5, 6] |
| direct-line calls | 0 | 0 |
| failed calls | 0 | 0 |
| ms per roadmap build | 3.43 | 3.78 [3.15, 4.88] |
| closest approach, vision, mm | 723 | 690 [488, 711] |

Every value falls inside the native envelope. The millisecond figure is a
different interpreter (Windows Python 3.13 against WSL Python 3.14), so treat
it as consistent rather than identical.

The 8 s smoke run before it travelled 7389 mm, placed 11 replayed robots in 155
frames with 48 held positions, and recorded 6 blue and 6 yellow robots in play,
which is the full Division B scene.

## What changes in the result file

`provenance.grsim_location` is `"wsl"`, and the grSim pid, start time, noise,
delay and `vision_address` are read through `wsl.exe` so the record is as
complete as a native run's. `meta.grsim_host` records the address commands
went to.

## The one thing to remember

grSim has a single vision destination. While it points at Windows, nothing
inside WSL receives vision, so `overnight_runs.sh` and any driver launched from
WSL will report "Only saw 0 blue and 0 yellow robots". Switch back first:

```bash
bash scripts/set_grsim_vision.sh multicast
```

`show` tells you which mode you are in.

## Not verified

Mac. The Python side should run wherever PySide6 does, but grSim's vision
target would need the host's address on whatever virtual network the simulator
runs in, and a Mac has no WSL. Nobody has tried it.
