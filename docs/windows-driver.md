# Running grSim and its driver on Windows

Two working arrangements, both verified on Windows 10 Enterprise 19045 without
admin rights:

1. **Native grSim on Windows** (19 September 2026). Built from the same commit
   as the WSL build with MSYS2's prebuilt Qt, ODE and protobuf. No WSL
   involved at all. Run it headless for experiments; see below for why.
2. **grSim in WSL, driver on Windows over unicast** (18 September 2026). Kept
   for machines without an MSYS2 build.

## 1. Native Windows build

grSim's source has no Linux-only code, its CMake has `WIN32` branches and its
INSTALL.md documents a 64-bit Windows build via vcpkg. What kept it in WSL here
was toolchain, and MSYS2 (`C:\msys64`) removes that: `pacman` supplies gcc,
CMake, Ninja, Qt 5.15, ODE 0.16.6 with its CMake config, and protobuf as
prebuilt packages, so the build took minutes rather than the hours vcpkg needs
to compile Qt.

Recipe, from an MSYS2 MINGW64 shell, source cloned to a path without spaces
outside OneDrive (`%LOCALAPPDATA%\grsim\src`), Emma's
`grsim-isolated-config.patch` applied:

```bash
pacman -S --needed git mingw-w64-x86_64-toolchain mingw-w64-x86_64-cmake \
    mingw-w64-x86_64-ninja mingw-w64-x86_64-pkgconf mingw-w64-x86_64-qt5-base \
    mingw-w64-x86_64-ode mingw-w64-x86_64-protobuf
export CMAKE_POLICY_VERSION_MINIMUM=3.5
cmake -S src -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_CLIENTS=OFF -DCMAKE_INSTALL_PREFIX=install
cmake --build build --parallel 8 && cmake --install build
```

Two things in that recipe are not Windows-specific. `CMAKE_POLICY_VERSION_MINIMUM`
is needed because VarTypes and the protobuf 3.6.1 that grSim builds for itself
declare minimum CMake versions that CMake 4 refuses; the same would happen on
Linux with CMake 4. And grSim builds protobuf 3.6.1 deliberately: its
`cmake/modules/FindOrBuildProtobuf.cmake` says versions at or above 3.21 are
incompatible with how the project is set up.

The executable links Qt and the MinGW runtime from `C:\msys64\mingw64\bin`, so
`scripts/grsim-windows.ps1` puts that on PATH for the process and points
`RESEARCH_GRSIM_CONFIG` at `%LOCALAPPDATA%\grsim\grsim-windows.xml`, seeded
from the WSL configuration so it carries the same 23 mm noise and 9 ms delay.

```powershell
.\scripts\grsim-windows.ps1 -Headless
$env:PYTHONPATH = "src"
.\..\research-sdk\.venv\Scripts\python.exe scripts\drive_grsim.py `
  --planner voronoi --robots 1 --obstacles 11 --opponents log `
  --log \\wsl$\Ubuntu\home\paulk\ssl-gamelogs\2024-07-19_TurtleRabbit-vs-NAMeC.log.gz `
  --log-skip 180 --log-seconds 30 --duration 30 `
  --trigger geometric --trigger-centre-threshold-mm 180 `
  --replay-team both --traverse-x 2400 --lane-centre-mm 750 --force-full-build `
  --out-json results\grsim\windows-native-voronoi-30s.json
```

No `--grsim-host`: commands go to `127.0.0.1:20010` and vision arrives on the
multicast group, which works on the local host (481 frames in 4 s with the
window, 945 headless).

### Headless or it is not real time

| 30 s run, same configuration | GUI window | `--headless` | WSL-native reference |
|---|---|---|---|
| grSim capture clock advanced | 11.8 s | 25.0 s | 29.4 s |
| distance travelled | 16174 mm | 30010 mm | 30118 mm |
| effective speed | 0.54 m/s | 1.00 m/s | 1.00 m/s |
| crossings | 2 | 4 | 4 [4, 5] |
| ms per build | 4.14 | 3.69 | 3.78 [3.15, 4.88] |
| closest approach, mm | 698 | 644 | 690 [488, 711] |

With the window open on this machine's Intel integrated graphics, rendering
slows the physics loop to about 0.4 times real time and the robot covers half
the distance. Headless matches the WSL reference on every metric. The capture
clock span is the time between the first and last planning call, so it varies
with when plans happen; distance travelled is the decisive row.

The result file records `provenance.grsim_location: "windows"`, the local pid
and start time, and the config path actually read.

## 2. grSim in WSL, driver on Windows over unicast

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
