# Start the native Windows build of grSim.
#
#   .\scripts\grsim-windows.ps1 -Headless    # for experiments (see below)
#   .\scripts\grsim-windows.ps1              # with a window, to watch
#   .\scripts\grsim-windows.ps1 -Stop
#
# USE -Headless FOR ANY RUN YOU WILL QUOTE. Measured 19/09/2026 on this
# machine's Intel integrated graphics: with the window open, grSim's own
# capture clock advanced 11.8 s in 30 s of wall time, the robot averaged
# 0.54 m/s against a commanded 1.2 and completed 2 crossings where the WSL
# reference completes 4. Headless, the same 30 s run travelled 30010 mm at
# 1.00 m/s with 4 crossings, matching the WSL reference (30118 mm, 1.00 m/s).
# Rendering the window is what slows the physics loop.
#
# WHY THIS EXISTS. grSim's source has no Linux-only code and its INSTALL.md
# documents a 64-bit Windows build; what kept it in WSL here was the
# toolchain. MSYS2 (C:\msys64) provides gcc, CMake, Ninja, Qt 5.15, ODE 0.16.6
# and protobuf as prebuilt packages, so the build is minutes, not the hours a
# vcpkg build of Qt takes. Built 19/09/2026 at grSim fe2bd29, the same commit
# as the WSL build and Emma's pin, with her config-isolation patch applied.
#
# The executable links the MinGW runtime and Qt DLLs from C:\msys64\mingw64\bin,
# so that directory goes on PATH for the process. Installing
# mingw-w64-x86_64-qt5-tools and running windeployqt would make a standalone
# folder; not done yet.
#
# WHERE IT LIVES: %USERPROFILE%\grsim (src, install, grsim-windows.xml, logs),
# not %LOCALAPPDATA%. The first build was made from inside the Claude desktop
# app, a packaged (MSIX) process, and Windows redirects every write such a
# process makes under AppData\Local into the app's private
# AppData\Local\Packages\Claude_*\LocalCache\Local. The install was complete
# and ran, and was invisible to an ordinary PowerShell and to WSL. Checked
# 19/09/2026 by writing a probe file and listing the real path from WSL. The
# profile root and C:\msys64 are not redirected.
#
# RESEARCH_GRSIM_CONFIG is the hook Emma's patch adds: grSim reads that file
# instead of %USERPROFILE%\.grsim.xml and does not write settings back on exit.
# grsim-windows.xml was seeded from the WSL configuration, so it carries the
# same realism settings (23 mm vision noise, 9 ms sending delay), vision on
# the multicast group 224.5.23.2:10020 and commands on 20010.
#
# Rebuild recipe, from an MSYS2 MINGW64 shell:
#   pacman -S --needed git mingw-w64-x86_64-toolchain mingw-w64-x86_64-cmake \
#       mingw-w64-x86_64-ninja mingw-w64-x86_64-pkgconf mingw-w64-x86_64-qt5-base \
#       mingw-w64-x86_64-ode mingw-w64-x86_64-protobuf
#   export CMAKE_POLICY_VERSION_MINIMUM=3.5   # VarTypes and protobuf 3.6.1 declare pre-3.5 minimums
#   cmake -S src -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_CLIENTS=OFF -DCMAKE_INSTALL_PREFIX=install
#   cmake --build build --parallel 8 && cmake --install build
# grSim builds protobuf 3.6.1 itself on purpose: its cmake/modules says
# versions >= 3.21 are incompatible with how the project is set up.
param(
    [switch]$Headless,
    [switch]$Stop,
    [string]$Root = "$env:USERPROFILE\grsim"
)

if ($Stop) {
    Get-Process grSim -ErrorAction SilentlyContinue | Stop-Process -Force
    Write-Host "grSim stopped"
    exit 0
}

$exe = Join-Path $Root "install\bin\grSim.exe"
if (-not (Test-Path $exe)) { throw "no grSim.exe at $exe; see the rebuild recipe in this script" }
if (Get-Process grSim -ErrorAction SilentlyContinue) { Write-Host "grSim already running"; exit 0 }

$env:PATH = "C:\msys64\mingw64\bin;" + $env:PATH
$env:RESEARCH_GRSIM_CONFIG = Join-Path $Root "grsim-windows.xml"
$args = @()
if ($Headless) { $args += "--headless" }
# No -RedirectStandardOutput/-RedirectStandardError here. Those make .NET start
# grSim with inheritable handles to this shell's stdout pipe, so any caller that
# pipes this script's output (`... | Select-Object`) blocks until grSim EXITS,
# not until this script returns. Measured 19/09/2026: a wrapper hung for over
# five minutes with the launcher long gone, and released the instant grSim
# was stopped. grSim prints almost nothing; if it dies at start, HasExited
# below reports it.
$p = Start-Process -FilePath $exe -ArgumentList $args -WorkingDirectory (Split-Path $exe) -PassThru
# Return only when vision is actually flowing, so that "the launcher returned"
# means "a driver can connect now" rather than "a process exists".
$udp = New-Object System.Net.Sockets.UdpClient
$udp.ExclusiveAddressUse = $false
$udp.Client.SetSocketOption([Net.Sockets.SocketOptionLevel]::Socket, [Net.Sockets.SocketOptionName]::ReuseAddress, $true)
$udp.Client.Bind((New-Object System.Net.IPEndPoint ([System.Net.IPAddress]::Any, 10020)))
$udp.JoinMulticastGroup([System.Net.IPAddress]::Parse("224.5.23.2"))
$udp.Client.ReceiveTimeout = 2000
$sw = [Diagnostics.Stopwatch]::StartNew()
$ready = $false
while ($sw.Elapsed.TotalSeconds -lt 60) {
    if ($p.HasExited) { $udp.Close(); throw "grSim exited with code $($p.ExitCode) before publishing vision; run it by hand from a MINGW64 shell to see its output" }
    try { $ep = $null; [void]$udp.Receive([ref]$ep); $ready = $true; break } catch { }
}
$udp.Close()
if (-not $ready) { throw "grSim is running (pid $($p.Id)) but published no vision on 224.5.23.2:10020 within 60 s" }
Write-Host ("grSim pid {0}, first vision frame after {1:N1} s, config {2}" -f $p.Id, $sw.Elapsed.TotalSeconds, $env:RESEARCH_GRSIM_CONFIG)
Write-Host "commands 127.0.0.1:20010, vision 224.5.23.2:10020 (multicast, works on the local host)"
