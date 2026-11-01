<#
Start, stop or restart Planar Motor's simulation tool's PMC (VirtualPMC.exe) without its window,
for testing the controller against it (PMC_IP=127.0.0.1) on a Windows PC.

    powershell -ExecutionPolicy Bypass -File tools\pmc-sim.ps1 restart
    powershell -ExecutionPolicy Bypass -File tools\pmc-sim.ps1 status

The tool is Planar Motor's and not commercially available: it is not in this repository. The
script runs it from where it is installed (PMC_SIM_DIR, default the installer's folder), or from
a copy in pmc-sim/ next to this repository's root, which git and Docker ignore.

The engine is what the window (Virtual PMC UI.exe) starts: Dlls\VirtualPMC.exe, run from the
tool's folder; it reads the table and xbots from the folder's config\ and listens on UDP port
8888. While the window is open, an engine started here may never listen: start it from the
window then.

Every pmclib program must disconnect from it when it ends (the controller does): the engine
writes an error to its trace log (%APPDATA%\Planar Motor Inc\Virtual PMC\data\PMTraceLog.txt)
for every read of a connection left open, gigabytes an hour, until it takes no new connections.
#>
param(
    [ValidateSet("start", "stop", "restart", "status")]
    [string]$Action = "status",
    [int]$TimeoutS = 30
)

$ErrorActionPreference = "Stop"
$Port = 8888

function Find-Tool {
    $candidates = @($env:PMC_SIM_DIR, (Join-Path $PSScriptRoot "..\pmc-sim"),
                    "C:\Program Files\Planar Motor Inc\Planar Motor Simulation Tool")
    foreach ($dir in $candidates) {
        if ($dir -and (Test-Path (Join-Path $dir "Dlls\VirtualPMC.exe"))) { return (Resolve-Path $dir).Path }
    }
    throw "VirtualPMC.exe not found: install the Planar Motor Simulation Tool, or set PMC_SIM_DIR to its folder"
}

function Get-Engine { Get-Process VirtualPMC -ErrorAction SilentlyContinue }

function Test-Listening {
    # The engine's own UDP port: another program may hold TCP 8888 (an OpenTelemetry collector does).
    $engine = Get-Engine
    if (-not $engine) { return $false }
    $udp = Get-NetUDPEndpoint -LocalPort $Port -ErrorAction SilentlyContinue |
           Where-Object { $engine.Id -contains $_.OwningProcess }
    return [bool]$udp
}

function Stop-Engine {
    $engine = Get-Engine
    if (-not $engine) { "not running"; return }
    $engine | Stop-Process -Force -Confirm:$false
    $engine | Wait-Process -Timeout 10 -ErrorAction SilentlyContinue
    "stopped (pid $($engine.Id -join ', '))"
}

function Start-Engine {
    if (Get-Engine) { "already running (pid $((Get-Engine).Id -join ', '))"; return }
    $dir = Find-Tool
    $engine = Start-Process -FilePath (Join-Path $dir "Dlls\VirtualPMC.exe") -WorkingDirectory $dir `
                            -WindowStyle Hidden -PassThru
    $deadline = (Get-Date).AddSeconds($TimeoutS)
    while (-not (Test-Listening)) {
        if ($engine.HasExited) { throw "VirtualPMC.exe exited with code $($engine.ExitCode)" }
        if ((Get-Date) -gt $deadline) {
            throw ("VirtualPMC.exe is not listening on port $Port after $TimeoutS s. Started on its own while the " +
                   "tool's window (Virtual PMC UI) is open, it may take no connections: stop it and start the " +
                   "simulation from the window")
        }
        Start-Sleep -Milliseconds 500
    }
    "started from $dir (pid $($engine.Id)), listening on port $Port"
}

switch ($Action) {
    "start" { Start-Engine }
    "stop" { Stop-Engine }
    "restart" { Stop-Engine; Start-Engine }
    "status" {
        $engine = Get-Engine
        if ($engine) { "running (pid $($engine.Id -join ', ')), port $Port " + $(if (Test-Listening) { "open" } else { "not open" }) }
        else { "not running" }
    }
}
