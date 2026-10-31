<#
Start, stop or restart Planar Motor's simulation tool's PMC (VirtualPMC.exe) without its window,
for testing the controller against it (PMC_IP=127.0.0.1) on a Windows PC.

    powershell -ExecutionPolicy Bypass -File tools\pmc-sim.ps1 restart
    powershell -ExecutionPolicy Bypass -File tools\pmc-sim.ps1 status

The tool is Planar Motor's and not commercially available: it is not in this repository. The
script runs it from where it is installed (PMC_SIM_DIR, default the installer's folder), or from
a copy in pmc-sim/ next to this repository's root, which git and Docker ignore.

The engine is what the window (Virtual PMC UI.exe) starts: Dlls\VirtualPMC.exe, run from the
tool's folder; it reads the table and xbots from the folder's config\ and listens on port 8888.
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
    try {
        $client = New-Object System.Net.Sockets.TcpClient
        $client.Connect("127.0.0.1", $Port)
        $client.Close()
        return $true
    } catch { return $false }
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
        if ((Get-Date) -gt $deadline) { throw "VirtualPMC.exe is not listening on port $Port after $TimeoutS s" }
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
