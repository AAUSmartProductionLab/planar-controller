# Getting started

This page installs the controller, brings the table up and makes a first move, by hand with
UaExpert and from Python.

## What you need

- **A PMC** that the controller's machine can reach over the network, with firmware 117.17.35
  or later (what the pmclib 117.9.1 in `dependencies/` needs). For tests, Planar Motor's
  simulator on a Windows PC also works (see [Testing](testing.md#planar-motors-simulator)).
- **A machine for the controller**: Linux with Docker (the installer installs Docker if it is
  missing), or Python 3.11 or later without Docker. On Linux, pmclib also needs mono.
- **An OPC UA client**: your orchestrator, or [UaExpert](https://www.unified-automation.com/products/development-tools/uaexpert.html)
  to try it by hand.

Only one program can command the PMC at a time: its master. Disconnect Planar Motor's PMT
before you Reset the controller's table. If PMT takes mastership later, the table aborts with
MastershipLost (32).

## Install with the installer

On a Linux machine that can reach the PMC, the installer installs Docker if it is missing,
clones the repository, builds the image and starts the controller, which then restarts with the
machine. With `--autostart`, the controller also Resets and Starts the table whenever it
starts:

```bash
curl -fsSL https://raw.githubusercontent.com/AAUSmartProductionLab/planar-controller/main/install.sh \
  | bash -s -- --pmc-ip 192.168.10.100 --autostart
```

From a clone:

```bash
git clone https://github.com/AAUSmartProductionLab/planar-controller.git
cd planar-controller
./install.sh --pmc-ip 192.168.10.100 --autostart
```

| Option | |
| --- | --- |
| `--pmc-ip IP` | the PMC's address (empty: search the local network) |
| `--port N` | the OPC UA port (4840) |
| `--shuttles LIST` | the xbot ids to offer, e.g. `1,2,3` (empty: the ones the PMC knows) |
| `--loading-station X,Y` | in mm: the free xbots queue up there, and a Reset sends them there |
| `--idle-rz DEG` | a multiple of 90: the free xbots are turned to it, and a Reset turns them all |
| `--mode free\|line`, `--line-layout FILE\|URL` | anywhere on the table, or only on the lanes of a line layout |
| `--autostart` / `--no-autostart` | Reset and Start the table when the controller starts, or leave it Stopped (the default) |
| `--dir DIR` | where to clone it when not run from a clone (`~/planar-controller`) |
| `--branch NAME` | the branch to install (`main`) |
| `--no-start` | install and build only |

The options are written to `.env`, the settings file (see [Configuration](configuration.md)).
Run the installer again to update: it pulls, rebuilds and restarts, and keeps `.env`, changing
only what the options say.

## Install with Docker Compose by hand

```bash
git clone https://github.com/AAUSmartProductionLab/planar-controller.git
cd planar-controller
cp .env.example .env              # then set PMC_IP and whatever else you need
docker compose up -d --build      # build and start; it restarts with the machine
docker compose logs -f            # follow the log
docker compose up -d              # after changing .env
docker compose down               # stop
```

The container uses host networking: the PMC's auto search needs it, and the OPC UA port is the
machine's own. Docker Desktop on Windows and macOS needs host networking switched on in its
settings. The alternative, publishing the port, is in a comment in `docker-compose.yml`; then
set `PMC_IP`, since the auto search does not work.

The image is healthy while its OPC UA server takes connections. `config/` is mounted into the
container, so `LINE_LAYOUT=config/my-line.json` and `ZONES=config/zones.json` refer to files
there.

## Run without Docker

```bash
pip install -r requirements.txt dependencies/pmclib-117.9.1-py3-none-any.whl
PMC_IP=127.0.0.1 AUTOSTART=1 python -m planar
python -m planar --help           # every setting, also as an option
```

On Windows, port 4840 is often taken by the OPC UA Local Discovery Server, and 4841 by UaExpert.
Use another port then: `OPCUA_ENDPOINT=opc.tcp://127.0.0.1:4850/`.

The controller stops on Ctrl+C, Ctrl+Break and SIGTERM (`docker stop`), and disconnects from the
PMC. Do not kill it: see [Troubleshooting](troubleshooting.md#the-simulators-trace-log-grows-by-gigabytes).

## Bring the table up

The controller starts with every module Stopped, as the IEC 61499 modules do. Reset and Start the
table, and the shuttles follow:

1. `PlanarTable/Module/Reset`: the table's Resetting connects to the PMC and takes mastership.
   It recovers xbots the PMC switched off after an accident, and levitates the xbots. With a
   loading station, it then sends the shuttles into the loading queue. The table turns Idle
   (4), and each shuttle runs its own Resetting and turns Idle.
2. `PlanarTable/Module/Start`: the table and every shuttle turn Execute (6).

With `AUTOSTART=1` the controller does both itself when it starts, and keeps trying while the
PMC is not there yet.

At the Reset, the log shows the table as the PMC reports it, one character per slot, the top row
first (`#` a slot, `.` a missing flyway):

```text
INFO planar.coordinator: PMC connected: table 4x3 flyways, holes [(2, 1)], xbots [1, 2, 3]
INFO planar.coordinator: table slots:
########
########
####..##
####..##
########
########
```

## A first move with UaExpert

1. Add the server: *Custom Discovery*, `opc.tcp://<host>:4840`, security *None*.
2. In the address space, open `Objects/PlanarTable/Module`. Call `Reset` with any session,
   e.g. `ua`: a module nobody occupies takes commands from any session. Watch `State` go from
   15 (Resetting) to 4 (Idle), then call `Start`. `State` turns 6 (Execute), and so do the
   shuttles' states.
3. Open `Objects/PlanarShuttle1/Occupation` and call `Occupy` with a session of your own, e.g.
   `my-session-1`. `Occupied` turns true.
4. Open `Objects/PlanarShuttle1/Skills/MoveToPosition` and call `Start` with your session and
   `X = 780`, `Y = 660`, `Rz = 90`. It answers `[true, 0]` at once. `State` turns 1 (Running),
   then 3 (Succeeded) when the xbot has arrived, and 0 (Idle) 1.5 s later. `Results` hold where
   it ended. `PlanarShuttle1/Equipment/Xbot/X`, `Y` and `Rz` show the pose as it moves.
5. Call `Release` with your session.

A refused call answers `[false, ErrorID]`; see [ErrorID](opcua-interface.md#errorid).

## A first move from Python

With [asyncua](https://github.com/FreeOpcUa/opcua-asyncio) (`pip install asyncua`):

```python
import asyncio
import uuid

from asyncua import Client, ua

NAMESPACE = "urn:aau:smartproductionlab:planar"


async def main():
    async with Client("opc.tcp://127.0.0.1:4840/") as client:
        ns = await client.get_namespace_index(NAMESPACE)

        async def node(*path):
            return await client.nodes.objects.get_child([f"{ns}:{name}" for name in path])

        async def wait(path, values):
            while (value := await (await node(*path)).read_value()) not in values:
                await asyncio.sleep(0.05)
            return value

        def text(value):
            return ua.Variant(value, ua.VariantType.String)

        def number(value):
            return ua.Variant(float(value), ua.VariantType.Double)

        # Bring the table up (a table nobody occupies takes any session)
        table = await node("PlanarTable", "Module")
        print("Reset:", await table.call_method(f"{ns}:Reset", text("")))
        await wait(("PlanarTable", "Module", "State"), (4,))      # Idle
        print("Start:", await table.call_method(f"{ns}:Start", text("")))
        await wait(("PlanarTable", "Module", "State"), (6,))      # Execute

        # Occupy a shuttle, move it, release it
        session = str(uuid.uuid4())
        occupation = await node("PlanarShuttle1", "Occupation")
        print("Occupy:", await occupation.call_method(f"{ns}:Occupy", text(session)))
        skill = await node("PlanarShuttle1", "Skills", "MoveToPosition")
        print("Start:", await skill.call_method(f"{ns}:Start", text(session), number(780), number(660), number(90)))
        state = await wait(("PlanarShuttle1", "Skills", "MoveToPosition", "State"), (3, 4))  # Succeeded, Failed
        print("ended in state", state)
        print("Release:", await occupation.call_method(f"{ns}:Release", text(session)))


asyncio.run(main())
```

Each call prints `[True, 0]` when it is accepted. [Examples](examples.md) has a small client
class built the same way, and longer examples.

## Next steps

- Set up a loading station, the idle orientation, line mode or zones:
  [Configuration](configuration.md).
- Everything a client can see and call: [OPC UA interface](opcua-interface.md).
