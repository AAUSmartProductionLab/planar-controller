# planar-controller

Controls a Planar Motor table (PMC) and offers it over OPC UA, built like the lab's generated
IEC 61499 modules: the table and each xbot are modules with an occupation, a PackML module state
manager and skills. An orchestrator occupies a shuttle, runs its `MoveToPosition` skill as often
as it likes, and releases it. All xbots on the table move as one plan, so they get past each
other in narrow passages, and requests for the same place are served in the order they were made.

The documentation is in [`mkdocs/docs/`](mkdocs/docs/index.md), for a MkDocs or Read the Docs site:
[getting started](mkdocs/docs/getting-started.md), [configuration](mkdocs/docs/configuration.md),
[the OPC UA interface](mkdocs/docs/opcua-interface.md), [examples](mkdocs/docs/examples.md),
[the planner](mkdocs/docs/planner.md), [line mode](mkdocs/docs/line-mode.md),
[speed and payload](mkdocs/docs/speed-and-payload.md), [zones](mkdocs/docs/zones.md),
[architecture](mkdocs/docs/architecture.md), [testing](mkdocs/docs/testing.md) and
[troubleshooting](mkdocs/docs/troubleshooting.md). This README is the short reference.

## Install

On a Linux machine that can reach the PMC (it installs Docker if it is missing, builds the image
and starts the controller; it restarts with the machine):

```bash
curl -fsSL https://raw.githubusercontent.com/AAUSmartProductionLab/planar-controller/main/install.sh \
  | bash -s -- --pmc-ip 192.168.10.100 --autostart
```

or from a clone:

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
| `--loading-station X,Y` | in mm: free xbots queue up there, and a Reset sends them there (`LOADING_STATION`) |
| `--idle-rz DEG` | a multiple of 90: free xbots are turned to it, and a Reset turns them all (`IDLE_RZ`) |
| `--mode free\|line`, `--line-layout FILE\|URL` | anywhere on the table, or only on the lanes of a line layout (`MODE`, `LINE_LAYOUT`; see [Line mode](#line-mode)) |
| `--autostart` / `--no-autostart` | Reset and Start the table when the controller starts, or leave it Stopped (the default) |
| `--dir DIR` | where to clone it when not run from a clone (`~/planar-controller`) |
| `--branch NAME` | the branch to install (`main`) |
| `--no-start` | install and build only |

Run it again to update: it pulls, rebuilds and restarts, and keeps `.env` (options change it).

## Run

The controller runs with Docker Compose from the repository folder; its settings are in `.env`
(`install.sh` makes it from `.env.example`):

```bash
docker compose up -d --build     # build and start
docker compose logs -f           # follow the log
docker compose up -d             # after changing .env
docker compose down              # stop
```

| `.env` | Default | |
| --- | --- | --- |
| `PMC_IP` | empty: search | the PMC (the simulation tool: `127.0.0.1`) |
| `OPCUA_PORT` | `4840` | the server is `opc.tcp://<this machine>:<port>/` |
| `SHUTTLES` | the ids the PMC knows | e.g. `1,2,3` |
| `AUTOSTART` | `0` | `1`: Reset and Start the table at start-up, and keep trying while the PMC is not there yet |
| `DISPLACE_HELD` | `0` | `1`: the planner may push occupied shuttles that are not moving |
| `BLOCKED_TIMEOUT_S` | `0` | seconds a move may wait without getting closer before it fails with Blocked (20); `0`: it waits until the way is free or it is stopped |
| `LOADING_STATION` | empty | `X,Y` in mm, e.g. `60,180`: the xbots nobody holds queue up there, and the table's Reset sends them there (see below); empty: an xbot nobody holds stays where it was |
| `IDLE_RZ` | empty | degrees, a multiple of 90, e.g. `0`: the xbots nobody holds are turned to it, and the table's Reset turns every shuttle to it; empty: as they are. Also `PlanarTable/Equipment/IdleRz` over OPC UA |
| `MODE` | `free` | `free`: the xbots may travel anywhere on the table; `line`: only on the lanes of `LINE_LAYOUT` (see [Line mode](#line-mode)) |
| `LINE_LAYOUT` | empty | line mode: the layout, a file in `config/` (e.g. `config/line-layout.json`) or an `http(s)://` URL to fetch it from at start-up |
| `ZONES` | empty | zones of the table to switch off for putting xbots on or taking them off, or to slow down in: a file in `config/` (see `config/zones.example.json`) or URL (see [Zones](#zones)) |
| `XBOT_SPEED`, `XBOT_ACCEL`, `XBOT_JERK` | `0.5`, `2.0`, empty | m/s and m/s² every xbot moves at, or slower (see [Speed](#speed); `PlanarTable/Equipment/SetSpeed` changes them over OPC UA), and m/s³ (empty: the PMC's) |
| `LOG_LEVEL` | `INFO` | |

The container uses host networking (the PMC's auto search needs it, and the OPC UA port is this
machine's). Docker Desktop on Windows and macOS needs host networking switched on; see the comment
in `docker-compose.yml` for the alternative. The image is healthy while its OPC UA server takes
connections.

Without Docker (Python 3.11 or later, and mono on Linux for pmclib):

```bash
pip install -r requirements.txt dependencies/pmclib-117.9.1-py3-none-any.whl
PMC_IP=127.0.0.1 AUTOSTART=1 python -m planar     # see --help
```

On a Windows PC with OPC UA tools installed, port 4840 is often taken by the OPC UA Local
Discovery Server and 4841 by UaExpert: set `OPCUA_ENDPOINT=opc.tcp://127.0.0.1:4850/`.

## A first move

The existing planner remains the default (`PLANNER=classic`). To try the experimental
rounded-corner executor, set `PLANNER=blended` in `.env` and recreate the container, or run
`python -m planar --planner blended` with your usual connection options. Restart with
`--planner classic` to switch back. Both use the same OPC UA interface and route search;
the experimental coordinator lives in `planar/coordinator_blended.py`. It blends one clear
XY corner at a time in free mode; line mode and constrained corners keep the old behavior.
See [experimental blended execution](mkdocs/docs/planner.md#experimental-blended-execution).

With [asyncua](https://github.com/FreeOpcUa/opcua-asyncio): bring the table up, occupy a
shuttle, move it, release it. Each call answers `[True, 0]` when it is accepted.

```python
import asyncio
import uuid

from asyncua import Client, ua


async def main():
    async with Client("opc.tcp://127.0.0.1:4840/") as client:
        ns = await client.get_namespace_index("urn:aau:smartproductionlab:planar")

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

        table = await node("PlanarTable", "Module")
        print("Reset:", await table.call_method(f"{ns}:Reset", text("")))  # nobody occupies the table
        await wait(("PlanarTable", "Module", "State"), (4,))                # Idle
        print("Start:", await table.call_method(f"{ns}:Start", text("")))
        await wait(("PlanarTable", "Module", "State"), (6,))                # Execute

        session = str(uuid.uuid4())
        occupation = await node("PlanarShuttle1", "Occupation")
        print("Occupy:", await occupation.call_method(f"{ns}:Occupy", text(session)))
        skill = await node("PlanarShuttle1", "Skills", "MoveToPosition")
        print("Start:", await skill.call_method(f"{ns}:Start", text(session), number(780), number(660), number(90)))
        print("State:", await wait(("PlanarShuttle1", "Skills", "MoveToPosition", "State"), (3, 4)))  # 3: arrived
        print("Release:", await occupation.call_method(f"{ns}:Release", text(session)))


asyncio.run(main())
```

More in [the examples](mkdocs/docs/examples.md): the loading queue, a product agent, payload and
speed, Hold, Stop and Abort, recovering after a fault, zones, subscriptions.

## Modules and their OPC UA interface

Namespace `urn:aau:smartproductionlab:planar`. Modules: `PlanarTable` and `PlanarShuttle<N>`
(one per xbot).

| Browse path below `/Objects/<Module>` | |
| --- | --- |
| `Occupation/Occupy(Session)`, `Release(Session)`, `Occupied` | the session is a UUID the client picks |
| `Module/Reset`, `Start`, `Hold`, `Unhold`, `Stop`, `Abort`, `Clear` `(Session)`; `State`, `ErrorID` | |
| `Skills/MoveToPosition/Start(Session, X, Y, Rz)`, `Stop`, `Abort`, `Reset` `(Session)` | shuttles only; mm and degrees |
| `Skills/MoveToPosition/State`, `ErrorID`, `Parameters/X,Y,Rz`, `Results/X,Y,Rz` | results: where it ended |
| `Equipment/Xbot/X`, `Y`, `Rz`, `XbotState` | shuttles: the live pose (mm, degrees in (-180, 180]); NaN until the PMC has been read, at the first Reset. `XbotState` is the PMC's (`XBOT_IDLE`, ...), `ACCIDENT (...)` when the PMC switched it off after an accident; `UNKNOWN` until the first Reset |
| `Equipment/Xbot/SetPayload(Session, PayloadKg, CgHeight)` | shuttles, the occupant's, not while its skill runs: tell the PMC what the xbot carries (kg, its centre of gravity mm above the xbot); see [Payload](#payload) |
| `Equipment/Xbot/PayloadKg`, `PayloadCgHeight`, `MaxAcceleration` | shuttles: the payload as the PMC was told it or has it (read at every Reset), and the acceleration the PMC allows with it (m/s²), which its moves keep to; NaN while unknown |
| `Equipment/Xbot/SetSpeed(Session, Speed, Acceleration)`; `Speed`, `Acceleration` | shuttles: keep the xbot slower than the table (m/s, m/s²; 0 or NaN: no limit of its own), whoever may command the shuttle; and how fast its moves are (NaN until the first Reset). See [Speed](#speed) |
| `Equipment/PMC/Connected`, `Columns`, `Rows`, `FlywaySize`, `Holes` | the table: the PMC's connection and the layout it reports (flyways of `FlywaySize` mm in `Columns` by `Rows`, without the `Holes`, given as `"<column>,<row>"` apart by spaces; 0 and empty until connected) |
| `Equipment/PMC/Flyways/Flyway_<c>_<r>/PowerConsumption`, `CpuTemperature`, `AmplifierTemperature`, `MotorTemperature` | the table: each flyway's power (W) and temperatures (°C), read every 5 s; added at the first Reset, NaN until read (the simulator reports 0) |
| `Equipment/Zones/<name>/State`, `Xbots`; `Deactivate`, `Activate` `(Session)` | the table, with `ZONES`: the PMC's state of the zone and the xbots inside (ids apart by spaces); switch it off and on (see [Zones](#zones)) |
| `Equipment/ZoneProblem` | the table: why the last Reset did not define the zones; empty when it did |
| `Equipment/LoadingQueue` | the table: the free shuttles waiting at the loading station (and turned to `IdleRz`), the one on it first, apart by spaces (e.g. `"PlanarShuttle3 PlanarShuttle1"`): the one to occupy for the next product; empty without `LOADING_STATION` |
| `Equipment/IdleRz` | the table, writable: the orientation in degrees free shuttles are turned to, a multiple of 90; NaN: as they are. Another value is not taken and reads back as it was. Starts as `IDLE_RZ` |
| `Equipment/SetSpeed(Session, Speed, Acceleration)`; `Speed`, `Acceleration` | the table: how fast every xbot moves at most (m/s, m/s²; 0 or NaN: `XBOT_SPEED`, `XBOT_ACCEL` again), whoever may command the table; and what it is now. See [Speed](#speed) |
| `Equipment/Mode`, `Equipment/LineLayout` | the table: `free` or `line`, and in line mode the layout's JSON as it was read (empty in free mode) |
| `Equipment/LineLayoutProblem` | the table, line mode: why the last Reset did not take the line layout as it is now (it is no layout, does not fit the table, or could not be read again and the one before is used); empty when it did |

Every method answers `[Accepted, ErrorID]` at once. An accepted `MoveToPosition` finishes when
the xbot has arrived: its `State` turns Succeeded (3), and Failed (4) with an `ErrorID` if it
cannot get there.

**The skill** (as SKILL_Core): Idle 0, Running 1, Stopping 2, Succeeded 3, Failed 4, Aborted 5.
Start from Idle, Succeeded or Failed; Succeeded returns to Idle after 1.5 s; Stop → Stopping →
Failed with Interrupted (the xbot ends the motion it is on, a straight run that may cross several
slots, and stays there); Abort → Aborted at once (the
xbot stops where it is); Reset only from Aborted. The gate, in order: the caller holds the
occupation (5), the shuttle and the table are in Execute (4), the skill is not running (6), the
parameters are in range (8), then the PMC, the xbot and the target (6, 21, 25 to 29 and 33
below).

**The modules** (PackML numbers): Stopped 2 -Reset-> Resetting 15 -> Idle 4 -Start-> Execute 6;
Execute -Hold-> Holding 10 -> Held 11 -Unhold-> Unholding 12 -> Execute; Stop → Stopping 7 →
Stopped; Abort → Aborted 9 -Clear-> Stopped. Only the occupant may command an occupied module;
Abort is taken from anyone.

- The table's Stop and Abort reach every shuttle, its Reset, Start, Hold, Unhold and Clear the
  shuttles in the matching state, so one Reset and one Start of the table bring everything up.
- The table's Resetting connects to the PMC, takes mastership, recovers the xbots the PMC switched
  off after an accident, levitates the xbots and, with `LOADING_STATION`, sends every shuttle into
  the loading queue with the PMC's auto routing (one command, up to 10 xbots; with more, or if the
  PMC refuses, the planner lines them up during the Resetting). The auto routing does not turn the
  xbots, so with `IdleRz` set the Resetting then turns each shuttle to it on a flyway centre before
  the table is Idle. A Stop meanwhile stops them.
- A shuttle's Resetting recovers its xbot if the PMC switched it off after an accident (and waits
  up to 15 s for it to levitate), and checks that its xbot is there.
- Hold pauses the xbots where they are, in the middle of a move or a turn (the table's: every
  xbot; a shuttle's: its own): the PMC pauses the ones moving, and none is sent anything more.
  Their skills stay Running and nothing times out while they are held; a skill Start is refused
  with NotReady (4), and a skill Stop takes effect once they are unheld. Unhold carries on with
  the same moves. A shuttle held on its own is unheld only while the table is in Execute, and the
  table's Unhold unholds its held shuttles too. A Stop while held lets the moves end first; an
  Abort stops them at once.
- A lost PMC connection aborts the table with ErrorID 27, and another program taking mastership
  of the PMC (PMT, another controller) with 32, within a second; Clear, Reset and Start bring it
  back (the Reset takes mastership again).

**ErrorID.** A refused method answers `[False, ErrorID]`; a skill or procedure that fails shows
its ErrorID in `ErrorID` next to its `State`.

| ErrorID | Name | When |
| --- | --- | --- |
| 0 | None | |
| 4 | NotReady | the module or skill is not in a state that takes the command (a skill start while the shuttle or the table is not in Execute, a skill Reset that is not Aborted, ...) |
| 5 | NotPermitted | the caller does not hold the occupation: occupying one another session holds, releasing or commanding one it does not hold |
| 6 | Busy | the skill is running, or the xbot is already on its way |
| 7 | Interrupted | stopped (skill Stop, module or table Stop) before it arrived |
| 8 | OutOfRange | a parameter outside its limits (X, Y 0..10000 mm, Rz -180..180 deg), or an empty session |
| 20 | Blocked | no way to the target within `BLOCKED_TIMEOUT_S` (only when that is set) |
| 21 | XbotFault | the PMC reports the xbot in error, obstacle detected or stopped |
| 22 | PmcError | the PMC refused a command |
| 23 | MotionTimeout | a move did not arrive in time |
| 24 | ProcedureFailed | Resetting or Stopping did not complete |
| 25 | NotOnTable | the target pose is not entirely on flyways (off the edge or over a hole) |
| 26 | AngleNotAllowed | an Rz that is not a multiple of 90 deg, away from a flyway centre |
| 27 | PmcNotConnected | no connection to the PMC (never made, or lost) |
| 28 | XbotNotDetected | the PMC does not report this xbot, or reports it switched off (missing, landed, in a zone switched off, after an accident) |
| 29 | XbotCrooked | the xbot is turned off the axes away from a flyway centre and cannot travel |
| 30 | MotionDeviated | a move ended somewhere else than it was sent |
| 31 | LineLayoutInvalid | line mode: the table's Resetting found that the line layout, as read again, is no layout or does not fit the table (`Equipment/LineLayoutProblem` says why) |
| 32 | MastershipLost | another program (PMT, another controller) has taken mastership of the PMC, or kept it at the Reset |
| 33 | ZoneClosed | the target is in a zone that is switched off or fenced |
| 34 | ZonesInvalid | the table's Resetting found a zone of `ZONES` that is not on the table, or the PMC would not define it (`Equipment/ZoneProblem` says why) |

Besides, `SetPayload` answers OutOfRange (8) for a payload the xbot does not take (the
simulator's M3-06 takes at most 0.5 kg), Busy (6) while the xbot moves or is held; `SetSpeed`
answers OutOfRange for a negative speed or acceleration, and on a shuttle XbotNotDetected (28)
before the table's first Reset; `Deactivate` answers Busy while an xbot is on its way into, out
of or across the zone, or stands across its edge, and NotReady (4) when the zone is not switched
on.

**After a failure.** When a move fails because of its xbot (21, 22, 23, 28, 29, 30), the PMC
stops the xbot and the shuttle refuses every Start with that ErrorID until it is Stopped, Reset
and Started again; its Resetting checks the xbot afresh. The occupant keeps the shuttle
throughout, and meanwhile the others go around the xbot. Blocked (20) and Interrupted (7) leave
the xbot fit for the next Start.

A typical run: `PlanarTable/Module/Reset`, wait for Idle, `Start`; then per product
`PlanarShuttle1/Occupation/Occupy(id)`, `Skills/MoveToPosition/Start(id, x, y, rz)`, wait for
State 3, ... , `Release(id)`.

## How the xbots move

- The planner works on slots one xbot wide (120 mm, 2 × 2 per flyway). An xbot on a slot centre
  covers that slot; anywhere else (a station pose between slots, a flyway centre) it covers every
  slot it overlaps, and others go around.
- A move whose straight line is free (nothing in the box it sweeps, at most 16 slots) is one
  straight move, slots or not (not in line mode: there it keeps to the lanes).
- Otherwise all xbots move as one plan (`planar/mapf.py`: LaCAM with PIBT). It finds a way
  whenever there is one, including two xbots passing each other in a one-lane passage by way of
  a pocket, and pushes idle xbots out of the way. Once it has a plan it tries more for up to
  50 ms and takes the one estimated to be done soonest, the way it is executed (each xbot at its
  own speed): every corner and every wait for another xbot costs a stop and a start, so a
  straight route wins over a staircase and a detour that saves waiting can win over the shortest
  way.
- A new plan is made as soon as anything changes (a request, an xbot taken or let go, a claim),
  without stopping the others: an xbot on its way is planned on from where its motion ends, and
  nobody enters a slot it has still to leave until it has left it.
- Idle xbots nobody holds go back to where they were, or with `LOADING_STATION` queue up there:
  the first on the slot under the station, the others on the slots nearest to it, along the
  table's edge rather than into the table. When the first is taken, the queue closes up with as
  few moves as it can. With `IdleRz` set, an idle xbot nobody holds that is turned otherwise
  (released after a turned station, or `IdleRz` changed) turns to it on the nearest clear flyway;
  a request for it replaces the turn at once.
- An xbot whose shuttle is occupied and not moving is never pushed (a station may be working on
  it), nor one whose shuttle is not in Idle or Execute. If such xbots wall a target off, the move
  waits until they leave (or fails with Blocked after `BLOCKED_TIMEOUT_S`, if that is set).
- Requests for the same place are served in the order they were made: a younger one waits, on a
  slot where the planner can move it aside, until the older one has arrived there and left again.
- Plans are executed by dependencies, not by time: an xbot starts into a slot only once the xbot
  that was there has fully left it, so a slow or late xbot cannot cause a collision. Nor is an
  xbot sent into the way another is still travelling (the slots that one has left on its current
  straight move, and one behind them) while that one has more than 0.4 s to go: Planar Motor's
  simulator stops it there with `XBOT_OBSTACLE_DETECTED` while the move ahead has more than
  about 0.65 s to go, though that xbot is well ahead.
- The PMC rotates an xbot only on a flyway centre. A turn claims a whole flyway, waits until the
  others have left it, drives to its centre, turns the shortest way (by the difference from its
  angle: to an absolute angle the PMC, which does not wrap angles, would turn from -90 to 180 deg
  three quarters round) and drives out. Several claims may wait at
  once, as long as every xbot that has to clear out of one can still get out of all of them; a
  claim that has waited 5 s gives way to an older one still waiting. An xbot travels only square
  to the axes; another angle is possible only as the final pose on a flyway centre.
- Two xbots off the slots may stand side by side and touch (sharing slots): a final move, a turn
  or a step back onto a slot is checked against the other xbots' real squares. Into a gap between
  two xbots it touches on both sides it comes from one xbot width beside the gap, straight in,
  and leaves the same way.
- An unoccupied xbot left off the slots steps back onto the nearest one.

## Speed

Every xbot moves at the table's speed and acceleration, or slower:

- `PlanarTable/Equipment/SetSpeed(Session, Speed, Acceleration)` sets the table's, in m/s and
  m/s² (`Speed` and `Acceleration` beside it read them). They start as `XBOT_SPEED` and
  `XBOT_ACCEL`; 0 or NaN sets one back to that.
- `PlanarShuttle<N>/Equipment/Xbot/SetSpeed(Session, Speed, Acceleration)` keeps one xbot slower,
  e.g. while it carries an open vial: it moves at the lower of its own and the table's; 0 or NaN:
  no limit of its own. The limits stay until they are set again (or the controller restarts),
  through releases and Resets, so set them back to 0 when the product leaves the shuttle.
- A payload the PMC allows less with lowers its acceleration further (see [Payload](#payload)),
  and a zone's `speed` scales both inside the zone (see [Zones](#zones)). The shuttle's `Speed`
  and `Acceleration` read what its moves are made with, apart from zones.

Whoever may command the module sets its speed: its occupant, or anyone while it is not occupied.
A motion under way keeps the speed it was sent with; the new one counts from the xbot's next
motion (a long straight move goes on as it was). Turns keep their own speed. The PMC takes any
speed but moves no xbot faster than it can go: in Planar Motor's simulator about 3 m/s, whether
4, 6 or 10 m/s is set. `XBOT_JERK` limits the jerk of every xbot, smoothing their starts and
stops.

## Payload

A station that puts something on a shuttle (filling, a part) tells the PMC what it now carries:
`PlanarShuttle<N>/Equipment/Xbot/SetPayload(Session, PayloadKg, CgHeight)`, the shuttle's
occupant, while the shuttle is not moving. The PMC tunes its control to the payload and says how
fast the xbot may accelerate with it (`MaxAcceleration`); the xbot's moves then keep to that when
it is below their acceleration (see [Speed](#speed)), so a full vial is driven more gently. The
PMC keeps the payload until it is told another (it is read back at every Reset); set it to 0 when
the product leaves the shuttle.

The PMC refuses a payload over what the xbot takes (OutOfRange, 8). Planar Motor's simulator
takes the payload but does not keep it: it reads back 0 kg and `MaxAcceleration` 20 m/s² whatever
it was told, so only a real table slows a loaded xbot down.

## Zones

`ZONES` names rectangles of the table (a JSON file or URL, see `config/zones.example.json` and
`planar/zones.py`) that the PMC knows as zones, defined at every Reset of the table:

```json
{"zones": [
  {"name": "Service", "from": [720, 480], "to": [960, 720]},
  {"name": "Filling", "from": [240, 480], "to": [480, 720], "speed": 0.5}
]}
```

`from` and `to` are opposite corners in mm (best on flyway edges); `speed` (0 to 1, default 1)
scales the speed and acceleration of every xbot inside, e.g. near a station. Over OPC UA, each is
`PlanarTable/Equipment/Zones/<name>`:

- `Deactivate(Session)` switches it off: the PMC fences it and switches off the xbots inside
  (their shuttles read `XbotNotDetected`); an operator may then take them off the table or put
  new ones on, while the rest of the table goes on. Busy (6) while an xbot is on its way into,
  out of or across it, or stands across its edge.
- `Activate(Session)` switches it on again: the PMC levitates the xbots inside, then the
  controller removes the fence. An xbot put on that the controller did not know takes part (it
  stays where it is, as one not offered as a shuttle, until the controller starts again with it).
- `State` is the PMC's: `ACTIVATED_UNFENCED` (part of the table), `LOADING_FENCED` (switched off),
  `ACTIVATED_FENCED` (on, its fence not removed yet), ...; `Xbots`: the ones inside.

While a zone is not `ACTIVATED_UNFENCED` the planner keeps every xbot out of it (the PMC would
stop one at its fence with an obstacle), turns on no turn spot in it, and a target in it is
refused with ZoneClosed (33). Only the table's occupant may switch one, or anyone while the table
is not occupied.

## Line mode

With `MODE=line` the xbots travel only on lanes you lay out, so the way between two places is
always the same one, and they turn only on the turn spots. The layout is JSON, in mm in the
table's coordinates (the same as the skill's X and Y; slot centres are at 60, 180, 300, ...):

```json
{
  "lanes": [
    {"name": "loop", "points": [[180, 180], [780, 180], [780, 540], [180, 540], [180, 180]], "oneWay": true},
    {"name": "shortcut", "points": [[420, 180], [420, 540]]}
  ],
  "turnSpots": [[840, 360]],
  "idleArea": [[60, 180], [60, 540]]
}
```

On the 4 x 3 table that is (the controller logs it like this; top row first):

```
########     > < ^ v  a one-way lane, the way on
Iv<+<<<#     +        a two-way lane, or where lanes cross
Iv#+##^T     I        the idle area
Iv#+##^T     T        what a turn on a turn spot sweeps
I>>+>>^#     #        a slot of no lane
########
```

- **lanes**: each a line through slot centres, every stretch along a row or a column. A one-way
  lane (`"oneWay": true`) is driven from its first point to its last, the others both ways. Where
  lanes meet or cross an xbot may change from one to the other; between two lane slots side by
  side there is no way but along a lane. The lanes together must lead from every lane slot to
  every other (with one-way lanes too), or the table's Reset fails with ErrorID 31.
- **Targets off the lanes** (a station beside them) are reached from the nearest lane slot, the
  shortest way across slots of no lane, and left the same way; only the xbot going there or
  coming from there drives on those slots. Keep stations off the lanes: a shuttle held at a
  station on a lane blocks the lane until it is released.
- **turnSpots**: the points xbots turn on, each a flyway's centre or a point where four flyways
  meet, e.g. `[720, 480]` (the PMC turns an xbot nowhere else; another point in a flyway stands
  for its centre). A turn claims the square one flyway wide around the spot, so a turn spot on a
  lane blocks it while the xbot turns, and it must have a lane slot under it. An xbot turns on the
  turn spot nearest to it along the lanes and, while another turns there, waits behind it rather
  than drive on to another. None: any flyway centre, whichever is free. A target turned off the
  axes is still turned on its own spot.
- **idleArea**: a line like a lane's: the loading queue in order, its first point the loading
  position (`LOADING_STATION` is not used). The xbots nobody holds wait there, reached and left
  like a target off the lanes. Without it, an idle xbot goes back to where it was. Give it a slot
  for every xbot on the table: a free xbot that does not fit waits where it is, and on a one-way
  lane that soon blocks the others (the controller warns at the Reset when it is too small).
- An idle xbot nobody holds is pushed only along the lanes; one waiting for its turn spot or
  station to clear stays where it is (for up to 5 s, then it lets go of its claim for a while, so
  the ones in its way can get out).
- Laying out lanes: xbots on a lane one slot wide cannot pass each other, so two-way lanes jam
  as soon as several xbots go opposite ways on them (in the simulated tests, 8 xbots on a single
  two-way lane mostly do). Prefer one-way loops, with two-way lanes only as short branches, and
  put turn spots where a waiting xbot does not stand in the others' way.
- The table's Reset still sends the shuttles into the queue with the PMC's auto routing, which
  knows nothing of the lanes; the lanes apply from then on.
- `LINE_LAYOUT` is read at start-up, and again at every Reset of the table: a layout changed
  (in the AAS, or the file) is taken by pressing Reset, without restarting the controller. At
  start-up from a URL the controller waits for the server while it cannot be reached; a layout
  that is not valid JSON, or not a layout, stops it with the reason. At a Reset such a layout,
  or one that does not fit the table, fails the Reset with ErrorID 31 and the reason in
  `Equipment/LineLayoutProblem`; a server that cannot be reached leaves the one read before.
  `config/line-layout.example.json` is the layout above; `config/` is mounted into the container.
- **Layouts to test with** are in [`config/layouts/`](config/layouts/), each with stations for
  the product agents of `tests/line_scenario.py` and the controller's settings for it
  (`controller.env`). `pytest` runs the agents on each on the simulated table, and
  `python tests/line_scenario.py --layout config/layouts/<name>` against a running controller
  (e.g. on Planar Motor's simulator), checking that the shuttles keep to the lanes' way and turn
  only on turn spots.
- **From an AAS**: the layout is a `PlanarRouting` collection in the line's `Parameters`, and
  `LINE_LAYOUT` is its `$value` (`…/submodels/<Parameters>/submodel-elements/PlanarRouting/$value`):

  ```
  PlanarRouting       SubmodelElementCollection
    Lanes             SubmodelElementList, a collection per lane:
      Name            Property xs:string
      OneWay          Property xs:boolean
      Points          SubmodelElementList of points
    TurnSpots         SubmodelElementList of points
    IdleArea          SubmodelElementList of points
  a point             SubmodelElementCollection: X, Y, Properties xs:double in mm
  ```

  Its value is read as the repository gives it, e.g.
  `{"Lanes": [{"Name": "loop", "OneWay": "true", "Points": [{"X": "180", "Y": "180"}, …]}], "TurnSpots": […], "IdleArea": […]}`
  (BaSyx gives the values as text; numbers and `true`/`false` are read too). A JSON string
  holding the file's form is read as well (the `$value` of a `Property`, or an AAS `File`'s
  `/attachment`).
- The lab's AAS Web UI draws it: its Configurator (Production lines, Shuttle routing) lays out the
  lanes, turn spots and idle area on the table as the table's AAS reports it, checks them as the
  controller does, keeps them in the line's AAS as that `PlanarRouting` collection and shows the
  `LINE_LAYOUT` URL to set.

## Known limitations

- **The PMC's obstacle check**: the PMC stops an xbot sent into the way another is still
  travelling, with `XBOT_OBSTACLE_DETECTED`, by a rule Planar Motor does not publish. The
  controller keeps out of a moving xbot's trail as measured on the simulator, which makes such
  stops rare but not impossible; a stopped move fails with XbotFault (21) (see *After a
  failure* above).
- The controller assumes 120 mm xbots on 240 mm flyways. Xbots travel square to the axes, and
  turn on flyway centres and where four flyways meet.
- One controller per PMC: the PMC has one master. The OPC UA server has no security (policy
  None): keep it on a trusted network.
- In line mode, the table's Reset lines the shuttles up with the PMC's auto routing, which knows
  nothing of the lanes.
- pmclib cannot delete a zone: one removed from `ZONES` stays defined on the PMC until the PMC
  restarts. An xbot put on the table through a zone is offered as a shuttle only once the
  controller restarts.

## Development

| | |
| --- | --- |
| `planar/statemachines.py` | occupation, module state manager, skill core |
| `planar/modules.py` | the table and shuttle modules on the coordinator |
| `planar/coordinator.py` | turns tasks into plans and plans into motions |
| `planar/mapf.py` | the multi-xbot planner |
| `planar/lines.py` | line mode: the layout, its slot graph and who may use which slots |
| `planar/zones.py` | the zones of `ZONES` |
| `planar/layout.py` | the table: flyways, holes, slots, footprints, boxes |
| `planar/driver.py` | what the coordinator needs from a PMC; a driver for another kind of table implements its `connect`, `read`, `move`, `rotate` and `stop`, the rest is optional |
| `planar/pmc_driver.py`, `planar/sim_driver.py` | pmclib, and a kinematic stand-in for tests |
| `planar/opcua_server.py` | the OPC UA address space |
| `planar/__main__.py` | settings, start-up, autostart |
| `tests/harness.py` | the table and its shuttles on the stand-in, with the tests' own clock |

```bash
pip install -r requirements.txt dependencies/pmclib-117.9.1-py3-none-any.whl pytest
python -m pytest tests -m "not slow"      # no PMC needed; about half a minute
python -m pytest tests                    # and the production line in real time; about 5 minutes
python tests/stress_contention.py 40      # 480 runs: many xbots, one place, in order, no collisions
```

`tests/line_scenario.py` is a production line driven through OPC UA only: product agents, each
with its own client and session, occupy the first free shuttle in the loading queue (or whichever
is free without one), take it clockwise through
Infeed, Filling, Stoppering, Capping and Outfeed (stations at or near the edge of the 4 x 3
simulation table), send the product straight to the Outfeed when a step fails, and release the
shuttle there. `tests/test_line.py` runs it on the simulated driver; against a running
controller and the PMC (or its simulation tool):

```bash
python tests/line_scenario.py --endpoint opc.tcp://127.0.0.1:4840/ --agents 3 --products 3 --fail 0.2
```

`--hold-every S` holds the table every S seconds for 2 s meanwhile: nothing may move while it is
held, and the agents wait for the Unhold.

On a Windows PC with Planar Motor's simulation tool installed, `tools/pmc-sim.ps1 start|stop|restart|status`
runs its PMC (`Dlls\VirtualPMC.exe`) without the tool's window, for `PMC_IP=127.0.0.1`. The tool is
not commercially available and not in this repository; the script runs it from where it is
installed (or `PMC_SIM_DIR`, or a copy in `pmc-sim/`, which git and Docker ignore). While the
tool's window is open, an engine started by the script may take no connections: start the
simulation from the window then. `tools/sim_check.py` runs every layout of `config/layouts/`
against it, each with its own controller and the product agents, and prints a PASS or FAIL line
per layout:

```bash
python tools/sim_check.py [--start-sim] [--hold-every S] [--mastership]
```

`--mastership` then takes mastership from another session: the table must abort with
MastershipLost (32) and take it back at a Reset. The controller's own simulation does not turn
xbots as the PMC does (it once turned one three quarters round, unseen in the simulation), so run
`sim_check.py` after changing how xbots move.

**Disconnect from the PMC.** Every program that connects with pmclib must call
`disconnect_from_pmc()` before it ends. For each connection left open, the simulator writes an
error to its trace log (`%APPDATA%\Planar Motor Inc\Virtual PMC\data\PMTraceLog.txt`) on every
read, gigabytes an hour, until it takes no new connections. The controller disconnects when it
stops on Ctrl+C, Ctrl+Break or SIGTERM (`docker stop`); `sim_check.py` stops its controllers with
Ctrl+Break; one that is killed does not disconnect.

pmclib is Planar Motor's and not on PyPI; the wheel is in `dependencies/`. 117.9.1 works with PMC
firmware 117.17.35 and later. 117.15.01 (May 2026) needs firmware 117.26.45 and adds nothing this
controller uses (zone and sector activation options, failed flyway recovery, xbot weighing).

The documentation in `mkdocs/` builds with MkDocs:

```bash
pip install -r mkdocs/requirements.txt
mkdocs serve -f mkdocs/mkdocs.yml        # http://127.0.0.1:8000; mkdocs build -f mkdocs/mkdocs.yml for the site
```

## License

MIT License, Copyright (c) 2026 Aalborg University Smart Lab: see [LICENSE](LICENSE). pmclib, in
`dependencies/`, is Planar Motor's.
