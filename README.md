# planar-controller

Controls a Planar Motor table (PMC) and offers it over OPC UA, built like the lab's generated
IEC 61499 modules: the table and each xbot are modules with an occupation, a PackML module state
manager and skills. An orchestrator occupies a shuttle, runs its `MoveToPosition` skill as often
as it likes, and releases it. All xbots on the table move as one plan, so they get past each
other in narrow passages, and requests for the same place are served in the order they were made.

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
| `XBOT_SPEED`, `XBOT_ACCEL` | `0.5`, `2.0` | m/s, m/s² |
| `LOG_LEVEL` | `INFO` | |

The container uses host networking (the PMC's auto search needs it, and the OPC UA port is this
machine's). Docker Desktop on Windows and macOS needs host networking switched on; see the comment
in `docker-compose.yml` for the alternative. The image is healthy while its OPC UA server takes
connections.

Without Docker (Python 3.11, and mono on Linux for pmclib):

```bash
pip install -r requirements.txt dependencies/pmclib-117.9.1-py3-none-any.whl
PMC_IP=127.0.0.1 AUTOSTART=1 python -m planar     # see --help
```

On a Windows PC with OPC UA tools installed, port 4840 is often taken by the OPC UA Local
Discovery Server and 4841 by UaExpert: set `OPCUA_ENDPOINT=opc.tcp://127.0.0.1:4850/`.

## Modules and their OPC UA interface

Namespace `urn:aau:smartproductionlab:planar`. Modules: `PlanarTable` and `PlanarShuttle<N>`
(one per xbot).

| Browse path below `/Objects/<Module>` | |
| --- | --- |
| `Occupation/Occupy(Session)`, `Release(Session)`, `Occupied` | the session is a UUID the client picks |
| `Module/Reset`, `Start`, `Stop`, `Abort`, `Clear` `(Session)`; `State`, `ErrorID` | |
| `Skills/MoveToPosition/Start(Session, X, Y, Rz)`, `Stop`, `Abort`, `Reset` `(Session)` | shuttles only; mm and degrees |
| `Skills/MoveToPosition/State`, `ErrorID`, `Parameters/X,Y,Rz`, `Results/X,Y,Rz` | results: where it ended |
| `Equipment/Xbot/X`, `Y`, `Rz`, `XbotState` | shuttles: the live pose |
| `Equipment/PMC/Connected`, `Columns`, `Rows`, `FlywaySize`, `Holes` | the table: the PMC's connection and the layout it reports (flyways of `FlywaySize` mm in `Columns` by `Rows`, without the `Holes`, given as `"<column>,<row>"` apart by spaces; 0 and empty until connected) |
| `Equipment/LoadingQueue` | the table: the free shuttles waiting at the loading station (and turned to `IdleRz`), the one on it first, apart by spaces (e.g. `"PlanarShuttle3 PlanarShuttle1"`): the one to occupy for the next product; empty without `LOADING_STATION` |
| `Equipment/IdleRz` | the table, writable: the orientation in degrees free shuttles are turned to, a multiple of 90; NaN: as they are. Another value is not taken and reads back as it was. Starts as `IDLE_RZ` |
| `Equipment/Mode`, `Equipment/LineLayout` | the table: `free` or `line`, and in line mode the layout's JSON as it was read (empty in free mode) |

Every method answers `[Accepted, ErrorID]` at once. An accepted `MoveToPosition` finishes when
the xbot has arrived: its `State` turns Succeeded (3), and Failed (4) with an `ErrorID` if it
cannot get there.

**The skill** (as SKILL_Core): Idle 0, Running 1, Stopping 2, Succeeded 3, Failed 4, Aborted 5.
Start from Idle, Succeeded or Failed; Succeeded returns to Idle after 1.5 s; Stop → Stopping →
Failed with Interrupted (the xbot finishes the slot it is crossing); Abort → Aborted at once (the
xbot stops where it is); Reset only from Aborted. The gate, in order: the caller holds the
occupation (5), the shuttle and the table are in Execute (4), the skill is not running (6), the
parameters are in range (8), then the PMC, the xbot and the target (21, 25 to 29 below).

**The modules** (PackML numbers): Stopped 2 -Reset-> Resetting 15 -> Idle 4 -Start-> Execute 6;
Stop → Stopping 7 → Stopped; Abort → Aborted 9 -Clear-> Stopped. The table's Resetting connects
to the PMC, takes mastership, levitates the xbots and, with `LOADING_STATION`, sends every shuttle
into the loading queue with the PMC's auto routing (one command, up to 10 xbots; with more, or if
the PMC refuses, the planner lines them up during the Resetting). The auto routing does not turn
the xbots, so with `IdleRz` set the Resetting then turns each shuttle to it on a flyway centre
before the table is Idle. A Stop meanwhile stops them. A shuttle's Resetting checks that its
xbot is there.
The table's Stop and Abort reach every shuttle, its Reset, Start and Clear the shuttles in the
matching state, so one Reset and one Start of the table bring everything up. Only the occupant
may command an occupied module; Abort is taken from anyone. A lost PMC connection aborts the
table with ErrorID 27; Clear, Reset and Start bring it back.

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
| 28 | XbotNotDetected | the PMC does not report this xbot (missing or landed) |
| 29 | XbotCrooked | the xbot is turned off the axes away from a flyway centre and cannot travel |
| 30 | MotionDeviated | a move ended somewhere else than it was sent |
| 31 | LineLayoutInvalid | line mode: the table's Resetting found that the line layout does not fit the table (the log says why) |

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
  50 ms and takes the one estimated to be done soonest, the way it is executed: every corner and
  every wait for another xbot costs a stop and a start, so a straight route wins over a
  staircase and a detour that saves waiting can win over the shortest way.
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
  that was there has fully left it, so a slow or late xbot cannot cause a collision.
- The PMC rotates an xbot only on a flyway centre. A turn claims a whole flyway, waits until the
  others have left it, drives to its centre, turns and drives out. Several claims may wait at
  once, as long as every xbot that has to clear out of one can still get out of all of them; a
  claim that has waited 5 s gives way to an older one still waiting. An xbot travels only square
  to the axes; another angle is possible only as the final pose on a flyway centre.
- Two xbots off the slots may stand side by side and touch (sharing slots): a final move, a turn
  or a step back onto a slot is checked against the other xbots' real squares. Into a gap between
  two xbots it touches on both sides it comes from one xbot width beside the gap, straight in,
  and leaves the same way.
- An unoccupied xbot left off the slots steps back onto the nearest one.

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
Iv#+##^T     T        a turn spot's flyway
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
- **turnSpots**: points in the flyways xbots turn on (a turn claims the whole flyway, so a turn
  spot on a lane blocks it while the xbot turns); none: any flyway. A target turned off the axes
  is still turned on its own flyway.
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
- `LINE_LAYOUT` is read once at start-up. From a URL the controller waits for the server while it
  cannot be reached; a layout that is not valid JSON, or not a layout, stops it with the reason.
  A JSON string holding the layout is read too, so it can be the `$value` of an AAS `Property`,
  or the content of an AAS `File` element (its `/attachment`). `config/line-layout.example.json`
  is the layout above; `config/` is mounted into the container.
- The lab's AAS Web UI draws it: its Configurator (Production lines, Shuttle routing) lays out the
  lanes, turn spots and idle area on the table as the table's AAS reports it, checks them as the
  controller does, keeps them in the line's AAS (`Parameters/PlanarRouting/Value`) and shows the
  `LINE_LAYOUT` URL to set.

## Development

| | |
| --- | --- |
| `planar/statemachines.py` | occupation, module state manager, skill core |
| `planar/modules.py` | the table and shuttle modules on the coordinator |
| `planar/coordinator.py` | turns tasks into plans and plans into motions |
| `planar/mapf.py` | the multi-xbot planner |
| `planar/lines.py` | line mode: the layout, its slot graph and who may use which slots |
| `planar/layout.py` | the table: flyways, holes, slots, footprints, boxes |
| `planar/pmc_driver.py`, `planar/sim_driver.py` | pmclib, and a kinematic stand-in for tests |
| `planar/opcua_server.py` | the OPC UA address space |
| `planar/__main__.py` | settings, start-up, autostart |

```bash
pip install -r requirements.txt dependencies/pmclib-117.9.1-py3-none-any.whl pytest
python -m pytest tests                    # no PMC needed; about 3 minutes
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

On a Windows PC with Planar Motor's simulation tool installed, `tools/pmc-sim.ps1 start|stop|restart|status`
runs its PMC (`Dlls\VirtualPMC.exe`) without the tool's window, for `PMC_IP=127.0.0.1`. The tool is
not commercially available and not in this repository; the script runs it from where it is
installed (or `PMC_SIM_DIR`, or a copy in `pmc-sim/`, which git and Docker ignore).

pmclib is Planar Motor's and not on PyPI; the wheel is in `dependencies/`. 117.9.1 works with PMC
firmware 117.17.35 and later. 117.15.01 (May 2026) needs firmware 117.26.45 and adds nothing this
controller uses (zone and sector activation options, failed flyway recovery, xbot weighing).
