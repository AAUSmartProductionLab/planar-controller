# Troubleshooting

The controller logs what it does, and why something failed: `docker compose logs -f`, or the
console without Docker. `LOG_LEVEL=DEBUG` adds why moves wait.

## The controller does not start

| It says | Why | What to do |
| --- | --- | --- |
| `OPC UA endpoint ...: its port is in use by another program` | Another program has the port: another controller, the OPC UA Local Discovery Server (4840 on Windows), UaExpert (4841). | Stop the other program, or set another port: `OPCUA_PORT` with Docker, `OPCUA_ENDPOINT` without. |
| `python -m planar: error: argument --...` | A setting is not valid. | Correct it; see [Configuration](configuration.md#what-is-checked-when-it-starts). |
| `waiting for the PMC to list its xbots (or set SHUTTLES)` | `SHUTTLES` is empty and the PMC cannot be reached. | Check `PMC_IP` and the network. In Docker, the auto search needs host networking. |
| `waiting for the line layout from ...` | The `LINE_LAYOUT` URL cannot be reached. | Check the AAS server and the URL. |
| `pmclib ... is not the one the controller is written for` | Another pmclib is installed. | `pip install dependencies/pmclib-117.9.1-py3-none-any.whl` |

## The table's Reset fails

The table returns to Stopped, with the reason in `PlanarTable/Module/ErrorID`:

| ErrorID | Why | What to do |
| --- | --- | --- |
| 27 PmcNotConnected | No PMC at `PMC_IP`, or none found. | Check that the PMC is on and reachable, and `PMC_IP`. |
| 32 MastershipLost | Another program, PMT or another controller, is the PMC's master. | Disconnect it, then Reset again. |
| 22 PmcError | The PMC refused something, e.g. it was not ready within 30 s. | The log says what; look at the PMC in PMT. |
| 31 LineLayoutInvalid | The line layout is no layout, or does not fit the table. | `Equipment/LineLayoutProblem` says why; see [Line mode](line-mode.md#what-a-layout-must-keep-to). |
| 34 ZonesInvalid | A zone is not on the table, or the PMC would not define it. | `Equipment/ZoneProblem` says why. |
| 21 XbotFault, 23 MotionTimeout | Lining up the shuttles at the loading station failed: an xbot faulted, or they were not in place within 120 s. | The log says which xbot. Is something on the table that the PMC does not know? |

A shuttle that stays Stopped after the table's Reset has its reason in its own `Module/ErrorID`:
usually XbotNotDetected (28), when its xbot is missing, landed or in a zone switched off.

## The table aborts while it runs

| ErrorID | Why | What to do |
| --- | --- | --- |
| 27 PmcNotConnected | The connection to the PMC was lost. | Clear, Reset, Start, once the PMC is back. |
| 32 MastershipLost | Another program took mastership of the PMC (PMT, another controller). | Disconnect it, then Clear, Reset, Start: the Reset takes mastership back. |
| other | A Hold or Unhold the PMC refused. | The log says why. Clear, Reset, Start. |

## A move is refused

`MoveToPosition/Start` answers `[false, ErrorID]`:

| ErrorID | Why | What to do |
| --- | --- | --- |
| 5 NotPermitted | The session does not hold the shuttle. | Occupy it first, with the same session. |
| 4 NotReady | The shuttle or the table is not in Execute, or is held; or the skill is Aborted. | Start the table; Unhold; Reset the skill. |
| 6 Busy | The skill is running. | Wait for it to end, or Stop it. |
| 8 OutOfRange | X, Y outside 0 to 10000 mm, or Rz outside -180 to 180 degrees. | |
| 25 NotOnTable | The xbot would not be entirely on flyways there. | A target's square must not reach off the edge or over a hole. |
| 26 AngleNotAllowed | An Rz that is not a multiple of 90 degrees, away from a spot. | Use a multiple of 90, or put the target on a flyway centre. |
| 33 ZoneClosed | The target is in a zone that is switched off. | Switch the zone on again. |
| 27, 28 | No PMC; the xbot is missing or switched off. | Reset the table; check the xbot. |
| 21, 22, 23, 28, 29, 30 | The xbot failed before and has not been reset since. | Stop, Reset and Start the shuttle ([example](examples.md#recover-a-shuttle-after-a-fault)). |
| 29 XbotCrooked | The xbot is turned off the axes away from a spot, and cannot travel. | Put it on a flyway centre, or square to the axes, by hand or with PMT. |

## A move fails

The skill turns Failed with an ErrorID:

| ErrorID | Why | What to do |
| --- | --- | --- |
| 21 XbotFault | The PMC put the xbot in a fault. Most often `XBOT_OBSTACLE_DETECTED`: its obstacle check stopped the xbot near another xbot's way (see [below](#known-limitations)). | Stop, Reset and Start the shuttle; the occupant keeps it. Then send the move again. |
| 23 MotionTimeout | A motion took half as long again as the PMC said, and 2 s more. | Was the xbot obstructed, or paused by another program? |
| 30 MotionDeviated | A motion ended somewhere other than where it was sent: something stopped it. | As for 21. |
| 28 XbotNotDetected | The xbot was switched off on its way: an accident, a zone switched off, landed. | Reset the shuttle; its Resetting recovers an xbot after an accident. |
| 22 PmcError | The PMC refused a motion. | The log says why. |
| 20 Blocked | No motion arrived within `BLOCKED_TIMEOUT_S`. | See the next section. |
| 7 Interrupted | The skill, the shuttle or the table was stopped. | |

## A move never ends

By default a move waits for its way to clear for as long as it takes. When the skill stays
Running, something walls off its target or its way:

- **Pinned shuttles**: an occupied shuttle that is not moving is never pushed. Is a shuttle
  standing on, or in front of, the target, occupied by an agent that forgot it?
- **Docked xbots**: xbots between slots are walls until they leave.
- **A closed zone** in the way.
- **An older request for the same place**: requests are served in order; this one waits
  until the older one has arrived and left again.
- **Line mode**: a jam on a two-way lane, or an idle area too small. See below.

With `LOG_LEVEL=DEBUG`, the log says which slot each xbot waits for: `xbot 3: waits, (4, 2) is
not free`. It also logs claims (`claims`, `gives way to an older claim`, `lets go of it for
now`). Set `BLOCKED_TIMEOUT_S` to have such moves fail with Blocked (20) instead.

## Shuttles jam in line mode

- **Two-way lanes**: xbots on a lane one slot wide cannot pass each other. Prefer one-way loops.
- **The idle area is too small**: a free xbot that does not fit waits where it is, on a lane in
  the way. At the Reset the log warns:
  `line mode: the idle area has 2 slots for 3 xbots; the free xbots that do not fit wait where they are, on a lane in the way`.
- **A station on a lane**: a shuttle held there blocks the lane. Put stations beside the lanes.
- **A turn spot over a lane**: it blocks the lane while an xbot turns there.

See [Laying out lanes](line-mode.md#laying-out-lanes).

## Planar Motor's simulator

### The simulator's trace log grows by gigabytes

A program connected with pmclib ended without `disconnect_from_pmc()`: a controller that was
killed, or a script that crashed. For each connection left open, the simulator writes an error to
`%APPDATA%\Planar Motor Inc\Virtual PMC\data\PMTraceLog.txt` on every read, gigabytes an hour,
until it takes no new connections. Restart the simulation, and stop controllers with Ctrl+C,
Ctrl+Break or SIGTERM, never by killing them.

### The engine takes no connections

While the simulation tool's window is open, an engine started by `tools/pmc-sim.ps1` may never
listen; start the simulation from the window then. Right after a disconnect, a new connection may
fail once with "Socket binding failed"; the next try works.

### A zone is left on the simulator

pmclib cannot delete a zone. Restart the simulation to clear zones a test defined.

## Docker

- **Docker Desktop** (Windows, macOS) needs host networking switched on in its settings. Or
  publish the OPC UA port instead, as a comment in `docker-compose.yml` describes, and set
  `PMC_IP`.
- **After changing `.env`**: `docker compose up -d` recreates the container with it.
- **The health check** only tells whether the OPC UA port takes connections. `PMC/Connected`
  says whether the PMC is connected.

## Known limitations

- **The PMC's obstacle check** is not published, and the controller's rule against it is
  measured on Planar Motor's simulator ([the trail rule](planner.md#keeping-out-of-another-xbots-trail)).
  It makes the stops rare, but not impossible: one was seen where a target only touched the
  corner of another xbot's start slot. A stopped move fails with XbotFault (21), and the shuttle
  must be Stopped, Reset and Started before it moves again.
- **Sizes**: the controller assumes 120 mm xbots and 240 mm flyways.
- **Motion**: xbots travel only square to the axes, and turn only on flyway centres and where four
  flyways meet.
- **One controller per PMC**: the PMC has one master.
- **No OPC UA security**: the server offers security policy *None* only. Keep it on a trusted
  network.
- **Line mode**: the table's Reset lines the shuttles up with the PMC's auto routing, which knows
  nothing of the lanes.
- **Zones** cannot be deleted from the PMC; one removed from `ZONES` stays defined until the PMC
  restarts.
- **New xbots**: an xbot put on the table through a zone is offered as a shuttle only once the
  controller restarts.
- **Planning holds the lock**: a plan takes at most 0.5 s, usually a few milliseconds, during
  which OPC UA method calls wait.
