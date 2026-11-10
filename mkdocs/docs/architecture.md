# Architecture

This page is for developers: how the code is laid out, how a request flows through it, and how to
extend it, e.g. with a driver for another kind of table.

## The package

| File | |
| --- | --- |
| `planar/__main__.py` | The settings (environment and options), start-up, autostart, and stopping on a signal. |
| `planar/opcua_server.py` | The OPC UA address space, built from the modules; methods run the controller's code in worker threads. |
| `planar/statemachines.py` | The generic module level, as in the lab's generated IEC 61499 modules: `Occupation`, `Module` (the PackML state machine), `Skill` (the skill core), `Procedure`, `ErrorID`. It knows nothing about tables. |
| `planar/modules.py` | `PlanarController`: the table and shuttle modules wired to the coordinator. Their procedures, the `MoveToPosition` skill, the equipment commands, and which xbots are pinned. |
| `planar/coordinator.py` | `Coordinator`: turns tasks into plans and plans into motions, every 20 ms. Turns, stations, claims, the loading queue, Hold, zones, speed. |
| `planar/mapf.py` | The multi-xbot path finder: LaCAM with PIBT, the dependencies between moves, the time estimate. |
| `planar/coordinator_blended.py` | Optional experimental coordinator, selected with `PLANNER=blended`. Reuses the classic lifecycle and route search, but reserves and executes eligible pairs of straight runs as one rounded XY macro. |
| `planar/rounded_path.py` | Quarter-circle path geometry and conservative swept footprint for the experimental executor and kinematic simulator. |
| `planar/layout.py` | `Layout`: the table's geometry, the slots, footprints, spots, boxes and trails. |
| `planar/lines.py` | Line mode: the layout's JSON (or AAS) form, the slot graph, and which slots each xbot may use. |
| `planar/zones.py` | The zones of `ZONES`. |
| `planar/driver.py` | `Driver`: what the coordinator needs from a PMC; `Reading`, `DriverError`. |
| `planar/pmc_driver.py` | `PmcDriver`: a real PMC, or Planar Motor's simulator, through pmclib. |
| `planar/sim_driver.py` | `SimDriver`: a kinematic stand-in for the tests, on the tests' own clock. |

## Threads and the lock

- **The main thread** runs asyncio: the OPC UA server (asyncua) and, with `AUTOSTART`, the
  autostart. A method call runs the controller's code in a worker thread
  (`asyncio.to_thread`), so a plan being made never holds up the server. Every 50 ms, the server
  takes a snapshot of every variable under the lock and writes the ones that changed.
- **The coordinator's thread** runs a tick every 20 ms.
- **One re-entrant lock** (`statemachines.make_lock`) guards all state: occupations, modules,
  skills and the coordinator. Everything that reads or changes state holds it. Planning happens
  inside a tick, with the lock held, for up to 0.5 s; a method call that needs the lock waits
  meanwhile.
- **Jobs**: what may block for long, the PMC connection at a Reset and the reading of the
  flyways, runs at the start of the next tick without the lock (`Coordinator.run_job`).
- **The driver serialises its own calls.** pmclib is one .NET session without synchronisation,
  and `stop`, `pause` and `resume` may come from any thread; `PmcDriver` takes its own lock
  around every call.

## A tick

`Coordinator.tick(now)` does, in order:

1. Run the jobs waiting, without the lock. Everything after this holds the lock.
2. Run the `on_tick` hooks: the table's `poll` (a skill's Succeeded turns Idle after 1.5 s; a
   Stop goes on), and a shuttle's wait for a recovered xbot.
3. Read every xbot from the driver. A `DriverError` is a lost connection, or lost mastership:
   the table aborts.
4. Update each xbot: its pose and state, and faults. A fault, or an xbot switched off, fails
   its task.
5. Read the zones' states every 0.5 s, and queue a job to read the flyways every 5 s.
6. While the PMC's auto routing lines up the xbots (at a Reset), nothing else moves.
7. Advance the motions in flight: progress past slot centres, arrival, timeouts.
8. Advance the tasks: each task's next step ([The planner](planner.md#the-life-of-a-request)).
9. Plan again, if anything that matters has changed.
10. Dispatch: send each xbot its next motion whose dependencies are met.

## A move, end to end

1. A client calls `PlanarShuttle1/Skills/MoveToPosition/Start(Session, X, Y, Rz)`. The server
   runs `Skill.start` in a worker thread.
2. `Skill.start` checks the gate: occupation, module state, busy, ranges. It then asks
   `MoveToPosition.check`, which asks `Coordinator.check_target` about the PMC, the xbot and the
   target.
3. The skill turns Running and calls `MoveToPosition.begin`, which calls `Coordinator.submit`.
   The coordinator creates a `Task` with a callback, `finished(ok, error, results)`.
4. The server refreshes the variables and answers `[true, 0]`.
5. The coordinator's ticks plan and dispatch: `Driver.move` and `Driver.rotate`, read back by
   `Driver.read`.
6. On arrival, the coordinator calls `finished(True, NONE, results)`: the skill turns Succeeded,
   with the results. The next refresh shows State 3, and 1.5 s later Idle.

## The driver interface

`planar/driver.py` defines what the coordinator needs from a table. Units are metres, radians
and seconds.

| Method | Needed | |
| --- | --- | --- |
| `connect()` | yes | Connect, take mastership (`DriverError` with MastershipLost if another program keeps it), recover the xbots switched off after an accident, levitate. Returns the `Layout`. |
| `read()` | yes | `{xid: Reading(x, y, rz, state, detail)}`, where state is `idle`, `moving`, `fault` or `off`, and detail is the PMC's word for it. A `DriverError` with MastershipLost once another program has taken mastership. |
| `move(xid, x, y, speed, accel)` | yes | One straight motion to (x, y), at most speed and accel. Returns the seconds it should take. |
| `rotate(xid, rz)` | yes | Turn to rz the shortest way, by at most half a turn. Returns the seconds it should take. |
| `stop(xid=0)` | yes | Stop the xbot at once; 0: every xbot. |
| `close()` | | Disconnect; the controller is stopping. |
| `pause(xid)`, `resume(xid)` | for Hold | Pause the xbot's motion where it is (it reads as idle meanwhile), and carry on with it. Without them, a Hold aborts the module. |
| `auto_drive(xids, targets, speed, accel)` | | Route these xbots with the table's own routing, each to any one of the targets. Without it, the planner lines them up at a Reset. |
| `payload(xid)`, `set_payload(xid, kg, cg_height)` | for SetPayload | The payload as the PMC has it, and the maximum acceleration with it. |
| `recover(xids)` | | Recover the xbots switched off after an accident; returns the ones it did. |
| `define_zone`, `zone_status`, `zone_control` | for ZONES | Without them, `ZONES` fails the Reset with ZonesInvalid. |
| `flyways()` | | `{(column, row): (power W, CPU, amplifier, motor temperature °C)}`. |

A `DriverError(message, code)` carries the ErrorID the failure shows: PmcError (22) by default,
OutOfRange (8) for a parameter the PMC refuses, PmcNotConnected (27), MastershipLost (32).

The coordinator relies on three things:

- every `move` is a straight line;
- `rotate` turns the short way;
- `read` reports a motion as `moving` until it has ended.

It does its own checking of where xbots may turn, and of what is on the table.

### A driver for another table

A driver implements at least the five required methods. A sketch, with the table's own calls
left out:

```python
from planar.driver import Driver, DriverError, Reading
from planar.layout import Layout
from planar.statemachines import ErrorID


class MyTableDriver(Driver):
    def connect(self):
        ...  # connect to the table's controller, take control, power the movers up
        return Layout(cols=4, rows=3, holes=frozenset({(2, 1)}))  # 240 mm tiles, 120 mm movers

    def read(self):
        return {xid: Reading(x, y, rz, "idle") for xid, (x, y, rz) in ...}

    def move(self, xid, x, y, speed, accel):
        ...  # one straight motion; DriverError(..., ErrorID.PMC_ERROR) if refused
        return seconds_it_should_take

    def rotate(self, xid, rz):
        ...
        return seconds_it_should_take

    def stop(self, xid=0):
        ...
```

`python -m planar` always uses `PmcDriver`. Another driver needs a start script of its own, like
`planar/__main__.py`'s `main()`:

```python
import asyncio

from planar.modules import PlanarController
from planar.opcua_server import OpcUaServer


async def main():
    ctl = PlanarController(MyTableDriver(), [1, 2, 3], speed=0.5, accel=2.0)
    server = OpcUaServer(ctl, "opc.tcp://0.0.0.0:4840/")
    await server.init()
    ctl.start()
    try:
        await server.run()
    finally:
        ctl.shutdown()  # stops the coordinator and closes the driver


asyncio.run(main())
```

`PlanarController` passes its other keyword arguments on to the `Coordinator`: `speed`, `accel`,
`patience_s`, `loading_station`, `idle_rz`, `line_spec`, `line_source`, `zones`, and
`displace_held` for the modules.

`Layout` assumes 240 mm flyways and 120 mm xbots by default. The flyway size must be a whole
number of xbot widths.

## PmcDriver

`planar/pmc_driver.py` drives a PMC through pmclib, Planar Motor's Python library. The wheel,
117.9.1, is in `dependencies/` (not on PyPI). It needs PMC firmware 117.17.35 or later. The
module refuses to load with an older pmclib, and says which to install.

- **One lock** around every pmclib call (`_pmc()`). What pmclib raises becomes a `DriverError`;
  `INVALIDPARAMS` becomes OutOfRange.
- **`connect`**: connects (to `PMC_IP`, or by auto search) and takes mastership. It activates
  the xbots and waits up to 30 s for the PMC to be ready. It recovers xbots switched off after an
  accident, and levitates the rest (not those in a zone switched off). It resumes motions paused
  by a controller that stopped while holding, and sets `XBOT_JERK`. Last, it has the PMC save
  its configuration as XML, to read the table's layout and the flyways' ids.
- **`read`**: every xbot's pose and state. Every second it asks whether it is still the PMC's
  master. While an xbot reads as off or faulted, it also asks every second which xbots had an
  accident. It wraps the PMC's angle, which is not wrapped: an xbot turned round twice reads
  -720 degrees. The PMC's states are mapped:

    | Reading | The PMC's `XBOTSTATE` |
    | --- | --- |
    | idle | `XBOT_IDLE` |
    | moving | `XBOT_MOTION`, `XBOT_WAIT`, `XBOT_STOPPING`, `XBOT_HOLDPOSITION`, `XBOT_RESERVED1` (driven by the auto routing) |
    | fault | `XBOT_ERROR`, `XBOT_OBSTACLE_DETECTED`, `XBOT_STOPPED` |
    | off | the rest: `XBOT_DISABLED`, `XBOT_LANDED`, `XBOT_UNDETECTED`, ... |

- **`move`**: `linear_motion_si`, absolute and direct, at the given speed and acceleration.
- **`rotate`**: `rotary_motion_p2p`, relative, by the wrapped difference from the xbot's angle,
  at 3 rad/s and 10 rad/s².
- **`stop`**: `stop_motion`. On Planar Motor's simulator, it also clears an
  `XBOT_OBSTACLE_DETECTED`: the xbot is idle again.
- **`auto_drive`**: `auto_driving_motion_si_with_speed`, any xbot to any target.
- **`pause`, `resume`**: `motion_interrupt_command`.
- **`payload`, `set_payload`**: `read_xbot_payload_settings`, `write_xbot_payload_settings`,
  and `get_mover_properties` for `MAXACCELERATION_7`. pmclib takes the property id as a plain
  `int`, not as its own IntEnum.
- **Zones**: `define_zone` with `override_zone` for the speed, `get_zone_status`,
  `zone_control`, and `zone_fence_control` to remove the fence.
- **`flyways`**: `get_flyway_physical_status` per flyway, one at a time, so moves go on between
  them.
- **`close`**: `disconnect_from_pmc`, always, when the controller stops.

## SimDriver

`planar/sim_driver.py` stands in for the PMC in the tests. Xbots move in straight lines with a
trapezoidal profile. They turn only on spots; anywhere else a turn is refused. Time is whatever
the caller says (`advance(now)`), so tests run faster than real time. Every overlap of two xbots
is recorded as a collision, counting a turning xbot as the circle it sweeps.

It behaves as Planar Motor's simulator does, where that matters to the controller:

- a paused xbot reads as idle where it stopped;
- a payload over 0.5 kg is refused, and `MaxAcceleration` is always 20 m/s²;
- the xbots in a zone switched off read as off;
- a move through a slow zone is slowed, though the time it is said to take is not;
- a move into the trail of another's move with more than 0.65 s to go is stopped with
  `XBOT_OBSTACLE_DETECTED`, which `stop` clears.

It also offers what tests need: `accident(xid)` switches an xbot off until it is recovered;
`put()` and `take()` put xbots on and take them off; `master = False` stands in for another
program taking mastership. The PMC's auto routing is stood in for by the slot planner.

## Extending

- **A setting**: an option in `parse_args` in `planar/__main__.py`, with its environment
  variable as the default. Pass it to `PlanarController` or the `Coordinator`. List it in
  `.env.example` and `docker-compose.yml`, and in the [configuration](configuration.md).
- **An equipment variable or method**: in `OpcUaServer._table_equipment` or
  `_shuttle_equipment`. `_variable(parent, name, read, type)` publishes `read()`, called with
  the lock held. `_method(parent, name, fn, inputs)` runs `fn` in a worker thread; it must
  return `(accepted, ErrorID)` and take the lock itself.
- **A skill**: a `SkillBody` with `PARAMETERS` and `RESULTS`, and `check`, `begin`, `halt` and
  `abort`. Add it with `Module.add_skill` in `PlanarController`, as `MoveToPosition` is. The
  server publishes it under `Skills/` with its parameters and results.
