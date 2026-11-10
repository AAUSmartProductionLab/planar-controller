# Testing

The controller is tested at three levels. The test suite runs the controller on a simulated
table, faster than real time. A production line scenario drives it over OPC UA. Planar Motor's
simulator stands in for a real PMC.

## The test suite

```bash
pip install -r requirements.txt dependencies/pmclib-117.9.1-py3-none-any.whl pytest
python -m pytest tests -m "not slow"      # no PMC needed; about half a minute
python -m pytest tests                    # and the production line in real time; about 5 minutes
```

The tests marked `slow` run the production line in real time, and the densest traffic.

| File | What it tests |
| --- | --- |
| `tests/test_mapf.py` | The planner alone: passing in a passage with a pocket, keeping out, no way found without hanging, random instances, straight paths before staircases, followers, the time estimate with each xbot's speed, the fastest of the seeds. |
| `tests/test_coordinator.py` | The coordinator on the simulated driver: passages, pushing, turns, stations between slots, xbots that touch, claims, request order, the trail rule, the loading queue, the idle orientation, random traffic. |
| `tests/test_modules.py` | The modules as an orchestrator uses them: the gate, Stop and Abort, the table's commands reaching its shuttles, who may command, pinning, the Reset lining up and turning the shuttles. |
| `tests/test_features.py` | Hold and Unhold, speed, payload, accidents, mastership, zones and the flyways' status. |
| `tests/test_lines.py` | Line mode: layouts read from files, JSON strings, an AAS and a URL; layouts that are wrong; lanes, spurs, turn spots, the idle area; reading the layout again at a Reset; traffic on a loop. |
| `tests/test_line.py` | The production line scenario over OPC UA on the simulated driver: on every layout of `config/layouts/`, with holds, and its own checks. |
| `tests/test_opcua.py` | The OPC UA address space end to end, with a client. |
| `tests/test_pmc_driver.py` | `PmcDriver` against a stand-in for pmclib that refuses what pmclib refuses: turns, angles, mastership, accidents, pausing, payload, speeds. |
| `tests/test_blended.py` | Experimental corner geometry, reservations through replanning, traffic conflicts, fallback, Hold, Stop and Abort. Coordinator scenarios also run with both planner variants. |

The optional `python tools/compare_planners.py --run --output <new-directory>` benchmark moves
xbot 1 on the standard vendor simulator using both planners, records CSV feedback and JSON
timings, and returns it to (420,180) mm. See [blended execution](planner.md#experimental-blended-execution).

### Writing a test

`tests/harness.py` sets up the table and its shuttles on the simulated driver, with the test's
own clock. `Cell.run(seconds, until)` steps time on in 20 ms steps until `until()` is true:

```python
from harness import Cell, c

from planar.statemachines import ErrorID, SkillState


def test_a_shuttle_gets_past_a_free_one_in_its_way():
    cell = Cell({1: c(0, 0), 2: c(3, 0)})  # two xbots on the 4 x 1 flyway strip (8 x 2 slots)
    cell.up()                               # Reset and Start the table, and every shuttle
    assert cell.s[1].occupation.occupy("agent") == (True, ErrorID.NONE)
    assert cell.move(1, "agent", *c(7, 0)) == (True, ErrorID.NONE)
    assert cell.run(30, lambda: cell.skill(1).state != SkillState.RUNNING)  # simulated seconds
    assert cell.skill(1).state == SkillState.SUCCEEDED
    assert cell.sim.at(1, 0.90, 0.06)       # slot (7, 0)'s centre, in metres
    assert cell.run(10, lambda: cell.sim.at(2, 0.42, 0.06))  # 2, pushed aside, is back on its slot
    assert not cell.sim.collisions
```

`c(i, j)` is slot (i, j)'s centre in mm, as the skill takes it. `Cell(..., layout=SIM)` is
Planar Motor's simulator's table, 4 × 3 flyways with (2, 1) missing. Options such as
`loading_station=...` or `zones=[...]` go on to the controller. `cell.sim` is the `SimDriver`:
its `collisions`, `commands` and `turns` record what happened.

### The contention stress test

```bash
python tests/stress_contention.py 40      # 40 seeds, 12 runs each
```

Many xbots ask for the same place. Each that gets there stays a while, pinned as a station would
hold it, then leaves for a parking spot of its own. The places are of every kind: slot centres,
between slots, slot corners, flyway centres, turned and crooked. It checks that every request
succeeds and that arrivals follow request order, with no collisions and no hangs. It prints a
line per bad run and a summary.

## The production line scenario

`tests/line_scenario.py` is a production line driven over OPC UA only (see
[Examples](examples.md#the-test-scenario)). Product agents, each with its own client and
session, take shuttles through five stations, and watch every shuttle as they go. In line
mode they also check the lanes' directions. `tests/test_line.py` runs it on the simulated
table. Against a running controller:

```bash
python tests/line_scenario.py --endpoint opc.tcp://127.0.0.1:4840/ --agents 3 --products 3 --fail 0.2
```

| Option | |
| --- | --- |
| `--endpoint URL` | the controller (`opc.tcp://127.0.0.1:4840/`) |
| `--agents N`, `--products N` | product agents, and products per agent (3, 3) |
| `--fail P` | the chance that a process step fails, and the product is discarded (0.2) |
| `--seed N` | for the agents' random choices (1) |
| `--stations FILE` | stations from a JSON file, `{"<station>": [x, y, rz]}` |
| `--layout DIR` | a folder of `config/layouts/`: its `stations.json`; start the controller with its `controller.env` |
| `--hold-every S` | hold the table every S seconds for 2 s: nothing may move while it is held |

It prints each product and a summary, and exits 1 on any problem.

## Planar Motor's simulator

Planar Motor's simulation tool runs a virtual PMC on a Windows PC. **It is not commercially
available and is not in this repository**; do not add it. `pmc-sim/` is ignored by git and
Docker in case a copy is kept there.

`tools/pmc-sim.ps1` runs the simulator's engine (`Dlls\VirtualPMC.exe`) without the tool's
window. It runs it from where the tool is installed, from `PMC_SIM_DIR`, or from `pmc-sim/`. The
engine listens on UDP port 8888, so the controller runs with `PMC_IP=127.0.0.1`.

```powershell
powershell -ExecutionPolicy Bypass -File tools\pmc-sim.ps1 start      # or stop, restart, status
```

While the tool's window is open, an engine started by the script may take no connections. Start
the simulation from the window then.

### Every layout against the simulator

`tools/sim_check.py` runs each layout of `config/layouts/` against the simulator. For each it
starts a controller with the layout's settings on a free port, runs the scenario's agents with
the layout's stations, and prints PASS or FAIL. Run it with the Python the controller runs with
(pmclib from `dependencies/`):

```bash
python tools/sim_check.py                          # every layout; the simulator running
python tools/sim_check.py clockwise-outwards --start-sim --agents 3 --products 4
python tools/sim_check.py --hold-every 8 --mastership
```

```text
PASS clockwise-outwards (101 s): 9 products: 4 finished, 5 discarded; 40 moves, mean 4.6s, longest 9.6s; 6 occupy refusals (5); 24 lane steps; turns 1x -180 deg, 28x -90 deg, 2x +90 deg; 8 holds (59 Starts refused while held); 0 problems
1 of 1 passed; logs in C:\Users\...\Temp\sim-check-tn8tjqw4
```

`--hold-every S` holds the table every S seconds during the scenario. `--mastership` then takes
mastership from another pmclib session: the table must abort with MastershipLost (32), and take
mastership back at a Clear and Reset. It exits 1 when a layout fails; the logs are kept in a
temporary folder.

### Manual motion probes and demos

The scripts in `tools/` keep their module summaries short; use `--help` for their options.
The following examples run from the repository root with the controller's Python environment:

| Command | Purpose |
| --- | --- |
| `python tools/jog_probe.py` | Inspect the installed jogging API without connecting or moving. |
| `python tools/jog_probe.py --run --xbot 1 --vx 0.02 --duration 1` | Jog a prepared, levitated and idle xbot for one second and print CSV feedback. Vector speed is capped at 0.05 m/s and duration at 5 seconds. |
| `python tools/jog_position_probe.py --run --restart --output jog-results` | Test PI position feedback: move 20 mm in X and 15 mm in Y, then return. `--restart` stops jogging before each velocity update. Without it, the test probes whether updates take effect during motion. |
| `python tools/demo_blended.py --simulate --output demo-preflight` | Run the three-shuttle demo on the kinematic model without connecting to a PMC. |
| `python tools/demo_blended.py --run --output demo-results` | Run two cycles of three-shuttle traffic on the vendor simulator, then park the movers. |
| `python tools/compare_planners.py --run --output comparison-results` | Compare classic and blended execution on the standard simulator layout. |

The vendor-simulator probes default to `127.1.1.1`. Release PMT mastership before running them;
the existing controller must not command the same session concurrently. The demo and comparison
use the standard simulator layout and xbot IDs described by their commands. These scripts use
the Python driver directly, rather than OPC UA. Use a new output directory for each run.

The motion probes require `--run` to connect and move. They request stops and disconnect during
normal exception cleanup, but that cleanup cannot run after process termination or network loss.
The PI probe does not activate or rotate movers; the demo and comparison use normal coordinator
connection and initialization. The jog probe's `--ip` option can override its default address.

### Disconnect from the PMC

Every program that connects with pmclib must call `disconnect_from_pmc()` before it ends. For
each connection left open, the simulator writes an error to its trace log
(`%APPDATA%\Planar Motor Inc\Virtual PMC\data\PMTraceLog.txt`) on every read: gigabytes an hour,
until it takes no new connections. The controller disconnects when it stops on Ctrl+C,
Ctrl+Break or SIGTERM. `sim_check.py` stops its controllers with Ctrl+Break. A controller that
is killed does not disconnect.

### How the simulator differs from the stand-in

The simulated driver of the test suite is kinematic: it follows the commands it is given. The
simulator is Planar Motor's PMC software, and it is the better test of anything that changes
how xbots move:

- **Turns**: the stand-in once turned an xbot three quarters round without anyone noticing; on
  the PMC that was obvious. Run `sim_check.py` after changing how xbots move.
- **Obstacle detection**: the PMC stops a move into another xbot's way while that xbot's
  motion still has a while to go. The stand-in copies the rule as measured (0.65 s), but the
  PMC's own rule is not published, and one corner case is known to differ (see
  [The planner](planner.md#keeping-out-of-another-xbots-trail)).

What was measured on the simulator, for reference:

- **Speed**: an xbot tops out at about 3 m/s, whatever higher speed is set.
- **Payload**: up to 0.5 kg is taken (more is refused), but not kept: it reads back 0 kg, and
  `MaxAcceleration` is always 20 m/s².
- **Flyways**: power and temperatures all read 0.
- **Zones**: switching one off and on works, landing and levitating the xbots inside. A zone
  stays defined until the simulation restarts.
- **Reconnecting**: right after a disconnect, a new connection from the same machine may fail
  once with "Socket binding failed"; the next try works.

## What to run after a change

| Change | Run |
| --- | --- |
| any | `python -m pytest tests -m "not slow"`, and pyflakes |
| before committing | `python -m pytest tests` |
| the planner, the coordinator | also `python tests/stress_contention.py 40` |
| how xbots move or turn, the driver | also `python tools/sim_check.py --hold-every 8 --mastership` on the simulator |
