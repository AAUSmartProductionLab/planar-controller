"""The production line of line_scenario.py, through OPC UA, on the simulated driver."""

import asyncio
import math
import socket
import time
from pathlib import Path

import pytest

from line_scenario import MotionCheck, Report, read_stations, run_line, summary
from planar.layout import Layout
from planar.lines import Lines, load
from planar.modules import PlanarController
from planar.opcua_server import OpcUaServer
from planar.sim_driver import SimDriver

SIM = Layout(cols=4, rows=3, holes=frozenset({(2, 1)}))
LAYOUTS = sorted(p for p in (Path(__file__).parent.parent / "config" / "layouts").iterdir() if p.is_dir())


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


async def _line(agents, products, seed, loading_station=None, sample=False, places=None, turn_speed=20.0, **kw):
    # Three xbots on the flyway centres of the left column, as in the simulation tool; fast moves
    # (turns slow enough to be sampled when the shuttles are watched).
    sim = SimDriver(SIM, {1: (0.12, 0.12, 0.0), 2: (0.12, 0.36, 0.0), 3: (0.12, 0.60, 0.0)},
                    speed=2.0, accel=10.0, turn_speed=turn_speed, turn_accel=10 * turn_speed)
    ctl = PlanarController(sim, [1, 2, 3], loading_station=loading_station, **kw)
    ctl.coordinator.on_tick.insert(0, lambda: sim.advance(time.monotonic()))
    endpoint = f"opc.tcp://127.0.0.1:{free_port()}/"
    server = OpcUaServer(ctl, endpoint)
    await server.init()
    ctl.start()
    serving = asyncio.create_task(server.run())
    try:
        report = await run_line(endpoint, agents=agents, products=products, process_s=(0.1, 0.3),
                                fail_p=0.2, seed=seed, sample=sample, places=places)
    finally:
        serving.cancel()
        ctl.shutdown()
    return report, sim


def _settings(path):
    """controller.env: {KEY: value}."""
    lines = (line.strip() for line in path.read_text(encoding="utf-8").splitlines())
    return dict(line.split("=", 1) for line in lines if line and not line.startswith("#"))


INFEED = (0.06, 0.18)  # line_scenario.STATIONS["Infeed"], where the products are put on


@pytest.mark.parametrize("agents,products,seed,loading_station", [(3, 4, 1, None), (4, 3, 2, None), (3, 4, 3, INFEED)])
def test_product_agents_run_the_line(agents, products, seed, loading_station):
    report, sim = asyncio.run(_line(agents, products, seed, loading_station))
    print(summary(report))
    assert not report.problems, report.problems
    assert not sim.collisions, sim.collisions[:3]
    assert len(report.products) == agents * products
    outcomes = [p.outcome for p in report.products]
    assert all(o == "finished" or o.startswith("discarded") for o in outcomes), outcomes
    assert "finished" in outcomes and any(o.startswith("discarded") for o in outcomes)
    if agents > 3:
        assert report.occupy_refusals > 0, "four agents share three shuttles"


@pytest.mark.parametrize("folder", LAYOUTS, ids=lambda p: p.name)
def test_each_layout_of_config_layouts_runs_the_line(folder):
    """config/layouts/<name>/: its layout and settings, its stations; the shuttles watched."""
    settings = _settings(folder / "controller.env")
    assert settings["MODE"] == "line"
    assert Path(settings["LINE_LAYOUT"]).as_posix() == f"config/layouts/{folder.name}/layout.json"
    idle_rz = math.radians(float(settings["IDLE_RZ"])) if settings.get("IDLE_RZ") else None
    report, sim = asyncio.run(_line(3, 4, 1, sample=True, places=read_stations(folder / "stations.json"),
                                    turn_speed=6.0, line_spec=load(str(folder / "layout.json")), idle_rz=idle_rz))
    print(summary(report))
    assert not report.problems, report.problems
    assert not sim.collisions, sim.collisions[:3]
    assert len(report.products) == 12 and "finished" in [p.outcome for p in report.products]
    assert report.lane_steps > 0 and report.turns, "the shuttles were watched"
    assert all(abs(turn) <= math.pi + 1e-9 for _, turn in sim.turns)


def test_the_watch_tells_a_step_against_the_lane_and_a_turn_off_a_spot_or_the_long_way():
    report = Report()
    check = MotionCheck(SIM, Lines(load(str(LAYOUTS[0] / "layout.json")), SIM), report)
    t = 0.0

    def see(*pose):
        nonlocal t
        t += 0.05
        check.sample("S", pose, t)

    see(180, 300, -90), see(180, 420, -90)      # north up the west side: the clockwise way
    see(180, 300, -90)                          # and back
    for k in range(1, 10):                      # -90 to 180 deg anticlockwise, on a turn spot
        see(120, 600, -90 + 30 * k)
    for _ in range(8):
        see(120, 600, 180)
    for k in range(1, 4):                       # a quarter turn off the turn spots
        see(300, 660, 180 - 30 * k)
    for _ in range(8):
        see(300, 660, 90)
    assert report.lane_steps == 2
    assert [p.split(" ", 1)[1] for p in report.problems] == [
        "went from slot (1, 3) to (1, 2), against the lane",
        "turned +270 deg at (120, 600) mm, the long way",
        "turned at (300, 660) mm, not on a turn spot"]
    assert [turn[-1] for turn in report.turns] == [270, -90]
