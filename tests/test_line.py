"""The production line of line_scenario.py, through OPC UA, on the simulated driver."""

import asyncio
import socket
import time

import pytest

from line_scenario import run_line, summary
from planar.layout import Layout
from planar.modules import PlanarController
from planar.opcua_server import OpcUaServer
from planar.sim_driver import SimDriver

SIM = Layout(cols=4, rows=3, holes=frozenset({(2, 1)}))


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


async def _line(agents, products, seed):
    # Three xbots on the flyway centres of the left column, as in the simulation tool; fast moves.
    sim = SimDriver(SIM, {1: (0.12, 0.12, 0.0), 2: (0.12, 0.36, 0.0), 3: (0.12, 0.60, 0.0)},
                    speed=2.0, accel=10.0, turn_speed=20.0, turn_accel=200.0)
    ctl = PlanarController(sim, [1, 2, 3])
    ctl.coordinator.on_tick.insert(0, lambda: sim.advance(time.monotonic()))
    endpoint = f"opc.tcp://127.0.0.1:{free_port()}/"
    server = OpcUaServer(ctl, endpoint)
    await server.init()
    ctl.start()
    serving = asyncio.create_task(server.run())
    try:
        report = await run_line(endpoint, agents=agents, products=products, process_s=(0.1, 0.3),
                                fail_p=0.2, seed=seed, sample=False)
    finally:
        serving.cancel()
        ctl.shutdown()
    return report, sim


@pytest.mark.parametrize("agents,products,seed", [(3, 4, 1), (4, 3, 2)])
def test_product_agents_run_the_line(agents, products, seed):
    report, sim = asyncio.run(_line(agents, products, seed))
    print(summary(report))
    assert not report.problems, report.problems
    assert not sim.collisions, sim.collisions[:3]
    assert len(report.products) == agents * products
    outcomes = [p.outcome for p in report.products]
    assert all(o == "finished" or o.startswith("discarded") for o in outcomes), outcomes
    assert "finished" in outcomes and any(o.startswith("discarded") for o in outcomes)
    if agents > 3:
        assert report.occupy_refusals > 0, "four agents share three shuttles"
