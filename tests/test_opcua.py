import asyncio
import socket
import time

from asyncua import Client, ua

from planar.layout import Layout
from planar.modules import PlanarController
from planar.opcua_server import NAMESPACE, OpcUaServer
from planar.sim_driver import SimDriver
from planar.statemachines import ErrorID, ModuleState, SkillState

SIM = Layout(cols=4, rows=3, holes=frozenset({(2, 1)}))


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def test_a_move_over_opc_ua():
    asyncio.run(_move_over_opc_ua())


async def _move_over_opc_ua():
    sim = SimDriver(SIM, {1: (0.06, 0.06, 0.0), 2: (0.90, 0.66, 0.0)})
    ctl = PlanarController(sim, [1, 2])
    ctl.coordinator.on_tick.insert(0, lambda: sim.advance(time.monotonic()))
    endpoint = f"opc.tcp://127.0.0.1:{free_port()}/"
    server = OpcUaServer(ctl, endpoint)
    await server.init()
    ctl.start()
    serving = asyncio.create_task(server.run())
    try:
        await asyncio.sleep(0.5)
        async with Client(endpoint) as client:
            ns = await client.get_namespace_index(NAMESPACE)

            async def path(*names):
                return await client.nodes.objects.get_child([f"{ns}:{n}" for n in names])

            async def read(*names):
                return await (await path(*names)).read_value()

            async def call(node, method, *args):
                out = await node.call_method(f"{ns}:{method}", *args)
                return out[0], ErrorID(out[1])

            async def wait(node, value, timeout=20.0):
                t0 = time.monotonic()
                while time.monotonic() - t0 < timeout:
                    if await node.read_value() == value:
                        return True
                    await asyncio.sleep(0.05)
                return False

            session = ua.Variant("6f1c2d3e-orchestrator", ua.VariantType.String)
            table = await path("PlanarTable", "Module")
            skill = await path("PlanarShuttle1", "Skills", "MoveToPosition")
            mm = lambda v: ua.Variant(float(v), ua.VariantType.Double)

            assert await call(table, "Reset", session) == (True, ErrorID.NONE)
            assert await wait(await path("PlanarTable", "Module", "State"), int(ModuleState.IDLE))
            pmc = ("PlanarTable", "Equipment", "PMC")
            assert [await read(*pmc, n) for n in ("Columns", "Rows", "FlywaySize", "Holes")] == [4, 3, 240.0, "2,1"]
            assert await call(table, "Start", session) == (True, ErrorID.NONE)
            assert await wait(await path("PlanarShuttle1", "Module", "State"), int(ModuleState.EXECUTE))

            assert await call(skill, "Start", session, mm(780), mm(60), mm(0)) == (False, ErrorID.NOT_PERMITTED)
            assert await call(await path("PlanarShuttle1", "Occupation"), "Occupy", session) == (True, ErrorID.NONE)
            assert await wait(await path("PlanarShuttle1", "Occupation", "Occupied"), True)
            assert await call(skill, "Start", session, mm(600), mm(360), mm(0)) == (False, ErrorID.NOT_ON_TABLE)

            assert await call(skill, "Start", session, mm(780), mm(660), mm(90)) == (True, ErrorID.NONE)
            assert await wait(await path("PlanarShuttle1", "Skills", "MoveToPosition", "State"), int(SkillState.SUCCEEDED))
            assert await read("PlanarShuttle1", "Skills", "MoveToPosition", "Results", "X") == 780.0
            assert abs(await read("PlanarShuttle1", "Equipment", "Xbot", "Rz") - 90.0) < 0.1
            assert await read("PlanarShuttle1", "Skills", "MoveToPosition", "Parameters", "Y") == 660.0
            assert await wait(await path("PlanarShuttle1", "Skills", "MoveToPosition", "State"), int(SkillState.IDLE), 3)

            assert await call(await path("PlanarShuttle1", "Occupation"), "Release", session) == (True, ErrorID.NONE)
            assert not sim.collisions
    finally:
        serving.cancel()
        ctl.shutdown()
