import asyncio
import math
import socket
import time

from asyncua import Client, ua

from planar.layout import Layout
from planar.modules import PlanarController
from planar.opcua_server import NAMESPACE, OpcUaServer
from planar.sim_driver import SimDriver
from planar.statemachines import ErrorID, ModuleState, SkillState
from planar.zones import ZoneSpec

SIM = Layout(cols=4, rows=3, holes=frozenset({(2, 1)}))


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def test_a_move_over_opc_ua():
    asyncio.run(_move_over_opc_ua())


async def _move_over_opc_ua():
    sim = SimDriver(SIM, {1: (0.06, 0.06, 0.0), 2: (0.90, 0.66, 0.0)})
    sim.flyway_status = {(0, 0): (12.0, 41.0, 36.0, 30.5)}
    ctl = PlanarController(sim, [1, 2], zones=[ZoneSpec("Service", (0.72, 0.48, 0.96, 0.72))])
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

            xbot = ("PlanarShuttle1", "Equipment", "Xbot")
            assert all(math.isnan(v) for v in [await read(*xbot, k) for k in ("X", "Y", "Rz")]), "not read until the Reset"
            assert await call(table, "Reset", session) == (True, ErrorID.NONE)
            assert await wait(await path("PlanarTable", "Module", "State"), int(ModuleState.IDLE))
            assert [await read(*xbot, k) for k in ("X", "Y", "Rz")] == [60.0, 60.0, 0.0]
            pmc = ("PlanarTable", "Equipment", "PMC")
            assert [await read(*pmc, n) for n in ("Columns", "Rows", "FlywaySize", "Holes")] == [4, 3, 240.0, "2,1"]
            assert [await read("PlanarTable", "Equipment", n) for n in ("Mode", "LineLayout", "LineLayoutProblem")] \
                == ["free", "", ""]
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

            # Hold and Unhold; the payload, the occupant's to set.
            assert await call(table, "Hold", session) == (True, ErrorID.NONE)
            assert await wait(await path("PlanarShuttle1", "Module", "State"), int(ModuleState.HELD))
            assert await call(table, "Unhold", session) == (True, ErrorID.NONE)
            assert await wait(await path("PlanarShuttle1", "Module", "State"), int(ModuleState.EXECUTE))
            assert [await read(*xbot, k) for k in ("PayloadKg", "PayloadCgHeight", "MaxAcceleration")] == [0.0, 0.0, 20.0]
            xbot_node = await path(*xbot)
            assert await call(xbot_node, "SetPayload", session, mm(0.8), mm(25)) == (False, ErrorID.OUT_OF_RANGE)
            assert await call(xbot_node, "SetPayload", session, mm(0.25), mm(25)) == (True, ErrorID.NONE)
            assert await wait(await path(*xbot, "PayloadKg"), 0.25)
            assert await read(*xbot, "PayloadCgHeight") == 25.0

            # Speed: the table's, every xbot's; a shuttle's own keeps its xbot to less.
            equipment = await path("PlanarTable", "Equipment")
            other = ("PlanarShuttle2", "Equipment", "Xbot")
            assert [await read("PlanarTable", "Equipment", k) for k in ("Speed", "Acceleration")] == [0.5, 2.0]
            assert await call(equipment, "SetSpeed", session, mm(-1), mm(0)) == (False, ErrorID.OUT_OF_RANGE)
            assert await call(equipment, "SetSpeed", session, mm(1.0), mm(4.0)) == (True, ErrorID.NONE)
            assert await call(xbot_node, "SetSpeed", session, mm(0.25), mm(0)) == (True, ErrorID.NONE)
            assert [await read(*xbot, k) for k in ("Speed", "Acceleration")] == [0.25, 4.0]
            assert [await read(*other, k) for k in ("Speed", "Acceleration")] == [1.0, 4.0]

            # A zone, and the flyways (added at the Reset).
            zone = ("PlanarTable", "Equipment", "Zones", "Service")
            assert [await read(*zone, k) for k in ("State", "Xbots")] == ["ACTIVATED_UNFENCED", "1 2"]  # 1 moved in
            assert await call(await path(*zone), "Deactivate", session) == (True, ErrorID.NONE)
            assert await wait(await path(*zone, "State"), "LOADING_FENCED")
            assert await read("PlanarShuttle2", "Equipment", "Xbot", "XbotState") == "XBOT_DISABLED"
            assert await call(await path(*zone), "Activate", session) == (True, ErrorID.NONE)
            assert await wait(await path(*zone, "State"), "ACTIVATED_UNFENCED")
            flyway = ("PlanarTable", "Equipment", "PMC", "Flyways", "Flyway_0_0")
            assert await wait(await path(*flyway, "PowerConsumption"), 12.0)
            assert await read(*flyway, "MotorTemperature") == 30.5
            assert math.isnan(await read("PlanarTable", "Equipment", "PMC", "Flyways", "Flyway_3_2", "CpuTemperature"))

            assert await call(await path("PlanarShuttle1", "Occupation"), "Release", session) == (True, ErrorID.NONE)

            # The idle orientation: a multiple of 90 is taken, anything else reads back as it was.
            idle_rz = await path("PlanarTable", "Equipment", "IdleRz")

            async def taken(rz, timeout=3.0):  # the controller has it (a write lands on the node at once)
                t0 = time.monotonic()
                while time.monotonic() - t0 < timeout:
                    if ctl.coordinator.idle_rz == rz:
                        return True
                    await asyncio.sleep(0.05)
                return False

            assert math.isnan(await idle_rz.read_value())
            await idle_rz.write_value(ua.Variant(-90.0, ua.VariantType.Double))
            assert await taken(math.radians(-90))
            await idle_rz.write_value(ua.Variant(45.0, ua.VariantType.Double))
            await asyncio.sleep(0.3)
            assert await read("PlanarTable", "Equipment", "IdleRz") == -90.0, "refused: it reads back as it was"
            assert ctl.coordinator.idle_rz == math.radians(-90)
            await idle_rz.write_value(ua.Variant(math.nan, ua.VariantType.Double))
            assert await taken(None)
            assert not sim.collisions
    finally:
        serving.cancel()
        ctl.shutdown()
