"""
The OPC UA interface, laid out like a generated module's (work.md, "A generated module"):

  /Objects/<Module>/Occupation      Occupy(Session), Release(Session); Occupied
  /Objects/<Module>/Module          Reset, Start, Hold, Unhold, Stop, Abort, Clear (Session);
                                    State, ErrorID
  /Objects/<Module>/Skills/<Skill>  Start(Session, <parameters>), Stop, Abort, Reset (Session);
                                    State, ErrorID, Parameters/<p>, Results/<r>
  /Objects/<Module>/Equipment/<Item>  the table's PMC: Connected, the layout (Columns, Rows,
                                    FlywaySize, Holes) and Flyways/Flyway_<c>_<r> (power and
                                    temperatures); its LoadingQueue, IdleRz (writable), Mode,
                                    LineLayout and Zones/<name> (State, Xbots; Deactivate,
                                    Activate); a shuttle's Xbot: X, Y, Rz, XbotState, the payload
                                    (PayloadKg, PayloadCgHeight, MaxAcceleration; SetPayload)

Every method answers [Accepted, ErrorID] at once. A MoveToPosition that was accepted finishes
when the xbot has arrived: its State turns Succeeded (3), or Failed (4) with an ErrorID.

Method calls run the controller's code in a worker thread, so a plan being made never holds up
the server. Variables are refreshed from the controller every 50 ms and written when they change,
and once more before a method answers: a client that reads State after Start sees Running, never
the Succeeded of the run before.
"""

import asyncio
import logging
import math

from asyncua import Server, ua

log = logging.getLogger(__name__)

NAMESPACE = "urn:aau:smartproductionlab:planar"


def _argument(name, type_id, description=""):
    a = ua.Argument()
    a.Name = name
    a.DataType = ua.NodeId(type_id)
    a.ValueRank = -1
    a.ArrayDimensions = []
    a.Description = ua.LocalizedText(description)
    return a


SESSION = _argument("Session", ua.ObjectIds.String, "The occupation's session ID (a UUID the client chose)")
ANSWER = [
    _argument("Accepted", ua.ObjectIds.Boolean, "Whether the command was taken"),
    _argument("ErrorID", ua.ObjectIds.Int32, "Why not: 4 NotReady, 5 NotPermitted, 6 Busy, 8 OutOfRange, ..."),
]


class OpcUaServer:
    def __init__(self, controller, endpoint="opc.tcp://0.0.0.0:4840/", name="Planar table", namespace=NAMESPACE):
        self.ctl = controller
        self.endpoint = endpoint
        self.name = name
        self.namespace = namespace
        self.server = None
        self.idx = None
        self._publish = []  # (node, read() -> value, VariantType)
        self._writable = []  # (node, write(value))
        self._last = {}
        self._refreshing = asyncio.Lock()
        self._flyways = None   # Equipment/PMC/Flyways, its flyways added once the layout is known
        self._flyway_nodes = set()

    async def init(self):
        self.server = Server()
        await self.server.init()
        self.server.set_endpoint(self.endpoint)
        self.server.set_server_name(self.name)
        self.server.set_security_policy([ua.SecurityPolicyType.NoSecurity])
        self.idx = await self.server.register_namespace(self.namespace)
        objects = self.server.nodes.objects
        await self._module(objects, self.ctl.table, equipment=self._table_equipment)
        for xid, shuttle in self.ctl.shuttles.items():
            await self._module(objects, shuttle, equipment=lambda obj, xid=xid: self._shuttle_equipment(obj, xid))

    async def run(self):
        """Serve until cancelled."""
        if self.server is None:
            await self.init()
        async with self.server:
            log.info("OPC UA server on %s", self.endpoint)
            while True:
                await self._refresh()
                await asyncio.sleep(0.05)

    # --- building the address space ----------------------------------------------------------

    async def _module(self, parent, module, equipment):
        idx = self.idx
        obj = await parent.add_object(idx, module.name)

        occupation = await obj.add_object(idx, "Occupation")
        await self._method(occupation, "Occupy", module.occupation.occupy, [SESSION])
        await self._method(occupation, "Release", module.occupation.release, [SESSION])
        await self._variable(occupation, "Occupied", lambda m=module: m.occupation.occupied, ua.VariantType.Boolean)

        control = await obj.add_object(idx, "Module")
        for command in ("Reset", "Start", "Hold", "Unhold", "Stop", "Abort", "Clear"):
            await self._method(control, command, lambda s, c=command, m=module: m.command(c, s), [SESSION])
        await self._variable(control, "State", lambda m=module: int(m.state), ua.VariantType.Int32)
        await self._variable(control, "ErrorID", lambda m=module: int(m.error), ua.VariantType.Int32)

        if module.skills:
            skills = await obj.add_object(idx, "Skills")
            for skill in module.skills.values():
                await self._skill(skills, skill)

        equipment_obj = await obj.add_object(idx, "Equipment")
        await equipment(equipment_obj)

    async def _skill(self, parent, skill):
        idx = self.idx
        obj = await parent.add_object(idx, skill.name)
        names = [p.name for p in skill.parameters]
        inputs = [SESSION] + [_argument(p.name, ua.ObjectIds.Double, f"{p.lower:g}..{p.upper:g} {p.unit}".strip())
                              for p in skill.parameters]

        def start(session, *values):
            return skill.start(session, dict(zip(names, (float(v) for v in values))))

        await self._method(obj, "Start", start, inputs)
        await self._method(obj, "Stop", skill.stop, [SESSION])
        await self._method(obj, "Abort", skill.abort, [SESSION])
        await self._method(obj, "Reset", skill.reset, [SESSION])
        await self._variable(obj, "State", lambda: int(skill.state), ua.VariantType.Int32)
        await self._variable(obj, "ErrorID", lambda: int(skill.error), ua.VariantType.Int32)
        parameters = await obj.add_object(idx, "Parameters")
        for p in skill.parameters:
            await self._variable(parameters, p.name, lambda n=p.name: float(skill.values[n]), ua.VariantType.Double)
        results = await obj.add_object(idx, "Results")
        for r in skill.result_names:
            await self._variable(results, r, lambda n=r: float(skill.results[n]), ua.VariantType.Double)

    async def _table_equipment(self, parent):
        pmc = await parent.add_object(self.idx, "PMC")
        await self._variable(pmc, "Connected", lambda: self.ctl.coordinator.connected, ua.VariantType.Boolean)
        # The layout as the PMC reports it, for a client that draws or describes the table: Columns by
        # Rows flyways of FlywaySize mm, without the Holes ("<column>,<row>" apart by spaces). Zero
        # and empty until the PMC has been connected.
        def layout(read, empty):
            return lambda: read(self.ctl.coordinator.layout) if self.ctl.coordinator.layout else empty

        await self._variable(pmc, "Columns", layout(lambda t: t.cols, 0), ua.VariantType.Int32)
        await self._variable(pmc, "Rows", layout(lambda t: t.rows, 0), ua.VariantType.Int32)
        await self._variable(pmc, "FlywaySize", layout(lambda t: t.flyway_size * 1000.0, 0.0), ua.VariantType.Double)
        await self._variable(pmc, "Holes", layout(lambda t: " ".join(f"{c},{r}" for c, r in sorted(t.holes)), ""),
                             ua.VariantType.String)
        # Flyway_<column>_<row>: PowerConsumption (W), CpuTemperature, AmplifierTemperature and
        # MotorTemperature (deg C), read every few seconds, NaN until they are; added at the first Reset.
        self._flyways = await pmc.add_object(self.idx, "Flyways")

        # The free shuttles waiting at the loading station, the one on it first, apart by spaces:
        # the one to occupy for the next product. Empty without a loading station.
        def queue():
            names = {xid: s.name for xid, s in self.ctl.shuttles.items()}
            return " ".join(names[x] for x in self.ctl.coordinator.queue_order() if x in names)

        await self._variable(parent, "LoadingQueue", queue, ua.VariantType.String)

        # The orientation idle shuttles are turned to, degrees: writable, a multiple of 90 or NaN
        # (as they are); anything else is not taken and the value reads back as it was.
        def idle_rz():
            rz = self.ctl.coordinator.idle_rz
            return math.nan if rz is None else math.degrees(rz)

        def set_idle_rz(deg):
            error = self.ctl.coordinator.set_idle_rz(math.radians(deg))
            log.info("IdleRz %s: %s", deg, "set" if error == 0 else "refused (a multiple of 90 or NaN)")

        await self._variable(parent, "IdleRz", idle_rz, ua.VariantType.Double, write=set_idle_rz)

        # free or line (MODE); in line mode the layout's JSON as it was read (LINE_LAYOUT, read
        # again at every Reset), and why the last Reset did not take it as it is now (empty: it did).
        coordinator = self.ctl.coordinator
        await self._variable(parent, "Mode", lambda: "line" if coordinator.line_spec else "free",
                             ua.VariantType.String)
        await self._variable(parent, "LineLayout", lambda: coordinator.line_spec.text if coordinator.line_spec else "",
                             ua.VariantType.String)
        await self._variable(parent, "LineLayoutProblem", lambda: coordinator.line_problem, ua.VariantType.String)

        # The zones of ZONES: the PMC's state of each (UNKNOWN until the first Reset;
        # ACTIVATED_UNFENCED: part of the table; LOADING_FENCED: switched off, xbots may be taken
        # off or put on) and the xbots inside; Deactivate and Activate switch it off and on.
        await self._variable(parent, "ZoneProblem", lambda: coordinator.zone_problem, ua.VariantType.String)
        zones = await parent.add_object(self.idx, "Zones")
        for spec in coordinator.zone_specs:
            name = spec.name
            obj = await zones.add_object(self.idx, name)

            def zone(read, name=name):
                return lambda: read(coordinator.zones[name]) if name in coordinator.zones else read(None)

            await self._variable(obj, "State", zone(lambda z: z.state if z else "UNKNOWN"), ua.VariantType.String)
            await self._variable(obj, "Xbots", zone(lambda z: " ".join(str(x) for x in z.xbots) if z else ""),
                                 ua.VariantType.String)
            for command in ("Deactivate", "Activate"):
                await self._method(obj, command, lambda s, c=command, n=name: self.ctl.zone_command(n, c, s),
                                   [SESSION])

    async def _add_flyways(self):
        """The flyways' variables, once the layout is known (the first Reset)."""
        coordinator = self.ctl.coordinator
        with self.ctl.lock:
            flyways = list(coordinator.layout.flyways) if coordinator.layout else []
        for flyway in flyways:
            if flyway in self._flyway_nodes:
                continue
            self._flyway_nodes.add(flyway)
            obj = await self._flyways.add_object(self.idx, f"Flyway_{flyway[0]}_{flyway[1]}")
            for k, name in enumerate(("PowerConsumption", "CpuTemperature", "AmplifierTemperature", "MotorTemperature")):
                read = lambda f=flyway, k=k: float(coordinator.flyway_status[f][k]) if f in coordinator.flyway_status \
                    else math.nan
                await self._variable(obj, name, read, ua.VariantType.Double)

    async def _shuttle_equipment(self, parent, xid):
        xbot = await parent.add_object(self.idx, "Xbot")

        def reading(i):
            snap = self.ctl.coordinator.snapshot(xid)
            if snap is None:  # the PMC not read yet (it is at the first Reset)
                return math.nan if i < 3 else "UNKNOWN"
            pose, state, detail = snap
            return [pose.x * 1000.0, pose.y * 1000.0, math.degrees(pose.rz), detail or state][i]

        await self._variable(xbot, "X", lambda: float(reading(0)), ua.VariantType.Double)
        await self._variable(xbot, "Y", lambda: float(reading(1)), ua.VariantType.Double)
        await self._variable(xbot, "Rz", lambda: float(reading(2)), ua.VariantType.Double)
        await self._variable(xbot, "XbotState", lambda: str(reading(3)), ua.VariantType.String)

        # The payload as the PMC was told it (kg, its centre of gravity mm above the xbot) and the
        # acceleration the PMC allows with it (m/s^2; its moves keep to it): NaN while unknown.
        # SetPayload is the occupant's, while the shuttle's skill does not run.
        def payload(read):
            def value():
                agent = self.ctl.coordinator.agents.get(xid)
                v = read(agent) if agent is not None else None
                return math.nan if v is None else float(v)
            return value

        await self._variable(xbot, "PayloadKg", payload(lambda a: a.payload and a.payload[0]), ua.VariantType.Double)
        await self._variable(xbot, "PayloadCgHeight", payload(lambda a: a.payload and a.payload[1] * 1000.0),
                             ua.VariantType.Double)
        await self._variable(xbot, "MaxAcceleration", payload(lambda a: a.max_accel), ua.VariantType.Double)
        await self._method(xbot, "SetPayload",
                           lambda s, kg, cg: self.ctl.set_payload(xid, s, float(kg or 0.0), float(cg or 0.0)),
                           [SESSION, _argument("PayloadKg", ua.ObjectIds.Double, "kg"),
                            _argument("CgHeight", ua.ObjectIds.Double, "mm above the xbot")])

    async def _method(self, parent, name, fn, inputs):
        async def handler(_parent, *args):
            values = [a.Value if a.Value is not None else "" for a in args]
            try:
                accepted, error = await asyncio.to_thread(fn, *values)
            except Exception:
                log.exception("%s", name)
                accepted, error = False, 4
            await self._refresh()
            return [ua.Variant(bool(accepted), ua.VariantType.Boolean), ua.Variant(int(error), ua.VariantType.Int32)]

        await parent.add_method(self.idx, name, handler, inputs, ANSWER)

    async def _variable(self, parent, name, read, vtype, write=None):
        """A variable kept as read() says; with write, clients may write it: write(value) is
        called with the controller's lock held, and the variable then reads as read() says."""
        with self.ctl.lock:
            value = read()
        node = await parent.add_variable(self.idx, name, ua.Variant(value, vtype))
        self._publish.append((node, read, vtype))
        self._last[node.nodeid] = value
        if write is not None:
            await node.set_writable()
            self._writable.append((node, write))

    # --- keeping variables current -----------------------------------------------------------

    def _snapshot(self):
        with self.ctl.lock:
            return [read() for _, read, _ in self._publish]

    def _apply(self, write, value):
        with self.ctl.lock:
            write(value)

    async def _refresh(self):
        async with self._refreshing:  # one at a time, so an older snapshot never overwrites a newer
            if self._flyways is not None and self.ctl.coordinator.layout is not None \
                    and len(self._flyway_nodes) < len(self.ctl.coordinator.layout.flyways):
                await self._add_flyways()
            for node, write in self._writable:
                value = await node.read_value()
                if not _same(value, self._last.get(node.nodeid)):  # a client wrote it
                    self._last[node.nodeid] = value
                    await asyncio.to_thread(self._apply, write, value)
            values = await asyncio.to_thread(self._snapshot)
            for (node, _, vtype), value in zip(self._publish, values):
                if not _same(self._last.get(node.nodeid), value):
                    self._last[node.nodeid] = value
                    await node.write_value(ua.Variant(value, vtype))


def _same(a, b):
    return a == b or (isinstance(a, float) and isinstance(b, float) and math.isnan(a) and math.isnan(b))
