# Examples

These examples are Python with [asyncua](https://github.com/FreeOpcUa/opcua-asyncio)
(`pip install asyncua`), but any OPC UA client does the same calls. They have been run against
the controller on its simulated table (see [Testing](testing.md)). For a full product agent,
see `tests/line_scenario.py` in the repository, described [at the end](#the-test-scenario).

## A small client

The examples share this client: a node by its browse path, a method call that answers
`(accepted, error_id)`, and a wait for a variable to reach a value.

```python
import asyncio
import uuid

from asyncua import Client, ua

NAMESPACE = "urn:aau:smartproductionlab:planar"

# Module states (PackML) and skill states, as the controller numbers them
STOPPED, IDLE, EXECUTE, ABORTED, HELD = 2, 4, 6, 9, 11
RUNNING, SUCCEEDED, FAILED, SKILL_ABORTED = 1, 3, 4, 5
NOT_PERMITTED = 5


def text(value):
    return ua.Variant(value, ua.VariantType.String)


def number(value):
    return ua.Variant(float(value), ua.VariantType.Double)


class PlanarClient:
    """A thin layer over asyncua for the planar controller's address space."""

    def __init__(self, client, ns):
        self.client = client
        self.ns = ns

    @classmethod
    async def connect(cls, endpoint):
        client = Client(endpoint)
        await client.connect()
        return cls(client, await client.get_namespace_index(NAMESPACE))

    async def disconnect(self):
        await self.client.disconnect()

    async def node(self, *path):
        """The node at a browse path below Objects, e.g. node("PlanarTable", "Module", "State")."""
        return await self.client.nodes.objects.get_child([f"{self.ns}:{name}" for name in path])

    async def read(self, *path):
        return await (await self.node(*path)).read_value()

    async def call(self, path, method, *args):
        """Call a method of the object at path: (accepted, error_id)."""
        accepted, error = await (await self.node(*path)).call_method(f"{self.ns}:{method}", *args)
        return accepted, error

    async def wait_for(self, path, values, timeout=60.0):
        """Wait until the variable at path reads one of values; returns the value."""
        loop = asyncio.get_running_loop()
        deadline = loop.time() + timeout
        while True:
            value = await self.read(*path)
            if value in values:
                return value
            if loop.time() > deadline:
                raise TimeoutError(f"{'/'.join(path)} is {value}, not one of {values}")
            await asyncio.sleep(0.05)
```

Polling every 50 ms matches how often the controller refreshes its variables. It is also fast
enough to see a skill's Succeeded, which turns back to Idle after 1.5 s.

## Bring the table up

Reset and Start the table, and with it every shuttle. The table is not occupied here, so any
session, even an empty one, may command it.

```python
async def bring_up(planar, session=""):
    """Reset and Start the table, and with it every shuttle, unless it runs already."""
    table = ("PlanarTable", "Module")
    state = await planar.read(*table, "State")
    if state == ABORTED:
        await planar.call(table, "Clear", text(session))
        state = await planar.wait_for((*table, "State"), (STOPPED,))
    if state == STOPPED:
        accepted, error = await planar.call(table, "Reset", text(session))
        if not accepted:
            raise RuntimeError(f"Reset refused: ErrorID {error}")
        if await planar.wait_for((*table, "State"), (IDLE, STOPPED), timeout=180) == STOPPED:
            raise RuntimeError(f"Reset failed: ErrorID {await planar.read(*table, 'ErrorID')}")
        state = IDLE
    if state == IDLE:
        await planar.call(table, "Start", text(session))
        await planar.wait_for((*table, "State"), (EXECUTE,))
```

A Reset that fails returns the table to Stopped with the reason in `Module/ErrorID`, e.g.
MastershipLost (32) when PMT still has the PMC.

## Move a shuttle

Occupy a shuttle with a session of your own, run `MoveToPosition`, and release it:

```python
async def move(planar, shuttle, session, x, y, rz=0.0, timeout=300.0):
    """Run MoveToPosition and wait until it has ended; where the xbot ended, (x mm, y mm, rz deg)."""
    skill = (shuttle, "Skills", "MoveToPosition")
    accepted, error = await planar.call(skill, "Start", text(session), number(x), number(y), number(rz))
    if not accepted:
        raise RuntimeError(f"{shuttle}: Start refused, ErrorID {error}")
    state = await planar.wait_for((*skill, "State"), (SUCCEEDED, FAILED, SKILL_ABORTED), timeout)
    if state != SUCCEEDED:
        error = await planar.read(*skill, "ErrorID")
        raise RuntimeError(f"{shuttle}: MoveToPosition ended in state {state}, ErrorID {error}")
    return tuple([await planar.read(*skill, "Results", r) for r in ("X", "Y", "Rz")])


async def first_move(endpoint):
    planar = await PlanarClient.connect(endpoint)
    try:
        await bring_up(planar)
        session = str(uuid.uuid4())  # this client's session: every command it sends carries it
        shuttle = ("PlanarShuttle1", "Occupation")
        accepted, error = await planar.call(shuttle, "Occupy", text(session))
        if not accepted:
            raise RuntimeError(f"Occupy refused: ErrorID {error}")  # 5: another session holds it
        try:
            print("arrived at", await move(planar, "PlanarShuttle1", session, 780, 660, 90))
        finally:
            await planar.call(shuttle, "Release", text(session))
    finally:
        await planar.disconnect()


asyncio.run(first_move("opc.tcp://127.0.0.1:4840/"))
```

```text
arrived at (780.0, 660.0, 90.0)
```

The move ends turned to 90 degrees: the xbot drove to a flyway centre on the way, turned there
and drove on. The client asks only for the pose; how the xbot gets there, past the others, is
the controller's business ([The planner](planner.md)).

## Take the next shuttle from the loading queue

With a loading station (or a line layout's idle area), the free shuttles wait in a queue, and
`PlanarTable/Equipment/LoadingQueue` names them, the one on the station first. A product agent
occupies the first it can get. Several agents may try at once, and Occupy settles who gets
which.

```python
async def shuttles(planar):
    """The shuttles the controller offers: PlanarShuttle1, PlanarShuttle2, ..."""
    names = []
    for child in await planar.client.nodes.objects.get_children():
        name = (await child.read_browse_name()).Name
        if name.startswith("PlanarShuttle"):
            names.append(name)
    return sorted(names, key=lambda n: int(n.removeprefix("PlanarShuttle")))


async def occupy_next(planar, session, all_shuttles):
    """Occupy the shuttle at the head of the loading queue, else any free one; its name."""
    while True:
        queue = (await planar.read("PlanarTable", "Equipment", "LoadingQueue")).split()
        for shuttle in queue + [s for s in all_shuttles if s not in queue]:
            accepted, error = await planar.call((shuttle, "Occupation"), "Occupy", text(session))
            if accepted:
                return shuttle
            if error != NOT_PERMITTED:  # NotPermitted: another session holds it
                raise RuntimeError(f"Occupy {shuttle}: ErrorID {error}")
        await asyncio.sleep(0.2)  # every shuttle is taken: try again
```

When the head of the queue is taken, the queue closes up behind it.

## A product through the line

A product agent occupies a shuttle, takes it from station to station, and releases it at the
end. Each station is a pose: X and Y in mm, Rz in degrees. Several agents run at once, here on
one connection:

```python
STATIONS = {  # where each module takes the shuttle: x mm, y mm, rz deg
    "Filling": (300, 660, 90),
    "Stoppering": (900, 300, 0),
    "Capping": (540, 60, -90),
    "Outfeed": (60, 180, 180),
}


async def make_product(planar, all_shuttles, process):
    session = str(uuid.uuid4())
    shuttle = await occupy_next(planar, session, all_shuttles)
    try:
        for station, (x, y, rz) in STATIONS.items():
            await move(planar, shuttle, session, x, y, rz)
            await process(station, shuttle)  # the shuttle stays where it is meanwhile
    finally:
        await planar.call((shuttle, "Occupation"), "Release", text(session))
    return shuttle


async def run_line(endpoint, products=3):
    planar = await PlanarClient.connect(endpoint)
    try:
        await bring_up(planar)
        names = await shuttles(planar)

        async def process(station, shuttle):
            print(f"{station} works on the product on {shuttle}")
            await asyncio.sleep(1.0)

        done = await asyncio.gather(*(make_product(planar, names, process) for _ in range(products)))
        print("products made on", done)
    finally:
        await planar.disconnect()
```

```text
Filling works on the product on PlanarShuttle3
Filling works on the product on PlanarShuttle2
Stoppering works on the product on PlanarShuttle3
Filling works on the product on PlanarShuttle1
Capping works on the product on PlanarShuttle3
...
products made on ['PlanarShuttle3', 'PlanarShuttle2', 'PlanarShuttle1']
```

The agents send their shuttles to the same stations without coordinating. The controller serves
requests for the same place in the order they were made: a later shuttle waits nearby until the
one before it has left. An occupied shuttle that is not moving stays where it is while the
station works on it; the planner never pushes it aside. Only reserve a station in your agents
if the station itself needs it.

## Carry a filled vial gently

A station that fills the vial tells the PMC the payload and keeps the shuttle slow until the
vial leaves it. The occupant sets the payload; whoever may command the shuttle sets its speed.

```python
async def carry_gently(planar, shuttle, session, x, y, rz=0.0):
    xbot = (shuttle, "Equipment", "Xbot")
    # 250 g with its centre of gravity 30 mm above the xbot; at most 0.2 m/s and 0.5 m/s^2
    await planar.call(xbot, "SetPayload", text(session), number(0.25), number(30))
    await planar.call(xbot, "SetSpeed", text(session), number(0.2), number(0.5))
    print("moves at", await planar.read(*xbot, "Speed"), "m/s,", await planar.read(*xbot, "Acceleration"), "m/s^2")
    await move(planar, shuttle, session, x, y, rz)
    # The vial leaves the shuttle: no payload, and the table's speed again
    await planar.call(xbot, "SetPayload", text(session), number(0), number(0))
    await planar.call(xbot, "SetSpeed", text(session), number(0), number(0))
```

```text
moves at 0.2 m/s, 0.5 m/s^2
```

The limits stay through releases and Resets, so set them back when the product leaves the
shuttle. See [Speed and payload](speed-and-payload.md).

## Hold the table

Hold pauses every xbot where it is, in mid-move. The skills stay Running, and a skill Start is
refused with NotReady (4) until the Unhold. Unhold carries on with the same moves.

```python
async def hold_for(planar, seconds, session=""):
    table = ("PlanarTable", "Module")
    await planar.call(table, "Hold", text(session))
    await planar.wait_for((*table, "State"), (HELD,))
    await asyncio.sleep(seconds)  # nothing moves; the skills stay Running
    await planar.call(table, "Unhold", text(session))
    await planar.wait_for((*table, "State"), (EXECUTE,))
```

A shuttle's own `Module/Hold` holds only that shuttle, if its occupant commands it.

## Stop or abort a move

```python
async def stop_move(planar, shuttle, session):
    skill = (shuttle, "Skills", "MoveToPosition")
    await planar.call(skill, "Stop", text(session))  # ends the motion it is on, then stops
    await planar.wait_for((*skill, "State"), (FAILED,))  # ErrorID 7, Interrupted


async def abort_move(planar, shuttle, session):
    skill = (shuttle, "Skills", "MoveToPosition")
    await planar.call(skill, "Abort", text(session))  # Aborted at once: the xbot stops where it is
    await planar.call(skill, "Reset", text(session))  # Aborted -> Idle, ready for the next Start
```

Stop is orderly: the xbot ends the motion it is on and stays there. A motion may be a straight
run across several slots. In the test, a shuttle sent from (900, 60) to (60, 660) and stopped
early ended at (900, 660), the end of its first straight run. `Results` say where it ended.
Abort stops the xbot at once, wherever it is, perhaps between slots. Once its shuttle is
released, a free xbot left between slots steps back onto the nearest slot.

## Recover a shuttle after a fault

When the PMC faults an xbot, e.g. `XBOT_OBSTACLE_DETECTED`, the move fails with XbotFault (21).
The shuttle then refuses Starts with that ErrorID until it is Stopped, Reset and Started again.
Its Resetting checks the xbot afresh. The occupant keeps the shuttle throughout.

```python
async def recover(planar, shuttle, session):
    """After a MoveToPosition failed with 21, 22, 23, 28, 29 or 30 the shuttle refuses new Starts
    with that ErrorID until it is Stopped, Reset and Started again."""
    module = (shuttle, "Module")
    await planar.call(module, "Stop", text(session))
    await planar.wait_for((*module, "State"), (STOPPED,))
    await planar.call(module, "Reset", text(session))
    if await planar.wait_for((*module, "State"), (IDLE, STOPPED), timeout=30) == STOPPED:
        raise RuntimeError(f"{shuttle}: Reset failed, ErrorID {await planar.read(*module, 'ErrorID')}")
    await planar.call(module, "Start", text(session))
    await planar.wait_for((*module, "State"), (EXECUTE,))
```

```text
failed with 21
start again: (False, 21)
recovered; moves again: (60.0, 420.0, 0.0)
```

If the Reset fails again with 21, the PMC still reports the xbot in a fault; look at its
`XbotState` and the PMC.

## Take xbots off through a zone

With `ZONES` set, an operator can take xbots off the table, or put new ones on, inside a zone
while the rest of the line goes on:

```python
async def service(planar, zone, session=""):
    path = ("PlanarTable", "Equipment", "Zones", zone)
    accepted, error = await planar.call(path, "Deactivate", text(session))
    if not accepted:
        raise RuntimeError(f"Deactivate {zone}: ErrorID {error}")  # 6: an xbot on its way in, out or across
    await planar.wait_for((*path, "State"), ("LOADING_FENCED",))
    inside = await planar.read(*path, "Xbots")
    await asyncio.to_thread(input, f"xbots {inside} are off: take them off or put new ones on, then Enter ")
    await planar.call(path, "Activate", text(session))
    await planar.wait_for((*path, "State"), ("ACTIVATED_UNFENCED",))
```

While the zone is off, the planner keeps every xbot out of it, and a target inside is refused
with ZoneClosed (33). See [Zones](zones.md).

## Turn the free shuttles

`PlanarTable/Equipment/IdleRz` is writable: every free shuttle turns to it, on the nearest clear
flyway centre.

```python
async def face_west(planar):
    node = await planar.node("PlanarTable", "Equipment", "IdleRz")
    await node.write_value(ua.Variant(180.0, ua.VariantType.Double))  # NaN: leave them as they are
```

A value that is not a multiple of 90 is not taken; the variable then reads back as it was.

## Read the table and where the shuttles are

```python
async def describe(planar):
    pmc = ("PlanarTable", "Equipment", "PMC")
    cols, rows, size, holes = [await planar.read(*pmc, k) for k in ("Columns", "Rows", "FlywaySize", "Holes")]
    print(f"{cols} x {rows} flyways of {size:g} mm; missing: {holes or 'none'}")
    for shuttle in await shuttles(planar):
        xbot = (shuttle, "Equipment", "Xbot")
        x, y, rz, state = [await planar.read(*xbot, k) for k in ("X", "Y", "Rz", "XbotState")]
        print(f"{shuttle}: ({x:.1f}, {y:.1f}) mm, {rz:.1f} deg, {state}")
```

On Planar Motor's simulator it prints, for instance:

```text
4 x 3 flyways of 240 mm; missing: 2,1
PlanarShuttle1: (780.0, 660.0) mm, 90.0 deg, XBOT_IDLE
PlanarShuttle2: (60.0, 60.0) mm, 0.0 deg, XBOT_IDLE
PlanarShuttle3: (60.0, 180.0) mm, 0.0 deg, XBOT_IDLE
```

The layout is 0 and empty, and the poses NaN, until the table's first Reset.

## Subscribe instead of polling

A subscription has the server report changes instead of the client asking. Here every shuttle's
skill state:

```python
class SkillWatcher:
    """asyncua calls datachange_notification for every change of a subscribed variable."""

    def __init__(self, names):
        self.names = names  # node id -> shuttle name

    def datachange_notification(self, node, value, data):
        print(f"{self.names[node.nodeid]}: MoveToPosition state {value}")


async def watch_skills(planar, seconds):
    names = {}
    for shuttle in await shuttles(planar):
        names[(await planar.node(shuttle, "Skills", "MoveToPosition", "State")).nodeid] = shuttle
    subscription = await planar.client.create_subscription(100, SkillWatcher(names))  # ms
    await subscription.subscribe_data_change([planar.client.get_node(n) for n in names])
    await asyncio.sleep(seconds)
    await subscription.delete()
```

```text
PlanarShuttle1: MoveToPosition state 0
PlanarShuttle2: MoveToPosition state 0
PlanarShuttle3: MoveToPosition state 0
PlanarShuttle1: MoveToPosition state 1
PlanarShuttle1: MoveToPosition state 3
PlanarShuttle1: MoveToPosition state 0
```

## The test scenario

`tests/line_scenario.py` is a production line driven over OPC UA only, as product agents would
drive it. Each agent has its own client and session. Per product it occupies the first free
shuttle in the loading queue and takes it through Infeed, Filling, Stoppering, Capping and
Outfeed. A step may fail, at random; the product then goes straight to the Outfeed. The agent
releases the shuttle there. Meanwhile it checks everything it sees: every answer, every move
ending at its target, and no shuttle moving while a station works on it. It also watches all
shuttles: no two overlap, every turn is on a turn spot, and in line mode every step goes the
lane's way.

```bash
python tests/line_scenario.py --endpoint opc.tcp://127.0.0.1:4840/ --agents 3 --products 3 --fail 0.2
python tests/line_scenario.py --layout config/layouts/clockwise-outwards   # its stations, for line mode
python tests/line_scenario.py --hold-every 8      # and hold the table every 8 s for 2 s
```

It prints each product and a summary, and exits 1 on any problem. See [Testing](testing.md).
