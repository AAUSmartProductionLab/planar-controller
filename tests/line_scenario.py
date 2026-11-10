"""Exercise a production line through OPC UA with concurrent product agents and motion checks."""

import argparse
import asyncio
import json
import os
import random
import sys
import time
import uuid
from dataclasses import dataclass, field

from asyncua import Client, ua

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # planar, for the checks
from planar.layout import Layout  # noqa: E402
from planar.lines import Lines, parse  # noqa: E402

NAMESPACE = "urn:aau:smartproductionlab:planar"

# (X mm, Y mm, Rz deg) on the 4 x 3 flyway table with the hole at flyway (2, 1), clockwise.
STATIONS = {
    "Infeed": (60.0, 180.0, 0.0),        # left edge, on a slot
    "Filling": (240.0, 660.0, 0.0),      # top edge, between two slots
    "Stoppering": (600.0, 660.0, 90.0),  # top edge, between two slots, turned
    "Capping": (840.0, 360.0, 0.0),      # near the right edge, on a flyway centre
    "Outfeed": (660.0, 60.0, 0.0),       # bottom edge, on a slot
}
ROUTE = ["Infeed", "Filling", "Stoppering", "Capping", "Outfeed"]
PROCESS_STEPS = {"Filling", "Stoppering", "Capping"}  # the ones that can fail

SUCCEEDED, FAILED = 3, 4
EXECUTE, HELD = 6, 11
HOLD_S = 2.0


def _s(value):
    """Wrap a value as an OPC UA String argument."""
    return ua.Variant(value, ua.VariantType.String)


def _d(value):
    """Wrap a numeric value as an OPC UA Double argument."""
    return ua.Variant(float(value), ua.VariantType.Double)


@dataclass
class Product:
    """Track a product's assigned shuttle, completed steps and final outcome."""
    agent: str
    name: str
    shuttle: str = ""
    outcome: str = ""
    steps: list = field(default_factory=list)  # (station, seconds to get there)


@dataclass
class Report:
    """Collect product outcomes, command refusals and observed motion problems."""
    products: list = field(default_factory=list)
    problems: list = field(default_factory=list)
    occupy_refusals: int = 0
    overlaps: list = field(default_factory=list)
    lane_steps: int = 0                         # line mode: steps from a lane slot to the next
    turns: list = field(default_factory=list)   # (shuttle, x mm, y mm, degrees turned, + counterclockwise)
    holds: int = 0                              # --hold-every: the table held and unheld
    hold_refusals: int = 0                      # Starts refused (4) while it was held
    holding: bool = False

    def problem(self, text):
        """Record and print a scenario failure."""
        self.problems.append(text)
        print("PROBLEM", text, flush=True)


class Opc:
    """One client's view of the controller."""

    def __init__(self, client, ns):
        """Bind an OPC UA client and namespace with a node lookup cache."""
        self.client = client
        self.ns = ns
        self._nodes = {}

    async def node(self, *names):
        """Resolve and cache a node by its namespaced browse path."""
        if names not in self._nodes:
            self._nodes[names] = await self.client.nodes.objects.get_child([f"{self.ns}:{n}" for n in names])
        return self._nodes[names]

    async def read(self, *names):
        """Read a value from a namespaced browse path."""
        return await (await self.node(*names)).read_value()

    async def call(self, path, method, *args):
        """Call a namespaced method and return Boolean acceptance and integer ErrorID."""
        out = await (await self.node(*path)).call_method(f"{self.ns}:{method}", *args)
        return bool(out[0]), int(out[1])

    async def pose(self, shuttle):
        """Read a shuttle's XY millimetres and Rz degrees together."""
        nodes = [await self.node(shuttle, "Equipment", "Xbot", k) for k in ("X", "Y", "Rz")]
        return await self.client.read_values(nodes)


async def _open(endpoint):
    """Retry connection to the OPC UA endpoint and return its planar namespace wrapper."""
    client = Client(endpoint, timeout=10)
    for _ in range(60):
        try:
            await client.connect()
            break
        except Exception:
            await asyncio.sleep(0.5)
    else:
        raise ConnectionError(f"no OPC UA server at {endpoint}")
    return Opc(client, await client.get_namespace_index(NAMESPACE))


async def _wait_value(opc, path, wanted, timeout):
    """Poll until a node reaches one of the wanted values or the timeout expires."""
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if await opc.read(*path) in wanted:
            return True
        await asyncio.sleep(0.05)
    return False


async def bring_up(opc):
    """Reset and Start the table if it is not running yet (the table is not occupied)."""
    if await opc.read("PlanarTable", "Module", "State") == 6:
        return
    if await opc.read("PlanarTable", "Module", "State") == 9:  # Aborted
        await opc.call(("PlanarTable", "Module"), "Clear", _s(""))
    await opc.call(("PlanarTable", "Module"), "Reset", _s(""))
    if not await _wait_value(opc, ("PlanarTable", "Module", "State"), (4,), 60):
        raise RuntimeError("the table did not reset")
    await opc.call(("PlanarTable", "Module"), "Start", _s(""))
    if not await _wait_value(opc, ("PlanarTable", "Module", "State"), (6,), 10):
        raise RuntimeError("the table did not start")


async def shuttles_of(opc):
    """Return sorted shuttle module names discovered in the server's Objects folder."""
    names = []
    for child in await opc.client.nodes.objects.get_children():
        name = (await child.read_browse_name()).Name
        if name.startswith("PlanarShuttle"):
            names.append(name)
    return sorted(names)


class ProductAgent:
    """Run products through shared stations while occupying one shuttle per product."""
    def __init__(self, name, endpoint, shuttles, stations, report, rng, products, process_s, fail_p, places=None):
        """Configure the agent's endpoint, stations, workload and simulated process failures."""
        self.name = name
        self.endpoint = endpoint
        self.shuttles = shuttles
        self.stations = stations
        self.places = places or STATIONS  # where each station is on the table
        self.report = report
        self.rng = rng
        self.products = products
        self.process_s = process_s
        self.fail_p = fail_p

    async def run(self):
        """Connect, process the assigned products and disconnect in all cases."""
        opc = await _open(self.endpoint)
        try:
            for k in range(self.products):
                await self.make(opc, Product(self.name, f"{self.name}-P{k + 1}"))
        finally:
            await opc.client.disconnect()

    async def make(self, opc, product):
        """Occupy a shuttle, execute a product's route and release station and shuttle ownership."""
        self.report.products.append(product)
        session = str(uuid.uuid4())
        product.shuttle = await self.occupy(opc, session)
        holding = None
        try:
            i = 0
            while i < len(ROUTE):
                station = ROUTE[i]
                await self.stations[station].acquire()
                ok = await self.move(opc, product, session, station, leaving=holding)
                holding = station
                if not ok:
                    product.outcome = f"move to {station} failed"
                    return
                await self.process(opc, product, station)
                if station == "Outfeed":
                    product.outcome = product.outcome or "finished"
                    return
                if station in PROCESS_STEPS and self.rng.random() < self.fail_p:
                    product.outcome = f"discarded after {station}"
                    i = ROUTE.index("Outfeed")
                else:
                    i += 1
        finally:
            if holding:
                self.stations[holding].release()
            answer = await opc.call((product.shuttle, "Occupation"), "Release", _s(session))
            if answer != (True, 0):
                self.report.problem(f"{product.name}: Release {product.shuttle} answered {answer}")
            print(f"{product.name} on {product.shuttle}: {product.outcome} "
                  f"({' -> '.join(f'{s} {t:.1f}s' for s, t in product.steps)})", flush=True)

    async def occupy(self, opc, session):
        """The first free shuttle in the loading queue (the one on the station), else any free one."""
        while True:
            queue = (await opc.read("PlanarTable", "Equipment", "LoadingQueue")).split()
            rest = self.rng.sample([s for s in self.shuttles if s not in queue], len(self.shuttles) - len(queue))
            for shuttle in queue + rest:
                answer = await opc.call((shuttle, "Occupation"), "Occupy", _s(session))
                if answer == (True, 0):
                    return shuttle
                if answer == (False, 5):
                    self.report.occupy_refusals += 1
                else:
                    self.report.problem(f"Occupy {shuttle} answered {answer}")
            await asyncio.sleep(0.2)

    async def move(self, opc, product, session, station, leaving):
        """Start a station move, handle Hold refusals and record its completion or failure."""
        x, y, rz = self.places[station]
        skill = (product.shuttle, "Skills", "MoveToPosition")
        t0 = time.monotonic()
        while True:
            answer = await opc.call(skill, "Start", _s(session), _d(x), _d(y), _d(rz))
            if answer != (False, 4) or not self.report.holding:
                break
            self.report.hold_refusals += 1  # held: it starts once unheld
            await asyncio.sleep(0.2)
        if leaving:
            self.stations[leaving].release()  # the product is on its way out of it
        if answer != (True, 0):
            self.report.problem(f"{product.name}: Start to {station} answered {answer}")
            return False
        if await opc.read(*skill, "State") not in (1, SUCCEEDED):
            self.report.problem(f"{product.name}: State after Start is not Running")
        if not await _wait_value(opc, (*skill, "State"), (SUCCEEDED, FAILED), 180):
            self.report.problem(f"{product.name}: move to {station} did not end in 180 s")
            return False
        if await opc.read(*skill, "State") == FAILED:
            error = await opc.read(*skill, "ErrorID")
            self.report.problem(f"{product.name}: move to {station} Failed with ErrorID {error}")
            return False
        rx, ry, rrz = [await opc.read(*skill, "Results", k) for k in ("X", "Y", "Rz")]
        drz = abs((rrz - rz + 180.0) % 360.0 - 180.0)
        if abs(rx - x) > 0.5 or abs(ry - y) > 0.5 or drz > 0.3:
            self.report.problem(f"{product.name}: at {station} ended at ({rx}, {ry}, {rrz})")
        product.steps.append((station, time.monotonic() - t0))
        return True

    async def process(self, opc, product, station):
        """The station works on the product: the shuttle must not move meanwhile."""
        before = await opc.pose(product.shuttle)
        await asyncio.sleep(self.rng.uniform(*self.process_s))
        after = await opc.pose(product.shuttle)
        if any(abs(a - b) > 0.1 for a, b in zip(before, after)):
            self.report.problem(f"{product.name}: {product.shuttle} moved at {station}: {before} -> {after}")


class MotionCheck:
    """Detect travel against lane directions and rotations outside permitted spots or angles."""

    SETTLED_S = 0.3  # a turn is over when the angle has not changed for this long
    # Less is not a turn (every turn is to a multiple of 90 deg): Planar Motor's simulator's xbot
    # yaws by a degree or so now and then on a diagonal move.
    JITTER_DEG = 5.0

    def __init__(self, table, lines, report):
        """Build allowed lane transitions and initialize per-shuttle pose history."""
        self.table, self.lines, self.report = table, lines, report
        self.forward = {(a, b) for _, cells in lines.lanes for a, b in zip(cells, cells[1:])} if lines else set()
        self.cell, self.rz, self.turning = {}, {}, {}

    def _slot(self, x, y):
        """Resolve millimetre coordinates to a cell centre within the sampling tolerance."""
        pitch = self.table.pitch * 1000.0
        cell = (int(x // pitch), int(y // pitch))
        on = abs(x - (cell[0] + 0.5) * pitch) < 5.0 and abs(y - (cell[1] + 0.5) * pitch) < 5.0
        return cell if on else None

    def _on_spot(self, x, y):
        """Whether millimetre coordinates lie at a permitted rotation spot."""
        if self.lines and self.lines.turn_spots:
            return any(abs(sx * 1000.0 - x) < 3.0 and abs(sy * 1000.0 - y) < 3.0 for sx, sy in self.lines.turn_spots)
        return self.table.spot_at(x / 1000.0, y / 1000.0, tolerance=0.003) is not None

    def sample(self, shuttle, pose, now):
        """Check a sampled pose against prior motion and record lane or rotation violations."""
        x, y, rz = pose
        cell = self._slot(x, y)
        last = self.cell.get(shuttle)
        if self.lines and cell and last and last != cell and abs(last[0] - cell[0]) + abs(last[1] - cell[1]) == 1 \
                and {last, cell} <= self.lines.lane_cells:
            self.report.lane_steps += 1
            if (last, cell) not in self.forward:
                self.report.problem(f"{shuttle} went from slot {last} to {cell}, against the lane")
        if cell:
            self.cell[shuttle] = cell
        if shuttle in self.rz:
            step = (rz - self.rz[shuttle] + 180.0) % 360.0 - 180.0
            if abs(step) > 0.05:
                turn = self.turning.setdefault(shuttle, [x, y, 0.0, now])
                turn[2] += step
                turn[3] = now
            elif shuttle in self.turning and self.report.holding:
                self.turning[shuttle][3] = now  # paused half way: it goes on once unheld
            elif shuttle in self.turning and now - self.turning[shuttle][3] > self.SETTLED_S:
                tx, ty, swept, _ = self.turning.pop(shuttle)
                if abs(swept) >= self.JITTER_DEG:
                    self.report.turns.append((shuttle, round(tx), round(ty), round(swept)))
                    if not self._on_spot(tx, ty):
                        self.report.problem(f"{shuttle} turned at ({tx:.0f}, {ty:.0f}) mm, not on a turn spot")
                    if abs(swept) > 180.5:
                        self.report.problem(f"{shuttle} turned {swept:+.0f} deg at ({tx:.0f}, {ty:.0f}) mm, "
                                            f"the long way")
        self.rz[shuttle] = rz


async def motion_check(opc, report):
    """The check for the table the controller has (connected, so after its Reset)."""
    pmc = ("PlanarTable", "Equipment", "PMC")
    holes = frozenset(tuple(int(v) for v in h.split(",")) for h in (await opc.read(*pmc, "Holes")).split())
    table = Layout(await opc.read(*pmc, "Columns"), await opc.read(*pmc, "Rows"), holes,
                   await opc.read(*pmc, "FlywaySize") / 1000.0)
    lines = None
    if await opc.read("PlanarTable", "Equipment", "Mode") == "line":
        lines = Lines(parse(await opc.read("PlanarTable", "Equipment", "LineLayout")), table)
    return MotionCheck(table, lines, report)


async def watch(endpoint, shuttles, report, stop, check=None):
    """Sample shuttle poses, detect footprint overlaps and apply optional lane and rotation checks."""
    opc = await _open(endpoint)
    try:
        while not stop.is_set():
            poses = {s: await opc.pose(s) for s in shuttles}
            if check:
                now = time.monotonic()
                for s, pose in poses.items():
                    check.sample(s, pose, now)
            names = sorted(poses)
            for a in range(len(names)):
                for b in range(a + 1, len(names)):
                    pa, pb = poses[names[a]], poses[names[b]]
                    square = all(abs(((p[2] + 45) % 90) - 45) < 0.5 for p in (pa, pb))
                    reach = 119.5 if square else 145.0
                    if abs(pa[0] - pb[0]) < reach and abs(pa[1] - pb[1]) < reach:
                        report.overlaps.append((names[a], names[b], pa, pb))
            await asyncio.sleep(0.05)
    finally:
        await opc.client.disconnect()


async def holder(endpoint, shuttles, report, stop, every):
    """Hold the table every few seconds for HOLD_S, then unhold it: nothing may move meanwhile."""
    opc = await _open(endpoint)
    module = ("PlanarTable", "Module")
    try:
        while not stop.is_set():
            try:
                await asyncio.wait_for(stop.wait(), every)
                break
            except asyncio.TimeoutError:
                pass
            report.holding = True
            answer = await opc.call(module, "Hold", _s(""))
            if answer != (True, 0):
                report.problem(f"Hold answered {answer}")
                report.holding = False
                continue
            if not await _wait_value(opc, (*module, "State"), (HELD,), 3):
                report.problem("the table was not Held 3 s after Hold")
            await asyncio.sleep(0.3)  # a move's deceleration is over
            before = {s: await opc.pose(s) for s in shuttles}
            await asyncio.sleep(HOLD_S)
            for s in shuttles:
                after = await opc.pose(s)
                if any(abs(a - b) > 0.5 for a, b in zip(before[s], after)):
                    report.problem(f"{s} moved while the table was held: {before[s]} -> {after}")
            answer = await opc.call(module, "Unhold", _s(""))
            if answer != (True, 0):
                report.problem(f"Unhold answered {answer}")
            for s in shuttles:
                if not await _wait_value(opc, (s, "Module", "State"), (EXECUTE,), 3):
                    report.problem(f"{s} was not in Execute 3 s after Unhold")
            report.holding = False
            report.holds += 1
    finally:
        await opc.client.disconnect()


async def run_line(endpoint, agents=3, products=3, process_s=(0.5, 1.5), fail_p=0.2, seed=1, sample=True,
                   places=None, hold_every=None):
    """Run concurrent product agents with optional sampling and periodic Holds; return the scenario report.
    Station poses use mm and degrees.
    """
    places = dict(places or STATIONS)
    report = Report()
    opc = await _open(endpoint)
    try:
        await bring_up(opc)
        shuttles = await shuttles_of(opc)
        check = await motion_check(opc, report) if sample else None
    finally:
        await opc.client.disconnect()
    stations = {name: asyncio.Lock() for name in places}
    workers = [ProductAgent(f"Agent{k + 1}", endpoint, shuttles, stations, report, random.Random(seed * 100 + k),
                            products, process_s, fail_p, places) for k in range(agents)]
    stop = asyncio.Event()
    watcher = asyncio.create_task(watch(endpoint, shuttles, report, stop, check)) if sample else None
    holding = asyncio.create_task(holder(endpoint, shuttles, report, stop, hold_every)) if hold_every else None
    try:
        await asyncio.gather(*(w.run() for w in workers))
    finally:
        stop.set()
        for task in (watcher, holding):
            if task:
                await task
    if report.overlaps:
        report.problem(f"{len(report.overlaps)} sampled overlaps, first {report.overlaps[0]}")
    return report


def summary(report):
    """Format product outcomes, motion timings and detected problems for the console."""
    finished = sum(p.outcome == "finished" for p in report.products)
    discarded = sum(p.outcome.startswith("discarded") for p in report.products)
    moves = [t for p in report.products for _, t in p.steps]
    turned = {}
    for *_, degrees in report.turns:
        turned[degrees] = turned.get(degrees, 0) + 1
    return (f"{len(report.products)} products: {finished} finished, {discarded} discarded; "
            f"{len(moves)} moves, mean {sum(moves) / max(1, len(moves)):.1f}s, longest {max(moves, default=0):.1f}s; "
            f"{report.occupy_refusals} occupy refusals (5); {report.lane_steps} lane steps; "
            f"turns {', '.join(f'{n}x {d:+d} deg' for d, n in sorted(turned.items())) or 'none'}; "
            + (f"{report.holds} holds ({report.hold_refusals} Starts refused while held); " if report.holds else "")
            + f"{len(report.problems)} problems")


def read_stations(path):
    """{"<station>": (x mm, y mm, rz deg)}, every station of ROUTE there."""
    with open(path, encoding="utf-8") as f:
        places = {name: tuple(float(v) for v in pose) for name, pose in json.load(f).items()}
    missing = [name for name in ROUTE if name not in places]
    if missing:
        raise SystemExit(f"{path}: no {', '.join(missing)} (the stations are {', '.join(ROUTE)})")
    return places


def main():
    """Parse scenario options, load station positions and run the OPC UA production-line test."""
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--endpoint", default="opc.tcp://127.0.0.1:4840/")
    p.add_argument("--agents", type=int, default=3)
    p.add_argument("--products", type=int, default=3, help="per agent")
    p.add_argument("--fail", type=float, default=0.2, help="chance a process step fails")
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--stations", help='a JSON file of {"<station>": [x, y, rz]} instead of STATIONS')
    p.add_argument("--layout", metavar="DIR", help="a folder of config/layouts/: its stations.json")
    p.add_argument("--hold-every", type=float, metavar="S", help="hold the table every S seconds, for 2 s")
    args = p.parse_args()
    places = None
    stations = args.stations or (os.path.join(args.layout, "stations.json") if args.layout else None)
    if stations:
        places = read_stations(stations)
    report = asyncio.run(run_line(args.endpoint, args.agents, args.products, fail_p=args.fail, seed=args.seed,
                                  places=places, hold_every=args.hold_every))
    print(summary(report))
    sys.exit(1 if report.problems else 0)


if __name__ == "__main__":
    main()
