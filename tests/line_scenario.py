"""
A small production line driven through OPC UA only, the way product agents would drive it.

Five stations at or near the edge of the table, visited clockwise: Infeed, Filling, Stoppering,
Capping, Outfeed. Each product agent has its own OPC UA client and session. Per product it

1. occupies whichever shuttle is free (Occupy answers 5 while another session holds one),
2. takes it through the stations in order, holding each station while its step runs,
3. may see a process step fail; the product is then discarded and goes straight to the Outfeed,
4. releases the shuttle at the Outfeed and goes on with its next product.

The stations are reserved the way station modules would be (one product at a time), the moves
are left to the controller. Everything an agent sees is checked: every command's answer, every
move ending Succeeded at its target, and a shuttle standing still while a station works on it.

Against a running controller (python -m planar):

    python tests/line_scenario.py --endpoint opc.tcp://127.0.0.1:4840/ --agents 3 --products 3
"""

import argparse
import asyncio
import random
import sys
import time
import uuid
from dataclasses import dataclass, field

from asyncua import Client, ua

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


def _s(value):
    return ua.Variant(value, ua.VariantType.String)


def _d(value):
    return ua.Variant(float(value), ua.VariantType.Double)


@dataclass
class Product:
    agent: str
    name: str
    shuttle: str = ""
    outcome: str = ""
    steps: list = field(default_factory=list)  # (station, seconds to get there)


@dataclass
class Report:
    products: list = field(default_factory=list)
    problems: list = field(default_factory=list)
    occupy_refusals: int = 0
    overlaps: list = field(default_factory=list)

    def problem(self, text):
        self.problems.append(text)
        print("PROBLEM", text, flush=True)


class Opc:
    """One client's view of the controller."""

    def __init__(self, client, ns):
        self.client = client
        self.ns = ns
        self._nodes = {}

    async def node(self, *names):
        if names not in self._nodes:
            self._nodes[names] = await self.client.nodes.objects.get_child([f"{self.ns}:{n}" for n in names])
        return self._nodes[names]

    async def read(self, *names):
        return await (await self.node(*names)).read_value()

    async def call(self, path, method, *args):
        out = await (await self.node(*path)).call_method(f"{self.ns}:{method}", *args)
        return bool(out[0]), int(out[1])

    async def pose(self, shuttle):
        nodes = [await self.node(shuttle, "Equipment", "Xbot", k) for k in ("X", "Y", "Rz")]
        return await self.client.read_values(nodes)


async def _open(endpoint):
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
    names = []
    for child in await opc.client.nodes.objects.get_children():
        name = (await child.read_browse_name()).Name
        if name.startswith("PlanarShuttle"):
            names.append(name)
    return sorted(names)


class ProductAgent:
    def __init__(self, name, endpoint, shuttles, stations, report, rng, products, process_s, fail_p):
        self.name = name
        self.endpoint = endpoint
        self.shuttles = shuttles
        self.stations = stations
        self.report = report
        self.rng = rng
        self.products = products
        self.process_s = process_s
        self.fail_p = fail_p

    async def run(self):
        opc = await _open(self.endpoint)
        try:
            for k in range(self.products):
                await self.make(opc, Product(self.name, f"{self.name}-P{k + 1}"))
        finally:
            await opc.client.disconnect()

    async def make(self, opc, product):
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
        while True:
            for shuttle in self.rng.sample(self.shuttles, len(self.shuttles)):
                answer = await opc.call((shuttle, "Occupation"), "Occupy", _s(session))
                if answer == (True, 0):
                    return shuttle
                if answer == (False, 5):
                    self.report.occupy_refusals += 1
                else:
                    self.report.problem(f"Occupy {shuttle} answered {answer}")
            await asyncio.sleep(0.2)

    async def move(self, opc, product, session, station, leaving):
        x, y, rz = STATIONS[station]
        skill = (product.shuttle, "Skills", "MoveToPosition")
        t0 = time.monotonic()
        answer = await opc.call(skill, "Start", _s(session), _d(x), _d(y), _d(rz))
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


async def watch(endpoint, shuttles, report, stop):
    """Sample every shuttle's pose and note any two that overlap (axis-aligned squares)."""
    opc = await _open(endpoint)
    try:
        while not stop.is_set():
            poses = {s: await opc.pose(s) for s in shuttles}
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


async def run_line(endpoint, agents=3, products=3, process_s=(0.5, 1.5), fail_p=0.2, seed=1, sample=True):
    report = Report()
    opc = await _open(endpoint)
    try:
        await bring_up(opc)
        shuttles = await shuttles_of(opc)
    finally:
        await opc.client.disconnect()
    stations = {name: asyncio.Lock() for name in STATIONS}
    workers = [ProductAgent(f"Agent{k + 1}", endpoint, shuttles, stations, report, random.Random(seed * 100 + k),
                            products, process_s, fail_p) for k in range(agents)]
    stop = asyncio.Event()
    watcher = asyncio.create_task(watch(endpoint, shuttles, report, stop)) if sample else None
    try:
        await asyncio.gather(*(w.run() for w in workers))
    finally:
        stop.set()
        if watcher:
            await watcher
    if report.overlaps:
        report.problem(f"{len(report.overlaps)} sampled overlaps, first {report.overlaps[0]}")
    return report


def summary(report):
    finished = sum(p.outcome == "finished" for p in report.products)
    discarded = sum(p.outcome.startswith("discarded") for p in report.products)
    moves = [t for p in report.products for _, t in p.steps]
    return (f"{len(report.products)} products: {finished} finished, {discarded} discarded; "
            f"{len(moves)} moves, mean {sum(moves) / max(1, len(moves)):.1f}s, longest {max(moves, default=0):.1f}s; "
            f"{report.occupy_refusals} occupy refusals (5); {len(report.problems)} problems")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--endpoint", default="opc.tcp://127.0.0.1:4840/")
    p.add_argument("--agents", type=int, default=3)
    p.add_argument("--products", type=int, default=3, help="per agent")
    p.add_argument("--fail", type=float, default=0.2, help="chance a process step fails")
    p.add_argument("--seed", type=int, default=1)
    args = p.parse_args()
    report = asyncio.run(run_line(args.endpoint, args.agents, args.products, fail_p=args.fail, seed=args.seed))
    print(summary(report))
    sys.exit(1 if report.problems else 0)


if __name__ == "__main__":
    main()
