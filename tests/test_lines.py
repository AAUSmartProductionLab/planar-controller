import json
import math
import random
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

import pytest

from planar.coordinator import Coordinator, Pose
from planar.layout import Layout
from planar.lines import LineError, Lines, load, parse
from planar.sim_driver import SimDriver
from planar.statemachines import ErrorID

TABLE = Layout(cols=4, rows=3)  # 8 x 6 slots, 960 x 720 mm
HOLED = Layout(cols=4, rows=3, holes=frozenset({(2, 1)}))

# A one-way loop counterclockwise on the ring one slot in from the edge, a two-way shortcut up
# its middle, the idle area along the left edge and one turn spot on the loop's right side:
#
#   ########
#   Iv<+<<<#
#   Iv#+##^T
#   Iv#+##^T
#   I>>+>>^#
#   ########
LOOP = {
    "lanes": [{"name": "loop", "points": [[180, 180], [780, 180], [780, 540], [180, 540], [180, 180]], "oneWay": True},
              {"name": "shortcut", "points": [[420, 180], [420, 540]]}],
    "turnSpots": [[840, 360]],
    "idleArea": [[60, 180], [60, 540]],
}


def c(i, j):
    return ((i + 0.5) * 0.12, (j + 0.5) * 0.12)


def lines(layout=LOOP, table=TABLE):
    return Lines(parse(json.dumps(layout)), table)


class Harness:
    def __init__(self, xbots, layout=LOOP, **kw):
        self.lock = threading.RLock()
        self.t = 0.0
        self.sim = SimDriver(TABLE, {xid: (*c(*cell), 0.0) for xid, cell in xbots.items()})
        self.coord = Coordinator(self.sim, self.lock, clock=lambda: self.t, plan_timeout=0.3,
                                 line_spec=parse(json.dumps(layout)), **kw)
        assert self.coord.connect() == ErrorID.NONE
        self.lines = self.coord.lines
        self.visited = {xid: [cell] for xid, cell in xbots.items()}
        self.covered = set()  # every slot an xbot has been over

    def go(self, xid, cell, rz_deg=0.0, seconds=60):
        done = []
        with self.lock:
            assert self.coord.submit(xid, Pose(*c(*cell), math.radians(rz_deg)),
                                     lambda ok, error, results: done.append(ok)) == ErrorID.NONE
        assert self.run(seconds, lambda: done)
        return done[0]

    def run(self, seconds, until=None):
        for _ in range(int(seconds / 0.02)):
            self.t += 0.02
            self.sim.advance(self.t)
            self.coord.tick(self.t)
            for xid, xb in self.sim.xbots.items():
                self.covered |= TABLE.footprint(xb.x, xb.y, xb.rz) & TABLE.cells
                cell = TABLE.cell_at(xb.x, xb.y, xb.rz, tolerance=0.06)
                if cell is not None and self.visited[xid][-1] != cell:
                    self.visited[xid].append(cell)
            if until is not None and until():
                return True
        return until is None


# --- the layout ------------------------------------------------------------------------------

@pytest.mark.parametrize("layout, message", [
    ("[1, 2]", "a JSON object"),
    ("{lanes: []}", "not JSON"),
    ({"lanes": []}, "at least one lane"),
    ({"lanes": [{"points": [[60, 60], [180, 60]]}], "stations": []}, "unknown stations"),
    ({"lanes": [{"points": [[60, 60]]}]}, "at least 2"),
    ({"lanes": [{"points": [[60, 60], ["180", 60]]}]}, r"not an \[x, y\] point"),
    ({"lanes": [{"points": [[60, 60], [180, 60]], "oneWay": "yes"}]}, "oneWay is true or false"),
])
def test_what_is_no_layout_says_why(layout, message):
    with pytest.raises(LineError, match=message):
        parse(layout if isinstance(layout, str) else json.dumps(layout))


@pytest.mark.parametrize("layout, table, message", [
    ({"lanes": [{"points": [[60, 60], [200, 60]]}]}, TABLE, "not a slot centre"),
    ({"lanes": [{"points": [[60, 60], [180, 180]]}]}, TABLE, "not along a row or a column"),
    ({"lanes": [{"points": [[60, 300], [900, 300]]}]}, HOLED, "crosses a hole"),
    ({"lanes": [{"points": [[60, 60], [900, 60]], "oneWay": True}]}, TABLE, "lead nowhere"),
    ({"lanes": [{"points": [[60, 60], [900, 60]]}, {"points": [[60, 180], [900, 180]]}]}, TABLE, "lead nowhere"),
    ({"lanes": [{"points": [[60, 60], [900, 60]]}], "turnSpots": [[1000, 60]]}, TABLE, "not on a flyway"),
    ({"lanes": [{"points": [[60, 60], [900, 60]]}], "idleArea": [[60, 180], [180, 180], [60, 180]]}, TABLE,
     "passes a slot twice"),
])
def test_a_layout_that_does_not_fit_the_table_says_why(layout, table, message):
    with pytest.raises(LineError, match=message):
        lines(layout, table)


def test_a_layout_inside_a_json_string_is_read_too():
    spec = parse(json.dumps(json.dumps(LOOP)))  # an AAS Property's $value
    assert len(spec.lanes) == 2 and spec.idle_area[0] == (0.06, 0.18)


def test_lanes_are_driven_only_along_them_and_one_way_lanes_only_forward():
    ln = lines()
    on_lanes = lambda cell: {d for d in ln.graph[cell] if d in ln.lane_cells}
    assert on_lanes((2, 1)) == {(3, 1)}                      # on along the loop, not back
    assert on_lanes((3, 1)) == {(4, 1), (3, 2)}              # a crossing: on along both
    assert on_lanes((2, 4)) == {(1, 4)} and (2, 3) in ln.graph[(2, 4)]  # off the lanes: any way
    assert set(ln.graph[(3, 2)]) == {(3, 1), (3, 3), (2, 2), (4, 2)}  # two-way, and off the lanes
    assert (1, 1) in ln.graph[(0, 1)] and (0, 1) in ln.graph[(1, 1)]  # off a lane: both ways
    assert ln.idle == [(0, 1), (0, 2), (0, 3), (0, 4)]
    assert ln.turn_spots == [(3, 1)]
    assert ln.describe().splitlines() == ["########", "Iv<+<<<#", "Iv#+##^T", "Iv#+##^T", "I>>+>>^#", "########"]


def test_a_slot_off_the_lanes_is_reached_the_shortest_way_from_the_nearest_lane_slot():
    ln = lines()
    assert ln.spur((7, 0)) == {(7, 0), (6, 0)}
    assert ln.spur((5, 2)) == {(5, 2)}
    assert ln.spur((4, 1)) == frozenset()
    keep_out = ln.keep_out([(7, 0)])
    assert (7, 0) not in keep_out and (6, 0) not in keep_out and (7, 1) in keep_out
    assert (0, 3) in keep_out and (0, 3) not in ln.keep_out([], idle=True)


# --- the coordinator in line mode ------------------------------------------------------------

def test_a_one_way_lane_is_driven_round_even_to_the_slot_behind():
    h = Harness({1: (3, 1)})
    assert h.go(1, (2, 1))
    assert h.visited[1] == [(3, 1), (3, 2), (3, 3), (3, 4), (2, 4), (1, 4), (1, 3), (1, 2), (1, 1), (2, 1)]
    assert not h.sim.collisions


def test_a_station_off_the_lanes_is_reached_from_the_nearest_lane_slot_and_turned_on_the_turn_spot():
    h = Harness({1: (1, 1)})
    assert h.go(1, (7, 0), rz_deg=90)
    path = h.visited[1]
    assert path[-3:] == [(6, 1), (6, 0), (7, 0)], "off the loop the shortest way"
    assert set(path) <= h.lines.lane_cells | {(6, 0), (7, 0)} | TABLE.flyway_cells((3, 1))
    assert [cmd for cmd in h.sim.commands if cmd[2] == "rotate"], "turned"
    assert h.covered <= h.lines.lane_cells | {(6, 0), (7, 0)} | TABLE.flyway_cells((3, 1))
    assert not h.sim.collisions


def test_free_xbots_queue_in_the_idle_area_and_are_pushed_only_along_the_lanes():
    h = Harness({1: (2, 1), 2: (5, 4), 3: (6, 2)})
    h.run(30)
    assert {h.coord.agents[x].cell for x in (1, 2, 3)} == {(0, 1), (0, 2), (0, 3)}
    assert h.coord.queue_order()[0] in (1, 2, 3)
    allowed = h.lines.lane_cells | h.lines.idle_region | {(2, 1), (5, 4), (6, 2)}
    assert h.covered <= allowed
    assert not h.sim.collisions


def test_several_xbots_keep_to_the_lanes():
    h = Harness({1: (0, 1), 2: (0, 2), 3: (4, 1), 4: (6, 4)})
    stations = [(7, 0), (4, 5), (2, 0), (7, 5)]
    done = []
    with h.lock:
        for xid, cell in zip((1, 2, 3, 4), stations):
            h.coord.submit(xid, Pose(*c(*cell)), lambda ok, e, r: done.append(ok))
    assert h.run(120, lambda: len(done) == 4) and all(done)
    allowed = h.lines.lane_cells | h.lines.idle_region | {(0, 1), (0, 2), (4, 1), (6, 4)}
    for cell in stations:
        allowed |= h.lines.spur(cell)
    assert h.covered <= allowed
    assert not h.sim.collisions


def test_a_layout_that_does_not_fit_the_table_fails_the_connect():
    sim = SimDriver(TABLE, {1: (*c(0, 0), 0.0)})
    spec = parse(json.dumps({"lanes": [{"points": [[60, 60], [1500, 60]]}]}))
    coord = Coordinator(sim, threading.RLock(), line_spec=spec)
    assert coord.connect() == ErrorID.LINE_LAYOUT_INVALID
    assert not coord.connected


def test_the_example_layout_fits_the_4_by_3_table():
    ln = Lines(load(str(Path(__file__).parent.parent / "config" / "line-layout.example.json")), TABLE)
    assert ln.describe() == lines().describe()


def test_a_layout_is_fetched_from_a_url_as_from_an_aas_server():
    body = json.dumps(json.dumps(LOOP)).encode()  # a Property's $value: the JSON as a JSON string

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args):
            pass

    server = HTTPServer(("127.0.0.1", 0), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        spec = load(f"http://127.0.0.1:{server.server_port}/submodels/x/submodel-elements/LineLayout/$value")
    finally:
        server.shutdown()
    assert Lines(spec, TABLE).idle == lines().idle
    with pytest.raises(OSError):
        load(f"http://127.0.0.1:{server.server_port}/gone")


def _traffic(n, seed, moves=6, limit_s=400):
    """n xbots, each sent to `moves` random places on or beside the lanes (some turned), at random
    times; the way the line-mode stress test runs. (end time, all done, collisions, off lanes)."""
    rnd = random.Random(seed)
    ln = lines()
    stations = [s for s in sorted(TABLE.cells) if s not in ln.idle_region and len(ln.spur(s)) <= 2]
    starts = rnd.sample(sorted(TABLE.cells), n)
    h = Harness({k + 1: s for k, s in enumerate(starts)})
    allowed = set(ln.lane_cells) | ln.idle_region | TABLE.flyway_cells((3, 1))
    for s in starts:
        allowed |= ln.spur(s) | {s}
    left = {k + 1: moves for k in range(n)}
    pending = {}

    def send(x):
        cell = rnd.choice([s for s in stations if s not in pending.values()])
        rz = rnd.choice([0, 0, 90])

        def finished(ok, error, results, x=x):
            pending.pop(x, None)
            left[x] -= 1
        pending[x] = cell
        allowed.update(ln.spur(cell) | {cell})
        h.coord.submit(x, Pose(*c(*cell), math.radians(rz)), finished)

    while h.t < limit_s:
        for x in left:
            if x not in pending and left[x] > 0 and rnd.random() < 0.05:
                send(x)
        if not pending and not any(left.values()):
            break
        h.run(0.02)
    return h.t, not pending and not any(left.values()), len(h.sim.collisions), h.covered - allowed


@pytest.mark.parametrize("n, seed", [(4, 1), (8, 1), (8, 2)])
def test_traffic_on_the_loop_finishes_without_deadlock(n, seed):
    # These once deadlocked at the turn spot: an xbot pushed into a dead end of the one-way loop,
    # a claimer pushed off its turn spot round the loop, a claim trapping one behind it.
    t, done, collisions, off_lanes = _traffic(n, seed)
    assert done, f"not done in {t:.0f} s"
    assert collisions == 0 and not off_lanes
