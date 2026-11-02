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
    ({"lanes": [{"points": [[60, 180], [900, 180]]}], "turnSpots": [[240, 120]]}, TABLE, "between flyways"),
    ({"lanes": [{"points": [[60, 180], [900, 180]]}], "turnSpots": [[480, 240]]}, HOLED, "between flyways"),
    ({"lanes": [{"points": [[60, 60], [900, 60]]}], "turnSpots": [[600, 600]]}, TABLE, "no lane slot under it"),
    ({"lanes": [{"points": [[60, 60], [900, 60]]}], "idleArea": [[60, 180], [180, 180], [60, 180]]}, TABLE,
     "passes a slot twice"),
])
def test_a_layout_that_does_not_fit_the_table_says_why(layout, table, message):
    with pytest.raises(LineError, match=message):
        lines(layout, table)


def test_a_layout_inside_a_json_string_is_read_too():
    spec = parse(json.dumps(json.dumps(LOOP)))  # an AAS Property's $value
    assert len(spec.lanes) == 2 and spec.idle_area[0] == (0.06, 0.18)


def aas_form(layout):
    """The layout as the $value of an AAS PlanarRouting collection, its values as text (as BaSyx
    gives them)."""
    point = lambda p: {"X": str(p[0]), "Y": str(p[1])}
    return {"Lanes": [{"Name": lane.get("name", ""), "OneWay": str(lane.get("oneWay", False)).lower(),
                       "Points": [point(p) for p in lane["points"]]} for lane in layout["lanes"]],
            "TurnSpots": [point(p) for p in layout.get("turnSpots", [])],
            "IdleArea": [point(p) for p in layout.get("idleArea", [])]}


def test_a_layout_is_read_from_an_aas_planar_routing_collection():
    spec = parse(json.dumps(aas_form(LOOP)))
    assert spec.lanes == parse(json.dumps(LOOP)).lanes
    assert spec.turn_spots == ((0.84, 0.36),) and spec.idle_area == ((0.06, 0.18), (0.06, 0.54))
    assert lines(aas_form(LOOP)).describe() == lines().describe()
    typed = {"Lanes": [{"Name": "", "OneWay": True, "Points": [{"X": 60, "Y": 60}, {"X": 900.0, "Y": 60}]}]}
    assert parse(json.dumps(typed)).lanes[0].name == "lane 1" and parse(json.dumps(typed)).lanes[0].one_way


@pytest.mark.parametrize("value, message", [
    ({"Lanes": [], "Stations": []}, "PlanarRouting: unknown Stations"),
    ({"Lanes": []}, "at least one lane"),
    ({"Lanes": [{"Points": [{"X": "60", "Y": "60"}, {"X": "a", "Y": "60"}]}]}, "'a' is not a number"),
    ({"Lanes": [{"Points": [{"X": "60", "Y": "60"}, [180, 60]]}]}, "not a point with X and Y"),
    ({"Lanes": [{"Points": [{"X": "60", "Y": "60"}, {"X": "180", "Y": "60"}], "OneWay": "yes"}]}, "oneWay is true or false"),
])
def test_what_is_no_planar_routing_says_why(value, message):
    with pytest.raises(LineError, match=message):
        parse(json.dumps(value))


def test_lanes_are_driven_only_along_them_and_one_way_lanes_only_forward():
    ln = lines()
    on_lanes = lambda cell: {d for d in ln.graph[cell] if d in ln.lane_cells}
    assert on_lanes((2, 1)) == {(3, 1)}                      # on along the loop, not back
    assert on_lanes((3, 1)) == {(4, 1), (3, 2)}              # a crossing: on along both
    assert on_lanes((2, 4)) == {(1, 4)} and (2, 3) in ln.graph[(2, 4)]  # off the lanes: any way
    assert set(ln.graph[(3, 2)]) == {(3, 1), (3, 3), (2, 2), (4, 2)}  # two-way, and off the lanes
    assert (1, 1) in ln.graph[(0, 1)] and (0, 1) in ln.graph[(1, 1)]  # off a lane: both ways
    assert ln.idle == [(0, 1), (0, 2), (0, 3), (0, 4)]
    assert ln.turn_spots == [pytest.approx((0.84, 0.36))]  # flyway (3, 1)'s centre
    assert ln.describe().splitlines() == ["########", "Iv<+<<<#", "Iv#+##^T", "Iv#+##^T", "I>>+>>^#", "########"]


def test_a_turn_spot_may_be_where_four_flyways_meet():
    ln = lines({**LOOP, "turnSpots": [[720, 480], [820, 400]]})
    assert ln.turn_spots == [pytest.approx((0.72, 0.48)), pytest.approx((0.84, 0.36))]  # the second: its flyway's centre
    assert TABLE.spot_cells(ln.turn_spots[0]) == {(5, 3), (6, 3), (5, 4), (6, 4)}
    mm = lambda spots: [(round(x * 1000), round(y * 1000)) for x, y in spots]
    assert mm(TABLE.junctions) == [(240, 240), (240, 480), (480, 240), (480, 480), (720, 240), (720, 480)]
    assert mm(HOLED.junctions) == [(240, 240), (240, 480)]  # the hole at (2, 1) touches the other four


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


def _spy_turns(h):
    """Where each rotation is made: [(xid, (x mm, y mm))]."""
    turns = []
    rotate = h.sim.rotate

    def spy(xid, rz):
        xb = h.sim.xbots[xid]
        turns.append((xid, (round(xb.x * 1000), round(xb.y * 1000))))
        return rotate(xid, rz)
    h.sim.rotate = spy
    return turns


def test_an_xbot_turns_where_four_flyways_meet():
    h = Harness({1: (1, 1)}, layout={**LOOP, "turnSpots": [[720, 480]]})
    turns = _spy_turns(h)
    assert h.go(1, (2, 5), rz_deg=90)
    assert turns == [(1, (720, 480))]
    assert not h.sim.collisions


def test_xbots_wait_for_the_turn_spot_nearest_to_them_rather_than_drive_on_to_another():
    # Two turn spots on the loop: (840, 360) on its right side, met first from the bottom, and
    # (600, 600) on its top. All three turn on the first, one after the other.
    h = Harness({1: (5, 1), 2: (4, 1), 3: (3, 1)}, layout={**LOOP, "turnSpots": [[840, 360], [600, 600]]})
    turns = _spy_turns(h)
    done = []
    with h.lock:
        for xid, cell in ((1, (3, 5)), (2, (2, 5)), (3, (1, 5))):
            h.coord.submit(xid, Pose(*c(*cell), math.radians(90)), lambda ok, e, r: done.append(ok))
    assert h.run(120, lambda: len(done) == 3) and all(done)
    assert sorted(turns) == [(1, (840, 360)), (2, (840, 360)), (3, (840, 360))]
    assert not h.sim.collisions


# Clockwise round the ring one slot in from the edge, a turn spot on every edge flyway's centre,
# a station on each side, beside the ring (the xbot faces the module there: outwards).
CLOCKWISE = json.loads((Path(__file__).parent.parent / "config" / "layouts" / "clockwise-outwards" / "layout.json")
                       .read_text(encoding="utf-8"))


def test_round_the_sides_facing_outwards_every_turn_is_a_quarter_turn_on_a_turn_spot():
    h = Harness({1: (0, 4)}, layout=CLOCKWISE)
    turns = _spy_turns(h)
    sides = [((2, 5), 90), ((7, 2), 0), ((4, 0), -90), ((0, 1), 180)]  # north, east, south, west
    for cell, rz in sides * 2:
        assert h.go(1, cell, rz_deg=rz)
        assert math.degrees(h.sim.xbots[1].rz) == pytest.approx(rz % 360 if rz == 180 else rz, abs=1e-6)
    assert [round(math.degrees(t)) for _, t in h.sim.turns] == [90] + [-90] * 7  # never three quarters round
    spots = {(round(x * 1000), round(y * 1000)) for x, y in h.lines.turn_spots}
    assert {at for _, at in turns} <= spots
    forward = {(a, b) for _, cells in h.lines.lanes for a, b in zip(cells, cells[1:])}
    path = h.visited[1]
    assert all((a, b) in forward for a, b in zip(path, path[1:]) if a in h.lines.lane_cells and b in h.lines.lane_cells)
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


def test_a_reset_reads_the_layout_again_and_says_why_it_does_not_take_one():
    sim = SimDriver(TABLE, {1: (*c(0, 1), 0.0)})
    read = {"layout": LOOP}

    def source():
        if isinstance(read["layout"], Exception):
            raise read["layout"]
        return parse(json.dumps(read["layout"]))
    coord = Coordinator(sim, threading.RLock(), line_spec=source(), line_source=source)
    assert coord.connect() == ErrorID.NONE and coord.line_problem == ""
    read["layout"] = CLOCKWISE  # changed (in the AAS): taken at the next Reset
    assert coord.connect() == ErrorID.NONE
    assert coord.lines.describe() == lines(CLOCKWISE).describe() and coord.line_spec.text == json.dumps(CLOCKWISE)
    read["layout"] = OSError("connection refused")  # the server is not there: the one before is used
    assert coord.connect() == ErrorID.NONE
    assert coord.line_spec.text == json.dumps(CLOCKWISE) and "could not be read again (connection refused)" in coord.line_problem
    read["layout"] = {**CLOCKWISE, "turnSpots": [[600, 360]]}  # no lane slot under it
    assert coord.connect() == ErrorID.LINE_LAYOUT_INVALID
    assert coord.line_problem == "the line layout does not fit the table: turnSpots: (600, 360) mm has no lane slot under it"
    read["layout"] = {"lanes": 3}
    assert coord.connect() == ErrorID.LINE_LAYOUT_INVALID and "is no layout: lanes:" in coord.line_problem
    read["layout"] = LOOP
    assert coord.connect() == ErrorID.NONE and coord.line_problem == ""


def test_the_example_layout_fits_the_4_by_3_table():
    ln = Lines(load(str(Path(__file__).parent.parent / "config" / "line-layout.example.json")), TABLE)
    assert ln.describe() == lines().describe()


def test_a_layout_is_fetched_from_a_url_as_from_an_aas_server():
    body = json.dumps(aas_form(LOOP)).encode()  # the PlanarRouting collection's $value

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
        spec = load(f"http://127.0.0.1:{server.server_port}/submodels/x/submodel-elements/PlanarRouting/$value")
    finally:
        server.shutdown()
        server.server_close()  # else the next request waits out its 10 s timeout, unanswered
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


@pytest.mark.parametrize("n, seed", [(4, 1), pytest.param(8, 1, marks=pytest.mark.slow),
                                     pytest.param(8, 2, marks=pytest.mark.slow)])
def test_traffic_on_the_loop_finishes_without_deadlock(n, seed):
    # These once deadlocked at the turn spot: an xbot pushed into a dead end of the one-way loop,
    # a claimer pushed off its turn spot round the loop, a claim trapping one behind it.
    t, done, collisions, off_lanes = _traffic(n, seed)
    assert done, f"not done in {t:.0f} s"
    assert collisions == 0 and not off_lanes
