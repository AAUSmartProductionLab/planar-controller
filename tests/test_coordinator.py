import math
import random
import threading

import pytest

from planar.coordinator import Coordinator, Pose
from planar.layout import Layout
from planar.sim_driver import SimDriver
from planar.statemachines import ErrorID

SIM = Layout(cols=4, rows=3, holes=frozenset({(2, 1)}))
LAB = Layout(cols=3, rows=4, holes=frozenset({(2, 3)}))
STRIP = Layout(cols=4, rows=1)  # 8 x 2 slots


def c(i, j):
    return ((i + 0.5) * 0.12, (j + 0.5) * 0.12)


class Harness:
    def __init__(self, layout, xbots, **kw):
        self.lock = threading.RLock()
        self.t = 0.0
        self.sim = SimDriver(layout, xbots)
        self.coord = Coordinator(self.sim, self.lock, clock=lambda: self.t, plan_timeout=0.3, **kw)
        assert self.coord.connect() == ErrorID.NONE
        self.results = {}

    def send(self, xid, x, y, rz_deg=0.0):
        def finished(ok, error, results, xid=xid):
            self.results.setdefault(xid, []).append((ok, ErrorID(error), results))
        with self.lock:
            return self.coord.submit(xid, Pose(x, y, math.radians(rz_deg)), finished)

    def run(self, seconds, until=None):
        for _ in range(int(seconds / 0.02)):
            self.t += 0.02
            self.sim.advance(self.t)
            self.coord.tick(self.t)
            if until is not None and until():
                return True
        return until is None

    def done(self, *xids):
        return lambda: all(x in self.results and len(self.results[x]) >= 1 for x in xids)

    def assert_no_collisions(self):
        assert not self.sim.collisions, self.sim.collisions[:3]


def test_one_xbot_goes_around_the_hole():
    h = Harness(SIM, {1: (*c(3, 2), 0.0)})
    assert h.send(1, *c(6, 2)) == ErrorID.NONE
    assert h.run(30, h.done(1))
    assert h.results[1][0][0] is True
    assert h.sim.at(1, *c(6, 2))
    h.assert_no_collisions()


def test_two_xbots_pass_in_a_one_lane_passage_and_a_parked_one_makes_way():
    # Lane j=1 is walled by held xbots except the pockets at i=0, 4 and 7.
    walls = {10 + i: (*c(i, 1), 0.0) for i in (1, 2, 3, 5, 6)}
    h = Harness(STRIP, {1: (*c(0, 0), 0.0), 2: (*c(7, 0), 0.0), 3: (*c(4, 0), 0.0), **walls})
    for xid in walls:
        h.coord.hold(xid, True)
    assert h.send(1, *c(7, 0)) == ErrorID.NONE
    assert h.send(2, *c(0, 0)) == ErrorID.NONE
    assert h.run(120, h.done(1, 2))
    assert h.results[1][0][0] and h.results[2][0][0]
    h.run(10)
    assert h.sim.at(3, *c(4, 0)), "the parked xbot is back where it was"
    for xid, pose in walls.items():
        assert h.sim.at(xid, pose[0], pose[1]), "held xbots never move"
    h.assert_no_collisions()


def test_a_held_xbot_is_never_pushed_and_the_task_waits_then_fails():
    h = Harness(STRIP, {1: (*c(0, 0), 0.0), 2: (*c(3, 0), 0.0), 3: (*c(3, 1), 0.0)}, patience_s=5)
    h.coord.hold(2, True)
    h.coord.hold(3, True)
    assert h.send(1, *c(7, 0)) == ErrorID.NONE
    assert h.run(20, h.done(1))
    ok, error, _ = h.results[1][0]
    assert not ok and error == ErrorID.BLOCKED
    assert h.sim.at(2, *c(3, 0)) and h.sim.at(3, *c(3, 1))
    h.assert_no_collisions()


def test_rotation_happens_at_a_cleared_flyway_centre():
    # xbot 2 idles on the flyway xbot 1 will want to turn on.
    h = Harness(SIM, {1: (*c(0, 0), 0.0), 2: (*c(1, 1), 0.0)})
    assert h.send(1, *c(0, 1), rz_deg=90) == ErrorID.NONE
    assert h.run(30, h.done(1))
    assert h.results[1][0][0], h.results
    assert h.sim.at(1, *c(0, 1), math.radians(90))
    rotations = [cmd for cmd in h.sim.commands if cmd[2] == "rotate"]
    assert len(rotations) == 1
    h.run(10)
    assert h.sim.at(2, *c(1, 1))
    h.assert_no_collisions()


def test_docking_between_slots_and_at_a_flyway_centre():
    h = Harness(SIM, {1: (0.12, 0.12, 0.0), 2: (*c(7, 5), 0.0)})  # 1 starts docked on a flyway centre
    h.coord.hold(1, True)  # occupied, as an orchestrator's xbots are: they stay where they arrive
    h.coord.hold(2, True)
    assert h.send(1, 0.24, 0.42) == ErrorID.NONE      # straddles four slots
    assert h.send(2, 0.84, 0.36, rz_deg=45) == ErrorID.NONE  # flyway (3, 1) centre, 45 degrees
    assert h.run(40, h.done(1, 2))
    assert h.results[1][0][0] and h.results[2][0][0], h.results
    assert h.sim.at(1, 0.24, 0.42)
    assert h.sim.at(2, 0.84, 0.36, math.radians(45))
    # And away again.
    assert h.send(1, *c(0, 0)) == ErrorID.NONE
    assert h.send(2, *c(6, 0)) == ErrorID.NONE
    assert h.run(40, lambda: len(h.results[1]) == 2 and len(h.results[2]) == 2)
    assert h.results[1][1][0] and h.results[2][1][0], h.results
    h.assert_no_collisions()


def test_an_unoccupied_xbot_off_the_lattice_steps_back_onto_it():
    h = Harness(SIM, {1: (0.12, 0.12, math.radians(45)), 2: (0.24, 0.42, 0.0)})
    h.run(5)
    for xid in (1, 2):
        xb = h.sim.xbots[xid]
        assert SIM.cell_at(xb.x, xb.y, xb.rz) is not None
    h.assert_no_collisions()


def test_targets_it_cannot_reach_are_refused():
    h = Harness(SIM, {1: (*c(0, 0), 0.0)})
    assert h.send(1, 0.60, 0.36) == ErrorID.NOT_ON_TABLE       # in the hole
    assert h.send(1, *c(0, 0), rz_deg=30) == ErrorID.ANGLE_NOT_ALLOWED  # 30 degrees off a flyway centre
    assert h.send(1, 2.0, 0.1) == ErrorID.NOT_ON_TABLE
    assert h.send(1, *c(1, 0)) == ErrorID.NONE
    assert h.send(1, *c(2, 0)) == ErrorID.BUSY


def test_halt_ends_interrupted_on_a_slot_and_abort_stops_at_once():
    h = Harness(SIM, {1: (*c(0, 0), 0.0), 2: (*c(0, 5), 0.0)})
    h.send(1, *c(7, 0))
    h.send(2, *c(7, 5))
    h.run(0.6)
    with h.lock:
        h.coord.halt(1)
        h.coord.abort(2)
        h.coord.hold(2, True)  # its shuttle module is Aborted: it stays put
    assert h.run(10, h.done(1))
    ok, error, _ = h.results[1][0]
    assert not ok and error == ErrorID.INTERRUPTED
    x, y = h.sim.xbots[1].x, h.sim.xbots[1].y
    assert SIM.cell_at(x, y) is not None, "a halted xbot ends on a slot"
    assert 2 not in h.results, "an aborted task says nothing"
    assert SIM.cell_at(h.sim.xbots[2].x, h.sim.xbots[2].y) is None, "an aborted xbot stops where it is"
    # The aborted one gets back onto the lattice with its next task.
    h.send(2, *c(0, 5))
    assert h.run(20, h.done(2))
    assert h.results[2][0][0]
    h.assert_no_collisions()


@pytest.mark.parametrize("layout,n,seed", [(SIM, 4, 1), (SIM, 6, 2), (LAB, 6, 3), (LAB, 10, 4)])
def test_random_traffic(layout, n, seed):
    rng = random.Random(seed)
    cells = sorted(layout.cells)
    starts = rng.sample(cells, n)
    h = Harness(layout, {k + 1: (*c(*s), 0.0) for k, s in enumerate(starts)})
    flyway_centres = [layout.flyway_center(f) for f in layout.flyways]
    sent = {k + 1: 0 for k in range(n)}
    for _ in range(4):
        claimed = set()
        for xid in sent:
            for _attempt in range(20):
                if rng.random() < 0.2:
                    x, y = rng.choice(flyway_centres)
                    rz = rng.choice([0, 90, 180, -90])
                else:
                    x, y = c(*rng.choice(cells))
                    rz = rng.choice([0, 0, 0, 90])
                cells_needed = layout.footprint(x, y)
                if cells_needed & claimed:
                    continue
                claimed |= cells_needed
                assert h.send(xid, x, y, rz) == ErrorID.NONE
                sent[xid] += 1
                break
        assert h.run(240, lambda: all(len(h.results.get(x, [])) == sent[x] for x in sent)), \
            {x: (len(h.results.get(x, [])), sent[x]) for x in sent}
        failures = [(x, r) for x in sent for r in h.results[x] if not r[0]]
        assert not failures, failures
    h.assert_no_collisions()


def test_a_target_a_millimetre_off_a_slot_centre_is_reached_exactly_and_covers_both_slots():
    h = Harness(SIM, {1: (*c(0, 0), 0.0), 2: (*c(5, 0), 0.0)}, patience_s=3)
    h.coord.hold(1, True)
    h.coord.hold(2, True)
    x1 = c(1, 0)[0] + 0.001  # reaches 1 mm into slot (2, 0)
    assert h.send(1, x1, c(1, 0)[1]) == ErrorID.NONE
    assert h.run(20, h.done(1))
    assert h.results[1][0][0]
    xb = h.sim.xbots[1]
    assert abs(xb.x - x1) < 1e-6 and abs(xb.y - c(1, 0)[1]) < 1e-6
    with h.lock:
        assert h.coord.agents[1].footprint == {(1, 0), (2, 0)}
    # Slot (2, 0) is taken by the millimetre xbot 1 reaches into it.
    assert h.send(2, *c(2, 0)) == ErrorID.NONE
    assert h.run(20, h.done(2))
    ok, error, _ = h.results[2][0]
    assert not ok and error == ErrorID.BLOCKED
    h.assert_no_collisions()


def test_xbots_off_the_lattice_may_touch_where_they_share_slots():
    # What was tried by hand: 2 parked on the flyway centre (120, 120), 1 sent to (240, 120),
    # which shares two slots with it but only touches it at x = 180 mm.
    h = Harness(SIM, {1: (0.30, 0.18, 0.0), 2: (0.12, 0.12, 0.0)}, patience_s=10)
    h.coord.hold(1, True)
    h.coord.hold(2, True)
    assert h.send(1, 0.24, 0.12) == ErrorID.NONE
    assert h.run(20, h.done(1))
    assert h.results[1][0][0], h.results
    assert h.sim.at(1, 0.24, 0.12)
    # And the other way round: 1 parked at (240, 240), 2 sent back onto (120, 120) from elsewhere.
    assert h.send(2, *c(0, 4)) == ErrorID.NONE
    assert h.run(20, h.done(2))
    assert h.send(1, 0.24, 0.24) == ErrorID.NONE
    assert h.run(20, lambda: len(h.results[1]) == 2)
    assert h.send(2, 0.12, 0.12) == ErrorID.NONE
    assert h.run(20, lambda: len(h.results[2]) == 2)
    assert h.results[1][1][0] and h.results[2][1][0], h.results
    assert h.sim.at(2, 0.12, 0.12) and h.sim.at(1, 0.24, 0.24)
    h.assert_no_collisions()


def test_leaving_a_shared_slot_steps_onto_one_the_neighbour_does_not_reach_into():
    # 1 at (240, 120) touches 2 at (120, 120); slots (1, 0) and (1, 1) are under both.
    h = Harness(SIM, {1: (0.24, 0.12, 0.0), 2: (0.12, 0.12, 0.0)})
    h.coord.hold(2, True)
    assert h.send(1, *c(0, 4)) == ErrorID.NONE  # nearest way out would be through slot (1, 1)
    assert h.run(30, h.done(1))
    assert h.results[1][0][0], h.results
    assert h.sim.at(1, *c(0, 4))
    first = next(cmd for cmd in h.sim.commands if cmd[1] == 1)
    assert (first[3], first[4]) in (c(2, 0), c(2, 1)), first
    h.assert_no_collisions()


def test_a_turn_on_a_flyway_centre_waits_for_a_neighbour_reaching_into_it():
    h = Harness(SIM, {1: (0.12, 0.12, 0.0), 2: (0.24, 0.12, 0.0)}, patience_s=3)
    h.coord.hold(1, True)
    h.coord.hold(2, True)
    assert h.send(1, 0.12, 0.12, rz_deg=90) == ErrorID.NONE  # its corners would sweep into 2
    assert h.run(10, h.done(1))
    ok, error, _ = h.results[1][0]
    assert not ok and error == ErrorID.BLOCKED
    assert not [cmd for cmd in h.sim.commands if cmd[2] == "rotate"]
    h.coord.hold(2, False)  # 2 is let go: it steps back onto a slot, out of the way
    h.run(3)
    assert h.send(1, 0.12, 0.12, rz_deg=90) == ErrorID.NONE
    assert h.run(10, lambda: len(h.results[1]) == 2)
    assert h.results[1][1][0], h.results
    h.assert_no_collisions()


def test_requests_for_a_taken_place_wait_and_are_served_oldest_first():
    # What was tried by hand: 2 parks on (120, 120); 3 and then 1 are sent there; 2 leaves
    # only after more than a minute. 3 (the older request) gets the place, 1 the next turn.
    h = Harness(SIM, {1: (*c(6, 4), 0.0), 2: (0.12, 0.12, 0.0), 3: (*c(6, 0), 0.0)})
    for xid in (1, 2, 3):
        h.coord.hold(xid, True)
    assert h.send(3, 0.12, 0.12) == ErrorID.NONE
    h.run(1)
    assert h.send(1, 0.12, 0.12) == ErrorID.NONE
    h.run(90)
    assert 3 not in h.results and 1 not in h.results, "both wait, however long it takes"
    assert h.send(2, *c(7, 5)) == ErrorID.NONE
    assert h.run(30, h.done(2, 3))
    assert h.results[3][0][0] and 1 not in h.results
    assert h.sim.at(3, 0.12, 0.12)
    assert h.send(3, *c(7, 0)) == ErrorID.NONE
    assert h.run(30, h.done(1))
    assert h.results[1][0][0] and h.sim.at(1, 0.12, 0.12)
    h.assert_no_collisions()


def test_requests_made_at_once_for_the_same_place_do_not_wait_for_each_other():
    h = Harness(SIM, {1: (*c(0, 4), 0.0), 2: (*c(3, 0), 0.0)})
    h.coord.hold(1, True)
    h.coord.hold(2, True)
    assert h.send(1, *c(1, 1)) == ErrorID.NONE
    assert h.send(2, *c(1, 1)) == ErrorID.NONE  # the same tick
    assert h.run(20, h.done(1))
    assert h.results[1][0][0] and 2 not in h.results
    h.assert_no_collisions()


def test_a_short_move_with_nothing_in_the_way_is_one_straight_move():
    # By hand: from (850, 120) to (850, 240) it went over a slot centre, corner to corner.
    h = Harness(SIM, {1: (0.85, 0.12, 0.0), 2: (*c(0, 5), 0.0)})
    h.coord.hold(1, True)
    assert h.send(1, 0.85, 0.24) == ErrorID.NONE
    assert h.run(10, h.done(1))
    assert h.results[1][0][0]
    moves = [cmd for cmd in h.sim.commands if cmd[1] == 1]
    assert len(moves) == 1 and moves[0][2:] == ("move", 0.85, 0.24), moves


def test_a_longer_route_has_one_corner_not_a_staircase():
    # 2 stands in the box of the straight line, so 1 is planned: right, then up.
    h = Harness(SIM, {1: (*c(0, 0), 0.0), 2: (*c(1, 2), 0.0)})
    h.coord.hold(2, True)
    assert h.send(1, *c(3, 5)) == ErrorID.NONE
    assert h.run(20, h.done(1))
    assert h.results[1][0][0]
    moves = [cmd for cmd in h.sim.commands if cmd[1] == 1]
    assert len(moves) <= 3, moves
    h.assert_no_collisions()


def test_into_and_out_of_a_gap_between_two_xbots_it_touches_on_both_sides():
    # 1 at (120, 120) and 3 at (360, 120) leave (240, 120) between them: every slot it covers is
    # partly under one of them, so the way in is from above, straight down; and out the same way.
    h = Harness(SIM, {1: (0.12, 0.12, 0.0), 2: (*c(1, 4), 0.0), 3: (0.36, 0.12, 0.0)})
    for xid in (1, 2, 3):
        h.coord.hold(xid, True)
    assert h.send(2, 0.24, 0.12) == ErrorID.NONE
    assert h.run(20, h.done(2))
    assert h.results[2][0][0], h.results
    assert h.sim.at(2, 0.24, 0.12)
    assert h.send(2, *c(6, 5)) == ErrorID.NONE
    assert h.run(20, lambda: len(h.results[2]) == 2)
    assert h.results[2][1][0], h.results
    assert h.sim.at(1, 0.12, 0.12) and h.sim.at(3, 0.36, 0.12)
    h.assert_no_collisions()


def test_three_xbots_touching_in_a_row_change_places_round():
    row = {1: (0.12, 0.12), 2: (0.24, 0.12), 3: (0.36, 0.12)}
    h = Harness(SIM, {x: (*p, 0.0) for x, p in row.items()})
    for xid in row:
        h.coord.hold(xid, True)
    nxt = {1: row[2], 2: row[3], 3: row[1]}
    for xid, p in nxt.items():
        assert h.send(xid, *p) == ErrorID.NONE
    assert h.run(60, h.done(1, 2, 3))
    assert all(h.results[x][0][0] for x in row), h.results
    assert all(h.sim.at(x, *nxt[x]) for x in row)
    h.assert_no_collisions()


def test_many_requests_at_once_for_one_place_are_served_in_the_order_they_were_made():
    starts = {1: c(0, 0), 2: c(7, 0), 3: c(0, 5), 4: c(7, 5), 5: c(3, 0)}
    parks = {1: c(1, 0), 2: c(6, 0), 3: c(1, 5), 4: c(6, 5), 5: c(4, 0)}
    h = Harness(SIM, {x: (*p, 0.0) for x, p in starts.items()})
    for xid in starts:
        h.coord.hold(xid, True)
    spot = (0.36, 0.60)  # a flyway centre
    for xid in (4, 2, 5, 1, 3):  # the order of the requests, all in the same tick
        assert h.send(xid, *spot) == ErrorID.NONE
    arrived = []
    while len(arrived) < 5 and h.t < 300:
        h.run(0.02)
        for xid in starts:
            if xid not in arrived and len(h.results.get(xid, [])) == 1:
                assert h.results[xid][0][0], h.results
                arrived.append(xid)
                h.run(1.0)  # the station works on it
                assert h.send(xid, *parks[xid]) == ErrorID.NONE
    assert arrived == [4, 2, 5, 1, 3]
    assert h.run(30, lambda: all(len(h.results[x]) == 2 for x in starts))
    h.assert_no_collisions()


def test_stopping_the_oldest_waiting_request_lets_the_next_one_in():
    h = Harness(SIM, {1: (0.12, 0.12, 0.0), 2: (*c(6, 4), 0.0), 3: (*c(6, 0), 0.0)})
    for xid in (1, 2, 3):
        h.coord.hold(xid, True)
    assert h.send(2, 0.12, 0.12) == ErrorID.NONE  # 1 is there: 2 waits, 3 waits behind 2
    assert h.send(3, 0.12, 0.12) == ErrorID.NONE
    h.run(5)
    with h.lock:
        h.coord.halt(2)
    assert h.run(5, h.done(2))
    ok, error, _ = h.results[2][0]
    assert not ok and error == ErrorID.INTERRUPTED
    assert h.send(1, *c(7, 5)) == ErrorID.NONE
    assert h.run(30, h.done(1, 3))
    assert h.results[3][0][0] and h.sim.at(3, 0.12, 0.12)
    h.assert_no_collisions()


def test_a_new_request_is_planned_while_others_are_on_their_way():
    # 1 goes the long way along row 0 (the straight box is not free: 4 is in it); while it is on
    # its way, 2 is sent around the held 3. 2 must not wait for 1 to arrive before it sets off.
    h = Harness(SIM, {1: (*c(0, 0), 0.0), 2: (*c(0, 5), 0.0), 3: (*c(1, 5), 0.0), 4: (*c(5, 1), 0.0)})
    for xid in (3, 4):
        h.coord.hold(xid, True)
    assert h.send(1, *c(7, 1)) == ErrorID.NONE
    assert h.run(5, lambda: h.sim.xbots[1].motion is not None)
    h.run(0.1)
    assert h.send(2, *c(3, 5)) == ErrorID.NONE
    assert h.run(5, lambda: any(cmd[1] == 2 for cmd in h.sim.commands))
    assert h.sim.xbots[1].motion is not None, "2 set off while 1 was still on its way"
    assert h.run(30, h.done(1, 2))
    assert h.results[1][0][0] and h.results[2][0][0], h.results
    h.assert_no_collisions()


def test_two_turns_may_wait_for_their_flyways_to_clear_at_once():
    # 3 idles on the flyway 1 turns on, 4 on the one 2 turns on: both claims wait at once.
    h = Harness(SIM, {1: (*c(0, 0), 0.0), 2: (*c(6, 4), 0.0), 3: (*c(1, 1), 0.0), 4: (*c(7, 5), 0.0)})
    with h.lock:
        a1, a2 = h.coord.agents[1], h.coord.agents[2]
        h.coord.submit(1, Pose(*c(0, 0), math.radians(90)), lambda *a: None)
        h.coord.submit(2, Pose(*c(6, 4), math.radians(90)), lambda *a: None)
        assert h.coord._claim(a1, (SIM.flyway_box((0, 0)),))
        assert h.coord._claim(a2, (SIM.flyway_box((3, 2)),)), "a second claim waits alongside the first"
    h.coord.agents[1].task = h.coord.agents[2].task = None
    h.coord.agents[1].zone = h.coord.agents[2].zone = None
    h.coord.agents[1].zone_boxes = h.coord.agents[2].zone_boxes = ()
    assert h.send(1, *c(0, 0), rz_deg=90) == ErrorID.NONE
    assert h.send(2, *c(6, 4), rz_deg=90) == ErrorID.NONE
    assert h.run(30, h.done(1, 2))
    assert h.results[1][0][0] and h.results[2][0][0], h.results
    assert h.sim.at(1, *c(0, 0), math.radians(90)) and h.sim.at(2, *c(6, 4), math.radians(90))
    h.assert_no_collisions()


def test_a_claim_that_would_leave_no_way_out_is_not_made():
    # On the strip: 3 idles in flyway (1, 0); the held 5 and 6 wall it off on the left. With 2's
    # claim on flyway (2, 0) it could not get out of 1's claim on (1, 0) to anywhere.
    h = Harness(STRIP, {1: (*c(3, 1), 0.0), 2: (*c(5, 1), 0.0), 3: (*c(2, 0), 0.0),
                        5: (*c(1, 0), 0.0), 6: (*c(1, 1), 0.0)})
    for xid in (5, 6):
        h.coord.hold(xid, True)
    h.run(0.1)
    with h.lock:
        a1, a2 = h.coord.agents[1], h.coord.agents[2]
        for xid in (1, 2):
            h.coord.submit(xid, Pose(*c(xid * 2 + 1, 1), math.radians(90)), lambda *a: None)
        assert h.coord._claim(a1, (STRIP.flyway_box((1, 0)),))
        assert not h.coord._claim(a2, (STRIP.flyway_box((2, 0)),))
        assert h.coord._claim(a2, (STRIP.flyway_box((3, 0)),)), "one that leaves room is made"


def test_idle_xbots_queue_up_at_the_loading_station():
    # The station is the slot (0, 1) on the left edge; the queue goes along the edges.
    h = Harness(SIM, {1: (*c(7, 5), 0.0), 2: (*c(4, 0), 0.0), 3: (*c(6, 2), 0.0)},
                loading_station=c(0, 1))
    assert h.coord.queue[:4] == [(0, 1), (0, 0), (0, 2), (1, 0)]
    assert h.run(30, lambda: {h.coord.agents[x].cell for x in (1, 2, 3)} == {(0, 1), (0, 0), (0, 2)}
                 and all(h.coord.agents[x].motion is None for x in (1, 2, 3)))
    with h.lock:
        order = h.coord.queue_order()
    assert len(order) == 3 and h.coord.agents[order[0]].cell == (0, 1)
    # The first one is taken (occupied) for a product and sent off; the queue closes up, with
    # the fewest moves: the one at (0, 2) onto the station, the one at (0, 0) stays.
    first, second, third = order
    h.coord.hold(first, True)
    sent = h.t
    assert h.send(first, *c(7, 5)) == ErrorID.NONE
    assert h.run(30, h.done(first))
    assert h.run(30, lambda: h.coord.agents[third].cell == (0, 1) and h.coord.agents[third].motion is None)
    assert h.coord.agents[second].cell == (0, 0)
    assert not [cmd for cmd in h.sim.commands if cmd[1] == second and cmd[0] > sent]
    with h.lock:
        assert h.coord.queue_order() == [third, second]
    h.assert_no_collisions()


def test_idle_xbots_are_turned_to_the_idle_orientation_in_the_queue():
    q = math.radians(90)
    h = Harness(SIM, {1: (*c(7, 5), 0.0), 2: (*c(4, 0), q), 3: (*c(6, 2), -q)},
                loading_station=c(0, 1), idle_rz=0.0)
    assert h.run(60, lambda: all(h.coord.agents[x].task is None and h.coord.agents[x].motion is None
                                 and abs(h.sim.xbots[x].rz) < 0.01 for x in (1, 2, 3))
                 and {h.coord.agents[x].cell for x in (1, 2, 3)} == {(0, 1), (0, 0), (0, 2)})
    assert [cmd[1] for cmd in h.sim.commands if cmd[2] == "rotate"] and 1 not in \
        [cmd[1] for cmd in h.sim.commands if cmd[2] == "rotate"], "only the turned ones turn"
    with h.lock:
        assert len(h.coord.queue_order()) == 3
    h.assert_no_collisions()


def test_a_request_replaces_a_turn_to_the_idle_orientation():
    h = Harness(SIM, {1: (*c(0, 0), math.radians(90))}, idle_rz=0.0)
    assert h.run(1, lambda: h.coord.agents[1].task is not None)
    assert h.coord.agents[1].task.internal
    h.coord.hold(1, True)
    assert h.send(1, *c(7, 5), rz_deg=90) == ErrorID.NONE, "not Busy: the turn gives way"
    assert h.run(30, h.done(1))
    assert h.results[1][0][0] and h.sim.at(1, *c(7, 5), math.radians(90))
    h.run(5)
    assert h.sim.at(1, *c(7, 5), math.radians(90)), "held: it stays as it was sent"
    h.assert_no_collisions()


def test_the_idle_orientation_is_a_multiple_of_90_degrees():
    h = Harness(SIM, {1: (*c(0, 0), 0.0)})
    assert h.coord.set_idle_rz(math.radians(45)) == ErrorID.OUT_OF_RANGE and h.coord.idle_rz is None
    assert h.coord.set_idle_rz(math.radians(270)) == ErrorID.NONE
    assert h.coord.idle_rz == pytest.approx(math.radians(-90))
    assert h.coord.set_idle_rz(math.nan) == ErrorID.NONE and h.coord.idle_rz is None
