"""Hold and Unhold, the payload, accidents, mastership, zones and the flyways' status, on the
simulated table through the modules, as an orchestrator uses them."""

import pytest

from planar.layout import Layout, overlap
from planar.modules import PlanarController
from planar.sim_driver import SimDriver
from planar.statemachines import ErrorID, ModuleState, SkillState
from planar.zones import ZoneError, ZoneSpec, parse

STRIP = Layout(cols=4, rows=1)  # 8 x 2 slots
SIM = Layout(cols=4, rows=3, holes=frozenset({(2, 1)}))
SERVICE = ZoneSpec("Service", (0.72, 0.48, 0.96, 0.72))  # SIM's flyway (3, 2): slots (6..7, 4..5)


def c(i, j):
    return ((i + 0.5) * 120.0, (j + 0.5) * 120.0)  # mm, as the skill takes them


class Cell:
    def __init__(self, xbots, layout=STRIP, **options):
        self.t = 0.0
        self.sim = SimDriver(layout, {xid: (x / 1000, y / 1000, 0.0) for xid, (x, y) in xbots.items()})
        self.ctl = PlanarController(self.sim, sorted(xbots), clock=lambda: self.t, plan_timeout=0.3, **options)
        self.coord = self.ctl.coordinator
        self.table = self.ctl.table
        self.s = self.ctl.shuttles
        self.watch = None

    def run(self, seconds, until=None):
        for _ in range(int(seconds / 0.02)):
            self.t += 0.02
            self.sim.advance(self.t)
            self.coord.tick(self.t)
            if self.watch:
                self.watch()
            if until is not None and until():
                return True
        return until is None

    def up(self):
        assert self.table.command("Reset", "") == (True, ErrorID.NONE)
        assert self.run(2, lambda: self.table.state == ModuleState.IDLE)
        assert self.table.command("Start", "") == (True, ErrorID.NONE)
        assert all(s.state == ModuleState.EXECUTE for s in self.s.values())

    def skill(self, xid):
        return self.s[xid].skills["MoveToPosition"]

    def move(self, xid, session, x, y, rz=0.0):
        return self.skill(xid).start(session, {"X": x, "Y": y, "Rz": rz})

    def x(self, xid):
        return self.sim.xbots[xid].x


# --- Hold and Unhold -----------------------------------------------------------------------------

def test_a_held_table_pauses_the_moves_for_as_long_as_it_is_held_and_unheld_they_go_on():
    cell = Cell({1: c(0, 0)})
    cell.up()
    cell.s[1].occupation.occupy("A")
    assert cell.move(1, "A", *c(7, 0)) == (True, ErrorID.NONE)
    assert cell.run(0.8)
    assert cell.table.command("Hold", "") == (True, ErrorID.NONE)
    assert (cell.table.state, cell.s[1].state) == (ModuleState.HELD, ModuleState.HELD)
    assert cell.move(1, "A", *c(5, 0)) == (False, ErrorID.NOT_READY)
    cell.run(0.5)
    x = cell.x(1)
    assert c(0, 0)[0] / 1000 < x < c(7, 0)[0] / 1000
    cell.run(30)  # far longer than the move takes: it must not time out
    assert cell.x(1) == x, "paused where it was"
    assert cell.skill(1).state == SkillState.RUNNING
    assert cell.table.command("Unhold", "") == (True, ErrorID.NONE)
    assert (cell.table.state, cell.s[1].state) == (ModuleState.EXECUTE, ModuleState.EXECUTE)
    assert cell.run(10, lambda: cell.skill(1).state == SkillState.SUCCEEDED)
    assert cell.sim.at(1, c(7, 0)[0] / 1000, c(7, 0)[1] / 1000)
    assert not cell.sim.collisions


def test_one_shuttle_held_stops_alone_the_others_go_on_and_it_is_unheld_only_with_the_table_in_execute():
    cell = Cell({1: c(0, 0), 2: c(0, 1)})
    cell.up()
    for xid in (1, 2):
        cell.s[xid].occupation.occupy(f"S{xid}")
    assert cell.move(1, "S1", *c(4, 0)) == (True, ErrorID.NONE)
    assert cell.move(2, "S2", *c(7, 1)) == (True, ErrorID.NONE)
    cell.run(0.5)
    assert cell.s[2].command("Hold", "S2") == (True, ErrorID.NONE)
    x = cell.x(2)
    assert cell.run(10, lambda: cell.skill(1).state == SkillState.SUCCEEDED)
    assert cell.x(2) == x and cell.skill(2).state == SkillState.RUNNING
    assert cell.table.command("Hold", "") == (True, ErrorID.NONE)
    assert cell.s[2].command("Unhold", "S2") == (False, ErrorID.NOT_READY), "the table is held"
    assert cell.table.command("Unhold", "") == (True, ErrorID.NONE)
    assert cell.s[2].state == ModuleState.EXECUTE, "unheld with the table"
    assert cell.run(10, lambda: cell.skill(2).state == SkillState.SUCCEEDED)


def test_a_stop_while_held_lets_the_move_end_and_fails_it_interrupted():
    cell = Cell({1: c(0, 0)})
    cell.up()
    cell.s[1].occupation.occupy("A")
    assert cell.move(1, "A", *c(7, 0)) == (True, ErrorID.NONE)
    cell.run(0.5)
    assert cell.table.command("Hold", "") == (True, ErrorID.NONE)
    cell.run(5)
    assert cell.table.command("Stop", "") == (True, ErrorID.NONE)
    assert cell.run(10, lambda: cell.table.state == ModuleState.STOPPED)
    assert (cell.skill(1).state, cell.skill(1).error) == (SkillState.FAILED, ErrorID.INTERRUPTED)
    assert cell.sim.commands[-1][2] != "pause" and not any(xb.paused for xb in cell.sim.xbots.values())


def test_an_abort_while_held_leaves_no_xbot_paused():
    cell = Cell({1: c(0, 0)})
    cell.up()
    cell.s[1].occupation.occupy("A")
    cell.move(1, "A", *c(7, 0))
    cell.run(0.5)
    cell.table.command("Hold", "")
    assert cell.table.command("Abort", "") == (True, ErrorID.NONE)
    assert not any(xb.paused for xb in cell.sim.xbots.values()), "else the moves sent later would wait too"
    assert all(a.paused_at is None for a in cell.coord.agents.values())


# --- the payload ---------------------------------------------------------------------------------

def test_the_occupant_sets_the_payload_the_pmc_refuses_too_much_and_the_moves_keep_to_its_acceleration():
    cell = Cell({1: c(0, 0)})
    cell.up()
    agent = cell.coord.agents[1]
    assert agent.payload == (0.0, 0.0), "read at the Reset"
    assert cell.ctl.set_payload(1, "A", 0.3, 20.0) == (False, ErrorID.NOT_PERMITTED)
    cell.s[1].occupation.occupy("A")
    assert cell.ctl.set_payload(1, "A", 0.8, 20.0) == (False, ErrorID.OUT_OF_RANGE), "over the xbot's 0.5 kg"
    assert cell.ctl.set_payload(1, "A", -1.0, 0.0) == (False, ErrorID.OUT_OF_RANGE)
    assert cell.ctl.set_payload(1, "A", 0.3, 20.0) == (True, ErrorID.NONE)
    assert (agent.payload, agent.max_accel) == ((0.3, 0.02), 20.0)

    def there_and_back():
        t0 = cell.t
        for to in (c(7, 0), c(0, 0)):
            assert cell.move(1, "A", *to) == (True, ErrorID.NONE)
            assert cell.run(30, lambda: cell.skill(1).state == SkillState.SUCCEEDED)
            assert cell.run(3, lambda: cell.skill(1).state == SkillState.IDLE)
        return cell.t - t0

    quick = there_and_back()
    cell.sim.max_accel = lambda kg: 0.3 if kg > 0.2 else 20.0
    assert cell.ctl.set_payload(1, "A", 0.4, 30.0) == (True, ErrorID.NONE)
    assert agent.max_accel == 0.3
    assert there_and_back() > quick + 2.0, "gentler with the payload"
    assert cell.move(1, "A", *c(7, 0)) == (True, ErrorID.NONE)
    assert cell.ctl.set_payload(1, "A", 0.1, 0.0) == (False, ErrorID.BUSY), "not while it moves"


# --- accidents -----------------------------------------------------------------------------------

def test_an_xbot_the_pmc_switched_off_after_an_accident_is_recovered_by_its_shuttles_reset():
    cell = Cell({1: c(0, 0)})
    cell.up()
    cell.s[1].occupation.occupy("A")
    assert cell.move(1, "A", *c(7, 0)) == (True, ErrorID.NONE)
    cell.run(0.6)
    cell.sim.accident(1)
    assert cell.run(1, lambda: cell.skill(1).state == SkillState.FAILED)
    assert cell.skill(1).error == ErrorID.XBOT_NOT_DETECTED
    assert cell.coord.snapshot(1)[2] == "ACCIDENT (XBOT_DISABLED)"
    assert cell.s[1].command("Stop", "A") == (True, ErrorID.NONE)
    assert cell.run(1, lambda: cell.s[1].state == ModuleState.STOPPED)
    assert cell.s[1].command("Reset", "A") == (True, ErrorID.NONE)
    assert cell.run(2, lambda: cell.s[1].state == ModuleState.IDLE), cell.s[1].error
    assert cell.s[1].command("Start", "A") == (True, ErrorID.NONE)
    assert cell.move(1, "A", *c(7, 0)) == (True, ErrorID.NONE)
    assert cell.run(20, lambda: cell.skill(1).state == SkillState.SUCCEEDED)


def test_the_tables_reset_recovers_the_xbots_after_an_accident():
    cell = Cell({1: c(0, 0)})
    cell.sim.accident(1)
    cell.up()
    assert cell.s[1].state == ModuleState.EXECUTE


# --- mastership ----------------------------------------------------------------------------------

def test_another_program_taking_mastership_aborts_the_table_with_mastership_lost_and_a_reset_takes_it_back():
    cell = Cell({1: c(0, 0)})
    cell.up()
    cell.s[1].occupation.occupy("A")
    assert cell.move(1, "A", *c(7, 0)) == (True, ErrorID.NONE)
    cell.run(0.5)
    cell.sim.master = False
    cell.run(0.1)
    assert (cell.table.state, cell.table.error) == (ModuleState.ABORTED, ErrorID.MASTERSHIP_LOST)
    assert (cell.skill(1).state, cell.skill(1).error) == (SkillState.FAILED, ErrorID.MASTERSHIP_LOST)
    assert cell.table.command("Clear", "") == (True, ErrorID.NONE)
    assert cell.table.command("Reset", "") == (True, ErrorID.NONE)
    cell.run(1)
    assert (cell.table.state, cell.table.error) == (ModuleState.STOPPED, ErrorID.MASTERSHIP_LOST), "still not its"
    cell.sim.master = True
    assert cell.table.command("Reset", "") == (True, ErrorID.NONE)
    assert cell.run(2, lambda: cell.table.state == ModuleState.IDLE)


# --- zones ---------------------------------------------------------------------------------------

def test_a_zone_switched_off_is_kept_clear_its_xbots_swapped_by_hand_and_switched_on_it_is_part_of_the_table():
    cell = Cell({1: c(0, 0), 2: c(7, 5), 3: c(0, 5)}, layout=SIM, zones=[SERVICE])
    cell.up()
    zone = cell.coord.zones["Service"]
    assert (zone.state, zone.xbots, sorted(zone.cells)) == ("ACTIVATED_UNFENCED", (2,), [(6, 4), (6, 5), (7, 4), (7, 5)])
    cell.s[1].occupation.occupy("A")

    # Not while an xbot is on its way in.
    assert cell.move(1, "A", *c(6, 4)) == (True, ErrorID.NONE)
    assert cell.ctl.zone_command("Service", "Deactivate", "") == (False, ErrorID.BUSY)
    cell.skill(1).stop("A")
    assert cell.run(10, lambda: cell.skill(1).state == SkillState.FAILED)
    assert cell.run(5, lambda: not cell.coord.agents[1].actions and cell.coord.agents[1].motion is None)

    cell.table.occupation.occupy("T")
    assert cell.ctl.zone_command("Service", "Deactivate", "") == (False, ErrorID.NOT_PERMITTED), "the table's occupant's"
    assert cell.ctl.zone_command("Service", "Deactivate", "T") == (True, ErrorID.NONE)
    assert cell.run(1, lambda: zone.state == "LOADING_FENCED")
    assert cell.coord.snapshot(2)[1] == "off"
    assert cell.ctl.zone_command("Service", "Deactivate", "T") == (False, ErrorID.NOT_READY)
    cell.run(1)
    assert cell.move(1, "A", *c(6, 4)) == (False, ErrorID.ZONE_CLOSED)

    box = SERVICE.box
    entered = []

    def watch():
        xb = cell.sim.xbots[1]
        if overlap(cell.sim.layout.square(xb.x, xb.y, xb.rz), box):
            entered.append((xb.x, xb.y))
    cell.watch = watch
    assert cell.move(1, "A", *c(7, 3)) == (True, ErrorID.NONE)  # right beside it
    assert cell.run(30, lambda: cell.skill(1).state == SkillState.SUCCEEDED)
    assert not entered

    cell.sim.take(2)
    cell.sim.put(9, 0.78, 0.66)
    assert cell.ctl.zone_command("Service", "Activate", "T") == (True, ErrorID.NONE)
    assert cell.run(3, lambda: zone.state == "ACTIVATED_UNFENCED")
    assert zone.xbots == (9,)
    assert cell.coord.snapshot(9)[1] == "idle", "the one put on is on the table now"
    assert cell.coord.snapshot(2)[1] == "off"
    cell.run(3, lambda: cell.skill(1).state == SkillState.IDLE)
    cell.watch = None
    assert cell.move(1, "A", *c(6, 4)) == (True, ErrorID.NONE)
    assert cell.run(30, lambda: cell.skill(1).state == SkillState.SUCCEEDED)
    assert not cell.sim.collisions


def test_a_zone_off_the_table_fails_the_reset():
    cell = Cell({1: c(0, 0)}, layout=SIM, zones=[ZoneSpec("Out", (0.9, 0.0, 1.2, 0.24))])
    assert cell.table.command("Reset", "") == (True, ErrorID.NONE)
    assert cell.run(1, lambda: cell.table.state == ModuleState.STOPPED)
    assert cell.table.error == ErrorID.ZONES_INVALID
    assert "not on the table" in cell.coord.zone_problem


@pytest.mark.parametrize("text, problem", [
    ('{"zones": [{"name": "A", "from": [0, 0], "to": [0, 240]}]}', "opposite corners"),
    ('{"zones": [{"name": "1A", "from": [0, 0], "to": [240, 240]}]}', "name must be"),
    ('{"zones": [{"name": "A", "from": [0, 0], "to": [240, 240], "speed": 0}]}', "speed"),
    ('{"zones": [{"name": "A", "from": [0, 0], "to": [240, 240], "size": 1}]}', "unknown size"),
    ('{"zones": [{"name": "A", "from": [0, 0], "to": [240, 240]}, {"name": "A", "from": [0, 0], "to": [9, 9]}]}',
     "a second zone named A"),
    ('[]', '"zones"'),
])
def test_what_are_no_zones_says_why(text, problem):
    with pytest.raises(ZoneError, match=problem):
        parse(text)


def test_zones_are_read_in_mm_corners_either_way_round():
    (zone,) = parse('{"zones": [{"name": "Filling", "from": [480, 720], "to": [240, 480], "speed": 0.5}]}')
    assert zone == ZoneSpec("Filling", (0.24, 0.48, 0.48, 0.72), 0.5)


# --- the flyways, and stopping ---------------------------------------------------------------------

def test_the_flyways_power_and_temperatures_are_read_now_and_then_and_the_pmc_disconnected_at_the_end():
    cell = Cell({1: c(0, 0)})
    cell.sim.flyway_status = {(0, 0): (12.0, 41.0, 36.0, 30.5)}
    cell.up()
    cell.run(0.1)
    assert cell.coord.flyway_status == {(0, 0): (12.0, 41.0, 36.0, 30.5)}
    cell.ctl.shutdown()
    assert cell.sim.closed
