import pytest

from planar.layout import Layout
from planar.modules import PlanarController
from planar.sim_driver import SimDriver
from planar.statemachines import ErrorID, ModuleState, SkillState

STRIP = Layout(cols=4, rows=1)  # 8 x 2 slots


def c(i, j):
    return ((i + 0.5) * 120.0, (j + 0.5) * 120.0)  # mm, as the skill takes them


class Cell:
    def __init__(self, xbots):
        self.t = 0.0
        self.sim = SimDriver(STRIP, {xid: (x / 1000, y / 1000, 0.0) for xid, (x, y) in xbots.items()})
        self.ctl = PlanarController(self.sim, sorted(xbots), clock=lambda: self.t, plan_timeout=0.3)
        self.table = self.ctl.table
        self.s = self.ctl.shuttles

    def run(self, seconds, until=None):
        for _ in range(int(seconds / 0.02)):
            self.t += 0.02
            self.sim.advance(self.t)
            self.ctl.coordinator.tick(self.t)
            if until is not None and until():
                return True
        return until is None

    def up(self):
        assert self.table.command("Reset", "") == (True, ErrorID.NONE)
        assert self.run(2, lambda: self.table.state == ModuleState.IDLE)
        assert all(s.state == ModuleState.IDLE for s in self.s.values())
        assert self.table.command("Start", "") == (True, ErrorID.NONE)
        assert all(s.state == ModuleState.EXECUTE for s in self.s.values())

    def move(self, xid, session, x, y, rz=0.0):
        return self.s[xid].skills["MoveToPosition"].start(session, {"X": x, "Y": y, "Rz": rz})


def test_the_gate_checks_occupation_then_module_then_busy_then_range():
    cell = Cell({1: c(0, 0)})
    skill = cell.s[1].skills["MoveToPosition"]
    assert cell.move(1, "A", *c(3, 0)) == (False, ErrorID.NOT_PERMITTED)
    assert cell.s[1].occupation.occupy("A") == (True, ErrorID.NONE)
    assert cell.s[1].occupation.occupy("B") == (False, ErrorID.NOT_PERMITTED)
    assert cell.s[1].occupation.occupy("") == (False, ErrorID.OUT_OF_RANGE)
    assert cell.move(1, "A", *c(3, 0)) == (False, ErrorID.NOT_READY)
    cell.up()
    assert cell.move(1, "B", *c(3, 0)) == (False, ErrorID.NOT_PERMITTED)
    assert cell.move(1, "A", 100.0, 100.0, rz=200.0) == (False, ErrorID.OUT_OF_RANGE)
    assert cell.move(1, "A", 5000.0, 60.0) == (False, ErrorID.NOT_ON_TABLE)  # off the table
    assert cell.move(1, "A", *c(3, 0)) == (True, ErrorID.NONE)
    assert skill.state == SkillState.RUNNING
    assert cell.move(1, "A", *c(4, 0)) == (False, ErrorID.BUSY)
    assert cell.run(10, lambda: skill.state == SkillState.SUCCEEDED)
    assert skill.results["X"] == pytest.approx(c(3, 0)[0])
    assert cell.run(2, lambda: skill.state == SkillState.IDLE)
    assert cell.t > 1.4


def test_stop_interrupts_and_abort_needs_a_reset():
    cell = Cell({1: c(0, 0)})
    cell.up()
    cell.s[1].occupation.occupy("A")
    skill = cell.s[1].skills["MoveToPosition"]
    cell.move(1, "A", *c(7, 0))
    cell.run(0.3)
    assert skill.stop("A") == (True, ErrorID.NONE)
    assert skill.state == SkillState.STOPPING
    assert cell.run(5, lambda: skill.state == SkillState.FAILED)
    assert skill.error == ErrorID.INTERRUPTED
    # Failed may start again without a reset.
    assert cell.move(1, "A", *c(7, 0)) == (True, ErrorID.NONE)
    cell.run(0.3)
    assert skill.abort("A") == (True, ErrorID.NONE)
    assert skill.state == SkillState.ABORTED
    assert cell.move(1, "A", *c(0, 0)) == (False, ErrorID.NOT_READY)
    assert skill.reset("A") == (True, ErrorID.NONE)
    assert cell.move(1, "A", *c(0, 0)) == (True, ErrorID.NONE)
    assert cell.run(10, lambda: skill.state == SkillState.SUCCEEDED)


def test_table_commands_reach_every_shuttle():
    cell = Cell({1: c(0, 0), 2: c(7, 1)})
    cell.up()
    cell.s[1].occupation.occupy("A")
    cell.move(1, "A", *c(7, 0))
    cell.run(0.3)
    assert cell.table.command("Stop", "") == (True, ErrorID.NONE)
    assert cell.run(5, lambda: cell.table.state == ModuleState.STOPPED)
    assert all(s.state == ModuleState.STOPPED for s in cell.s.values())
    assert cell.s[1].skills["MoveToPosition"].error == ErrorID.INTERRUPTED
    cell.up()
    cell.move(1, "A", *c(0, 0))
    cell.run(0.3)
    assert cell.table.command("Abort", "anyone") == (True, ErrorID.NONE)
    assert all(s.state == ModuleState.ABORTED for s in cell.s.values())
    assert cell.s[1].skills["MoveToPosition"].state == SkillState.ABORTED
    assert cell.table.command("Clear", "") == (True, ErrorID.NONE)
    assert all(s.state == ModuleState.STOPPED for s in cell.s.values())
    assert not cell.sim.collisions


def test_only_the_occupant_commands_an_occupied_module_but_anyone_aborts():
    cell = Cell({1: c(0, 0)})
    cell.up()
    shuttle = cell.s[1]
    shuttle.occupation.occupy("A")
    assert shuttle.command("Stop", "B") == (False, ErrorID.NOT_PERMITTED)
    assert shuttle.command("Abort", "B") == (True, ErrorID.NONE)
    assert shuttle.command("Clear", "A") == (True, ErrorID.NONE)
    assert shuttle.command("Reset", "A") == (True, ErrorID.NONE)
    assert shuttle.state == ModuleState.IDLE
    assert shuttle.command("Start", "A") == (True, ErrorID.NONE)
    assert shuttle.state == ModuleState.EXECUTE
    assert shuttle.occupation.release("B") == (False, ErrorID.NOT_PERMITTED)
    assert shuttle.occupation.release("A") == (True, ErrorID.NONE)


def test_an_occupied_idle_xbot_is_not_pushed_a_free_one_is():
    # xbot 2 sits in the way in the lower lane; the upper lane is blocked at i=3 by xbot 3.
    cell = Cell({1: c(0, 0), 2: c(4, 0), 3: c(3, 1)})
    cell.up()
    for xid in (1, 2, 3):
        cell.s[xid].occupation.occupy(f"S{xid}")
    cell.ctl.coordinator.patience_s = 3
    skill = cell.s[1].skills["MoveToPosition"]
    cell.move(1, "S1", *c(7, 0))
    assert cell.run(10, lambda: skill.state == SkillState.FAILED)
    assert skill.error == ErrorID.BLOCKED
    assert cell.sim.at(2, *[v / 1000 for v in c(4, 0)])
    cell.s[2].occupation.release("S2")
    cell.move(1, "S1", *c(7, 0))
    assert cell.run(30, lambda: skill.state == SkillState.SUCCEEDED)
    cell.run(10)
    assert cell.sim.at(2, *[v / 1000 for v in c(4, 0)]), "pushed aside and brought back"
    assert not cell.sim.collisions
