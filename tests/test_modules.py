"""The table and shuttle modules on the simulated driver, as an orchestrator uses them."""

import math

import pytest

from harness import STRIP, Cell, c
from planar.layout import Layout
from planar.statemachines import ErrorID, ModuleState, SkillState


def test_the_gate_checks_occupation_then_module_then_busy_then_range():
    """Verify skill gates check ownership, module readiness, activity and ranges in order."""
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
    """Verify Stop reports interruption and Abort requires Reset."""
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
    """Verify table lifecycle commands propagate to shuttle modules."""
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
    """Verify occupation gates commands while module Abort remains available to anyone."""
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
    """Verify occupied idle xbots stay pinned while free ones may move aside."""
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


def test_reset_routes_the_shuttles_into_the_loading_queue():
    """Verify Reset places shuttles in the loading queue."""
    cell = Cell({1: c(7, 1), 2: c(4, 0), 3: c(6, 1)}, loading_station=(0.06, 0.06))
    assert cell.table.command("Reset", "") == (True, ErrorID.NONE)
    assert cell.run(1, lambda: cell.table.state == ModuleState.RESETTING)
    assert [cmd for cmd in cell.sim.commands if cmd[2] == "auto"], "one auto routing command"
    assert cell.run(30, lambda: cell.table.state == ModuleState.IDLE)
    slots = {cell.ctl.coordinator.agents[x].cell for x in (1, 2, 3)}
    assert slots == set(cell.ctl.coordinator.queue[:3]) and (0, 0) in slots
    assert [cmd for cmd in cell.sim.commands if cmd[2] != "auto"] == [], "the planner had nothing to do"
    assert not cell.sim.collisions


def test_a_stop_during_resetting_stops_the_auto_routing():
    """Verify Stop during Reset cancels auto routing."""
    cell = Cell({1: c(7, 1), 2: c(4, 0)}, loading_station=(0.06, 0.06))
    assert cell.table.command("Reset", "") == (True, ErrorID.NONE)
    cell.run(1.0)
    assert cell.table.command("Stop", "") == (True, ErrorID.NONE)
    assert cell.run(2, lambda: cell.table.state == ModuleState.STOPPED)
    assert cell.sim.auto is None and all(xb.motion is None for xb in cell.sim.xbots.values())
    x = cell.sim.xbots[1].x
    cell.run(1.0)
    assert cell.sim.xbots[1].x == x, "nothing moves after the Stop"


def test_more_shuttles_than_one_auto_routing_command_takes_are_lined_up_by_the_planner():
    """Verify oversized loading groups fall back to planned placement."""
    table = Layout(cols=4, rows=3)  # 8 x 6 slots
    xbots = {k + 1: c(7 - k % 4, 5 - k // 4) for k in range(12)}  # the far corner, 4 x 3
    cell = Cell(xbots, layout=table, loading_station=(0.06, 0.06), idle_rz=math.radians(90))
    assert cell.table.command("Reset", "") == (True, ErrorID.NONE)
    assert cell.run(60, lambda: cell.table.state == ModuleState.IDLE)
    assert not [cmd for cmd in cell.sim.commands if cmd[2] == "auto"], "no auto routing for 12"
    assert {a.cell for a in cell.ctl.coordinator.agents.values()} == set(cell.ctl.coordinator.queue[:12])
    assert all(abs(xb.rz - math.radians(90)) < 0.01 for xb in cell.sim.xbots.values())
    assert not cell.sim.collisions


def test_a_boxed_in_shuttle_keeps_its_slot_while_the_others_line_up_and_turn():
    """Verify a boxed-in shuttle stays put while other shuttles gather and turn."""
    # Turns claim flyways near the queue; one xbot is left without a free slot it can reach, and
    # the slot it stays on is not given to another as well.
    slots = [(6, 3), (2, 4), (3, 4), (7, 2), (6, 5), (5, 3), (0, 1), (4, 5), (2, 3), (0, 3)]
    angles = [180, 0, -90, 90, 180, 90, 0, 180, 0, 180]
    cell = Cell({k + 1: c(*s) for k, s in enumerate(slots)}, layout=Layout(cols=4, rows=3),
                loading_station=(0.06, 0.06), idle_rz=math.radians(90))
    for k, deg in enumerate(angles):
        cell.sim.xbots[k + 1].rz = math.radians(deg)
    assert cell.table.command("Reset", "") == (True, ErrorID.NONE)
    assert cell.run(60, lambda: cell.table.state == ModuleState.IDLE)
    assert all(abs(xb.rz - math.radians(90)) < 0.01 for xb in cell.sim.xbots.values())
    assert not cell.sim.collisions


def test_reset_turns_every_shuttle_to_the_idle_orientation():
    """Verify Reset aligns shuttles with the idle orientation."""
    cell = Cell({1: c(7, 1), 2: c(4, 0), 3: c(6, 1)}, loading_station=(0.06, 0.06), idle_rz=math.radians(90))
    cell.sim.xbots[2].rz = math.radians(-90)
    cell.sim.xbots[3].rz = math.radians(180)
    assert cell.table.command("Reset", "") == (True, ErrorID.NONE)
    assert cell.run(60, lambda: cell.table.state == ModuleState.IDLE)
    for xid in (1, 2, 3):
        xb = cell.sim.xbots[xid]
        assert abs(xb.rz - math.radians(90)) < 0.01, (xid, xb.rz)
        assert STRIP.cell_at(xb.x, xb.y, xb.rz) is not None, "back on a slot"
    assert not cell.sim.collisions
