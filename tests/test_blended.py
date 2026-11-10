"""Experimental execution: swept space, dependencies, replanning and lifecycle."""

import math
import threading
from collections import deque

import pytest

from planar.coordinator import Action, Coordinator as Classic, Pose
from planar.coordinator_blended import Coordinator
from planar.layout import Layout
from planar.modules import PlanarController
from planar.rounded_path import RoundedPath
from planar.sim_driver import SimDriver
from planar.statemachines import ErrorID


def setup_corner(others=None, layout=None):
    """Build a simulated coordinator with a ready two-leg route and a completion collector."""
    layout = layout or Layout(cols=3, rows=3)
    driver = SimDriver(layout, {1: (*layout.center((0, 0)), 0.0), **(others or {})})
    coordinator = Coordinator(driver, threading.RLock(), clock=lambda: driver.now, speed=0.1)
    assert coordinator.connect() == ErrorID.NONE
    result = []
    assert coordinator.submit(1, Pose(*layout.center((3, 3))), lambda *args: result.append(args)) == ErrorID.NONE
    cells = [(0, 0), (1, 0), (2, 0), (3, 0), (3, 1), (3, 2), (3, 3)]
    agent = coordinator.agents[1]
    agent.actions = deque(Action(i+1, 1, a, b, frozenset()) for i, (a, b) in enumerate(zip(cells, cells[1:])))
    agent.want = (3, 3)
    coordinator._next_action = 6
    return driver, coordinator, agent, result


def advance(driver, coordinator, duration):
    """Advance the simulated driver and coordinator together in 20 ms steps."""
    for _ in range(round(duration/0.02)):
        driver.advance(driver.now+0.02)
        coordinator.tick(driver.now)


@pytest.mark.parametrize('u,v', [((1, 0), (0, 1)), ((-1, 0), (0, 1)),
                                 ((0, -1), (-1, 0)), ((0, 1), (1, 0))])
def test_rounded_geometry_is_continuous_and_stays_in_reserved_boxes(u, v):
    """Verify rounded paths stay continuous and inside their reservations."""
    layout = Layout(cols=4, rows=4)
    start = (0.42, 0.42)
    corner = tuple(start[i]+0.12*u[i] for i in range(2))
    end = tuple(corner[i]+0.24*v[i] for i in range(2))
    path = RoundedPath(start, corner, end, 0.03)
    assert path.point(0) == start
    assert path.point(path.length) == pytest.approx(end)
    assert path.point(0.09) == pytest.approx(path.tangents[0])
    assert path.point(0.09+math.pi*0.03/2) == pytest.approx(path.tangents[1])
    reserved = set().union(*(layout.cells_in(b) for b in path.boxes(layout)))
    for k in range(101):
        assert layout.footprint(*path.point(path.length*k/100)) <= reserved


def test_default_and_explicit_planner_selection():
    """Verify classic is the default and blended must be selected explicitly."""
    driver = SimDriver(Layout(cols=2, rows=2), {1: (0.06, 0.06)})
    assert type(PlanarController(driver, [1]).coordinator) is Classic
    assert type(PlanarController(driver, [1], planner='blended').coordinator) is Coordinator
    with pytest.raises(ValueError, match='unknown planner'):
        PlanarController(driver, [1], planner='wrong')


def test_corner_is_one_continuous_move_and_claim_survives_replanning():
    """Verify a corner remains one reserved motion through replanning."""
    driver, coordinator, agent, result = setup_corner()
    coordinator._dispatch(0)
    assert agent.motion.kind == 'blend'
    claim = agent.claim
    assert (2, 1) in claim  # inside corner, not one of the original route's cells
    assert not coordinator._done  # no early grid progress across a curved path
    advance(driver, coordinator, 0.8)
    assert agent.claim == claim
    assert agent.motion.kind == 'blend'
    assert driver.read()[1].state == 'moving'
    advance(driver, coordinator, 10)
    assert result and result[0][0]
    assert agent.claim is None and driver.at(1, 0.42, 0.42)
    assert [c[2] for c in driver.commands] == ['blend']
    assert not driver.collisions


@pytest.mark.parametrize('reason', ['dependency', 'inside_corner', 'hole', 'line', 'unsupported'])
def test_unsafe_or_unsupported_corner_falls_back(reason):
    """Verify unsafe or unsupported corners use classic straight-run execution."""
    others = {2: (0.30, 0.18, 0.0)} if reason == 'inside_corner' else None
    driver, coordinator, agent, _ = setup_corner(others)
    if reason == 'dependency':
        agent.actions[3].deps = frozenset({999})
    elif reason == 'hole':
        # The route remains valid but rounding it cuts into a missing cell.
        coordinator.layout = Layout(cols=6, rows=6, flyway_size=0.12, holes=frozenset({(2, 1)}))
    elif reason == 'line':
        coordinator.lines = object()
    elif reason == 'unsupported':
        driver.supports_blended_moves = False
    coordinator._dispatch(0)
    assert driver.commands[0][2] == 'move'
    assert agent.motion.kind == 'nav'


def test_hold_preserves_claim_and_abort_clears_compound_motion():
    """Verify Hold retains a claim and Abort clears the compound motion."""
    driver, coordinator, agent, _ = setup_corner()
    coordinator._dispatch(0)
    advance(driver, coordinator, 0.5)
    assert coordinator.pause(1) == ErrorID.NONE
    pose = (driver.xbots[1].x, driver.xbots[1].y)
    claim = agent.claim
    advance(driver, coordinator, 3)
    assert (driver.xbots[1].x, driver.xbots[1].y) == pose
    assert agent.claim == claim
    assert coordinator.resume(1) == ErrorID.NONE
    advance(driver, coordinator, 0.5)
    assert (driver.xbots[1].x, driver.xbots[1].y) != pose
    coordinator.abort(1)
    assert driver.read()[1].state == 'idle'
    assert agent.motion is None and agent.claim is None and not agent.ops


def test_graceful_stop_finishes_only_the_committed_corner():
    """Verify graceful Stop finishes only the committed corner."""
    driver, coordinator, agent, result = setup_corner()
    coordinator._dispatch(0)
    coordinator.halt(1)
    advance(driver, coordinator, 10)
    assert result[0][0] is False and result[0][1] == ErrorID.INTERRUPTED
    assert driver.at(1, 0.42, 0.42)
    assert len(driver.commands) == 1 and not agent.claim


def test_refused_compound_command_fails_and_releases_claim():
    """Verify a refused blended command fails the task and releases its claim."""
    driver, coordinator, agent, result = setup_corner()
    from planar.driver import DriverError
    def refuse(*args):
        """Simulate firmware rejecting a blended move."""
        raise DriverError('unsupported PMC firmware')
    driver.move_blended = refuse
    coordinator._dispatch(0)
    assert result[0][0] is False and result[0][1] == ErrorID.PMC_ERROR
    assert agent.motion is None and agent.claim is None


def test_another_xbot_cannot_enter_reserved_inside_corner():
    """Verify another xbot cannot enter a reserved corner interior."""
    driver, coordinator, agent, result = setup_corner({2: (0.06, 0.42, 0.0)})
    coordinator._dispatch(0)
    assert agent.motion.kind == 'blend'
    other_result = []
    coordinator.submit(2, Pose(0.30, 0.18), lambda *args: other_result.append(args))
    advance(driver, coordinator, 20)
    assert result[0][0] and other_result[0][0]
    assert not driver.collisions
