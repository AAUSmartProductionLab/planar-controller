import random

import pytest

from planar.layout import Layout
from planar.mapf import Instance, plan, valid_step

LAB = Layout(cols=3, rows=4, holes=frozenset({(2, 3)}))
SIM = Layout(cols=4, rows=3, holes=frozenset({(2, 1)}))


def check(instance, result):
    assert result.configs[0] == instance.starts
    for a, b in zip(result.configs, result.configs[1:]):
        assert valid_step(instance, a, b)
    if result.complete:
        assert result.configs[-1] == instance.goals


def corridor(length, pocket_at=None):
    cells = {(i, 0) for i in range(length)}
    if pocket_at is not None:
        cells.add((pocket_at, 1))
    adjacency = {c: [d for d in [(c[0] + 1, c[1]), (c[0] - 1, c[1]), (c[0], c[1] + 1), (c[0], c[1] - 1)] if d in cells]
                 for c in cells}
    return cells, adjacency


def test_two_xbots_pass_each_other_in_a_one_lane_passage_with_a_pocket():
    cells, adjacency = corridor(7, pocket_at=3)
    inst = Instance(cells, adjacency, starts=[(0, 0), (6, 0)], goals=[(6, 0), (0, 0)])
    result = plan(inst, timeout=2)
    check(inst, result)
    assert result.complete


def test_a_parked_xbot_in_the_passage_steps_into_the_pocket_and_back():
    cells, adjacency = corridor(9, pocket_at=4)
    # xbot 1 is parked in the middle of the passage and must end where it started.
    inst = Instance(cells, adjacency, starts=[(0, 0), (6, 0), (8, 0)], goals=[(8, 0), (6, 0), (0, 0)])
    result = plan(inst, timeout=2)
    check(inst, result)
    assert result.complete


def test_no_way_past_is_reported_not_hung():
    cells, adjacency = corridor(6)
    inst = Instance(cells, adjacency, starts=[(0, 0), (5, 0)], goals=[(5, 0), (0, 0)])
    result = plan(inst, timeout=2)
    check(inst, result)
    assert not result.complete


def test_keep_out_is_respected_and_xbots_leave_it():
    cells, adjacency = LAB.cells, LAB.adjacency
    zone = LAB.flyway_cells((1, 1))
    start_in_zone = sorted(zone)[0]
    keep_out = [frozenset(), zone]
    inst = Instance(cells, adjacency, starts=[(0, 0), start_in_zone], goals=[(0, 7), (0, 4)], keep_out=keep_out)
    result = plan(inst, timeout=2)
    check(inst, result)
    assert result.complete
    entered = [c[1] for c in result.configs[1:] if c[1] in zone]
    # It may only be in the zone while leaving it, at the start.
    assert all(c[1] not in zone for c in result.configs[len(entered) + 1:])


@pytest.mark.parametrize("layout,n", [(LAB, 4), (LAB, 8), (LAB, 20), (SIM, 8), (SIM, 24)])
def test_random_instances(layout, n):
    rng = random.Random(n)
    for _ in range(10):
        cells = sorted(layout.cells)
        starts = rng.sample(cells, n)
        goals = rng.sample(cells, n)
        inst = Instance(layout.cells, layout.adjacency, starts, goals)
        result = plan(inst, timeout=3)
        check(inst, result)
        assert result.complete, (starts, goals, result.expanded)


def test_of_the_shortest_paths_a_straight_one_is_taken():
    for seed in range(10):
        inst = Instance(SIM.cells, SIM.adjacency, starts=[(0, 0)], goals=[(3, 5)])
        result = plan(inst, timeout=1, seed=seed)
        path = [cfg[0] for cfg in result.configs]
        assert len(path) == 9  # 3 + 5 steps: a shortest path
        steps = [(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:])]
        corners = sum(1 for a, b in zip(steps, steps[1:]) if a != b)
        assert corners <= 1, path
