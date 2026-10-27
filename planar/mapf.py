"""
Planning paths for several xbots at once on the slot lattice.

The plan is a sequence of configurations (one cell per xbot). From one configuration to the next
every xbot stays or moves to a neighbouring cell, and:

- no two xbots are in the same cell;
- no two xbots swap cells;
- an xbot may move into a cell another xbot leaves in the same step (it follows), but no ring
  of xbots follows each other, because the executor starts a follower only once the xbot ahead
  of it has left, and in a ring every xbot would wait for the next.

The configurations come from LaCAM (Okumura, "LaCAM: Search-Based Algorithm for Quick
Multi-Agent Pathfinding", AAAI 2023), which searches over whole configurations and lets PIBT
(Okumura et al., "Priority Inheritance with Backtracking for Iterative Multi-agent Path
Finding", AIJ 2022) propose each next one. PIBT pushes xbots that are in the way and lets them
push others in turn, which is what gets two xbots past each other in a one-lane passage; LaCAM
makes the search complete: if every xbot can reach its goal, it finds a way.
"""

import random
import time
from collections import deque
from dataclasses import dataclass, field

INF = 1 << 30


class Instance:
    """
    One planning problem.

    cells: the cells the xbots may use; adjacency: their neighbours (within cells);
    starts, goals: one cell each per xbot; keep_out: per xbot, cells it may not enter (it may
    leave one it starts in).
    """

    def __init__(self, cells, adjacency, starts, goals, keep_out=None):
        self.cells = frozenset(cells)
        self.adj = {c: [d for d in adjacency[c] if d in self.cells] for c in self.cells}
        self.starts = tuple(starts)
        self.goals = tuple(goals)
        self.n = len(self.starts)
        self.keep_out = [frozenset(k) for k in keep_out] if keep_out else [frozenset()] * self.n
        for i in range(self.n):
            if self.starts[i] not in self.cells or self.goals[i] not in self.cells:
                raise ValueError(f"xbot {i}: start or goal is not a usable cell")
        if len(set(self.starts)) != self.n or len(set(self.goals)) != self.n:
            raise ValueError("starts and goals must be distinct")
        self._dist = [self._distances(i) for i in range(self.n)]

    def _distances(self, i):
        goal, keep_out = self.goals[i], self.keep_out[i]
        dist = {goal: 0}
        queue = deque([goal])
        while queue:
            v = queue.popleft()
            for u in self.adj[v]:
                if u not in dist and u not in keep_out:
                    dist[u] = dist[v] + 1
                    queue.append(u)
        return dist

    def may_enter(self, i, cell):
        return cell not in self.keep_out[i]

    def dist(self, i, cell):
        d = self._dist[i].get(cell)
        if d is not None:
            return d
        # A cell the xbot may only leave: one more than its best neighbour.
        best = min((self._dist[i].get(u, INF) for u in self.adj[cell] if self.may_enter(i, u)), default=INF)
        return best + 1 if best < INF else INF

    def reachable(self, i):
        return self.dist(i, self.starts[i]) < INF


def valid_step(instance, q_from, q_to):
    """Whether q_to may follow q_from (see the module's rules)."""
    if len(set(q_to)) != len(q_to):
        return False
    occupant = {v: i for i, v in enumerate(q_from)}
    follows = {}
    for i, (a, b) in enumerate(zip(q_from, q_to)):
        if a == b:
            continue
        if b not in instance.adj[a] or not instance.may_enter(i, b):
            return False
        j = occupant.get(b)
        if j is not None:
            if q_to[j] == a:
                return False
            follows[i] = j
    for first in follows:
        seen = set()
        i = first
        while i in follows:
            if i in seen:
                return False
            seen.add(i)
            i = follows[i]
    return True


class _Pibt:
    """PIBT as LaCAM's configuration generator: one step for all xbots, under constraints."""

    def __init__(self, instance, rng):
        self.inst = instance
        self.rng = rng

    def __call__(self, q_from, order, constraints, q_prev=None):
        inst = self.inst
        q_to = [None] * inst.n
        occupant = {v: i for i, v in enumerate(q_from)}
        claimed = {}
        for i, v in constraints:
            if v in claimed:
                return None
            if v != q_from[i] and (v not in inst.adj[q_from[i]] or not inst.may_enter(i, v)):
                return None
            q_to[i] = v
            claimed[v] = i

        def plan(i, ancestors):
            here = q_from[i]
            candidates = [here] + [v for v in inst.adj[here] if inst.may_enter(i, v)]
            self.rng.shuffle(candidates)
            # Nearer the goal first; between equally near ones, on in the direction it came: of
            # the shortest paths, the straight ones (one corner instead of a staircase).
            heading = None
            if q_prev is not None and q_prev[i] != here:
                heading = (here[0] - q_prev[i][0], here[1] - q_prev[i][1])

            def key(v):
                turns = heading is not None and v != here and (v[0] - here[0], v[1] - here[1]) != heading
                return inst.dist(i, v), turns

            candidates.sort(key=key)
            for v in candidates:
                if v in claimed:
                    continue
                j = occupant.get(v)
                if j is not None and j != i:
                    # Into an ancestor's cell would close a ring; into the cell of one that
                    # moves to ours would be a swap.
                    if j in ancestors or q_to[j] == here:
                        continue
                q_to[i] = v
                claimed[v] = i
                if j is not None and j != i and q_to[j] is None and not plan(j, ancestors | {i}):
                    continue
                return True
            q_to[i] = here
            claimed[here] = i
            return False

        for i in order:
            if q_to[i] is None:
                plan(i, frozenset())
        q = tuple(q_to)
        return q if valid_step(inst, q_from, q) else None


@dataclass
class _Constraint:
    parent: "_Constraint | None"
    who: int = -1
    where: tuple = None
    depth: int = 0

    def pairs(self):
        out = []
        c = self
        while c.parent is not None:
            out.append((c.who, c.where))
            c = c.parent
        return out


@dataclass
class _Node:
    config: tuple
    parent: "_Node | None"
    order: list
    h: int
    tree: deque = field(default_factory=lambda: deque([_Constraint(None)]))


@dataclass
class Result:
    configs: list          # from the start configuration on
    complete: bool         # True when the last configuration is the goals
    expanded: int = 0
    seconds: float = 0.0


def _order(inst, config):
    """Who PIBT plans first: xbots away from their goals, the farthest first."""
    return sorted(range(inst.n), key=lambda i: (config[i] == inst.goals[i], -inst.dist(i, config[i])))


def _cost(inst, config):
    return sum(inst.dist(i, v) for i, v in enumerate(config))


def lacam(instance, timeout=0.5, seed=0):
    """
    Configurations from the starts to the goals. When there is none, or the time runs out,
    the result is not complete and leads to the configuration nearest the goals found so far.
    """
    t0 = time.monotonic()
    inst = instance
    rng = random.Random(seed)
    pibt = _Pibt(inst, rng)
    start = inst.starts
    root = _Node(start, None, _order(inst, start), _cost(inst, start))
    open_ = [root]
    explored = {start: root}
    best = root
    expanded = 0

    def path(node):
        configs = []
        while node is not None:
            configs.append(node.config)
            node = node.parent
        return configs[::-1]

    while open_:
        if time.monotonic() - t0 > timeout:
            break
        node = open_[-1]
        if node.config == inst.goals:
            return Result(path(node), True, expanded, time.monotonic() - t0)
        if not node.tree:
            open_.pop()
            continue
        constraint = node.tree.popleft()
        if constraint.depth < inst.n:
            i = node.order[constraint.depth]
            here = node.config[i]
            successors = [here] + [v for v in inst.adj[here] if inst.may_enter(i, v)]
            rng.shuffle(successors)
            for v in successors:
                node.tree.append(_Constraint(constraint, i, v, constraint.depth + 1))
        config = pibt(node.config, node.order, constraint.pairs(),
                      node.parent.config if node.parent is not None else None)
        expanded += 1
        if config is None:
            continue
        known = explored.get(config)
        if known is not None:
            open_.append(known)
            continue
        new = _Node(config, node, _order(inst, config), _cost(inst, config))
        open_.append(new)
        explored[config] = new
        if new.h < best.h:
            best = new
    return Result(path(best), best.config == inst.goals, expanded, time.monotonic() - t0)


def _rank(result):
    """Complete first, then the fewest steps, then the fewest moves and corners."""
    configs = result.configs
    moves = corners = 0
    for t in range(1, len(configs)):
        for i, (a, b) in enumerate(zip(configs[t - 1], configs[t])):
            if a == b:
                continue
            moves += 1
            before = configs[t - 2][i] if t >= 2 else a
            if before != a and (a[0] - before[0], a[1] - before[1]) != (b[0] - a[0], b[1] - a[1]):
                corners += 1
    return (not result.complete, len(configs), moves + corners)


def plan(instance, timeout=0.5, tries=3, seed=0):
    """LaCAM with a few seeds while time allows; the shortest complete plan wins."""
    t0 = time.monotonic()
    best = None
    for k in range(tries):
        left = timeout - (time.monotonic() - t0)
        if left <= 0:
            break
        result = lacam(instance, timeout=left, seed=seed + k)
        if best is None or _rank(result) < _rank(best):
            best = result
        if not result.complete:
            break
    best.seconds = time.monotonic() - t0
    return best
