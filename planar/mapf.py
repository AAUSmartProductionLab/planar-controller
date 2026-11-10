"""Plan dependency-safe multi-xbot routes with LaCAM search and PIBT configuration generation."""

import math
import random
import time
from collections import deque
from dataclasses import dataclass, field

INF = 1 << 30


class Instance:
    """A routing graph, distinct start/goal cells and per-xbot forbidden cells."""

    def __init__(self, cells, adjacency, starts, goals, keep_out=None):
        """Validate starts and goals, restrict the graph and precompute distances to each goal."""
        self.cells = frozenset(cells)
        self.adj = {c: [d for d in adjacency[c] if d in self.cells] for c in self.cells}
        self.radj = {c: [] for c in self.cells}  # the cells each may be reached from
        for c in sorted(self.cells):
            for d in self.adj[c]:
                self.radj[d].append(c)
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
        """Compute reverse shortest-path distances while excluding forbidden cells."""
        goal, keep_out = self.goals[i], self.keep_out[i]
        dist = {goal: 0}
        queue = deque([goal])
        while queue:
            v = queue.popleft()
            for u in self.radj[v]:
                if u not in dist and u not in keep_out:
                    dist[u] = dist[v] + 1
                    queue.append(u)
        return dist

    def may_enter(self, i, cell):
        """Whether the cell is outside this xbot's forbidden set."""
        return cell not in self.keep_out[i]

    def dist(self, i, cell):
        """Return distance to the goal, allowing departure from a forbidden start; INF means unreachable."""
        d = self._dist[i].get(cell)
        if d is not None:
            return d
        # A cell the xbot may only leave: one more than its best neighbour.
        best = min((self._dist[i].get(u, INF) for u in self.adj[cell] if self.may_enter(i, u)), default=INF)
        return best + 1 if best < INF else INF

    def reachable(self, i):
        """Whether this xbot can reach its goal from its start."""
        return self.dist(i, self.starts[i]) < INF

    def successors(self, i, cell):
        """Return waiting and legal adjacent positions from which the goal remains reachable."""
        return [cell] + [v for v in self.adj[cell] if self.may_enter(i, v) and self.dist(i, v) < INF]


def valid_step(instance, q_from, q_to):
    """Reject overlapping destinations, illegal steps, swaps and cyclic following dependencies."""
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
        """Store the routing instance and random generator used to break ties."""
        self.inst = instance
        self.rng = rng

    def __call__(self, q_from, order, constraints, q_prev=None):
        """Propose one joint step under the supplied constraints, or None if invalid."""
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
            """Choose a move, recursively pushing lower-priority occupants with cycle checks."""
            here = q_from[i]
            candidates = inst.successors(i, here)
            self.rng.shuffle(candidates)
            # Nearer the goal first; between equally near ones, on in the direction it came: of
            # the shortest paths, the straight ones (one corner instead of a staircase).
            heading = None
            if q_prev is not None and q_prev[i] != here:
                heading = (here[0] - q_prev[i][0], here[1] - q_prev[i][1])

            def key(v):
                """Rank candidate cells by goal distance, then by whether they change heading."""
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
    """A node in the per-configuration tree of constrained xbot destinations."""
    parent: "_Constraint | None"
    who: int = -1
    where: tuple = None
    depth: int = 0

    def pairs(self):
        """Collect this constraint branch as xbot-index and destination-cell pairs."""
        out = []
        c = self
        while c.parent is not None:
            out.append((c.who, c.where))
            c = c.parent
        return out


@dataclass
class _Node:
    """A LaCAM configuration with its search parent, priority order and constraint frontier."""
    config: tuple
    parent: "_Node | None"
    order: list
    h: int
    tree: deque = field(default_factory=lambda: deque([_Constraint(None)]))


@dataclass
class Result:
    """A complete or partial configuration path with search timing and expansion counts."""
    configs: list          # from the start configuration on
    complete: bool         # True when the last configuration is the goals
    expanded: int = 0
    seconds: float = 0.0


def _order(inst, config):
    """Who PIBT plans first: xbots away from their goals, the farthest first."""
    return sorted(range(inst.n), key=lambda i: (config[i] == inst.goals[i], -inst.dist(i, config[i])))


def _cost(inst, config):
    """Sum the remaining goal distances for a joint configuration."""
    return sum(inst.dist(i, v) for i, v in enumerate(config))


def lacam(instance, timeout=0.5, seed=0):
    """Configurations from the starts to the goals. When there is none, or the time runs out, the result is
    not complete and leads to the configuration nearest the goals found so far.
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
        """Reconstruct configurations from the search root to the supplied node."""
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
            successors = inst.successors(i, node.config[i])
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


@dataclass(frozen=True)
class Motion:
    """How the xbots move, for estimating how long a plan takes: slot pitch (m), m/s, m/s^2."""
    pitch: float = 0.12
    speed: float = 0.5
    accel: float = 2.0

    def travel_time(self, distance):
        """A straight move from standstill to standstill (trapezoidal speed profile)."""
        if distance <= 0:
            return 0.0
        if distance < self.speed * self.speed / self.accel:
            return 2 * math.sqrt(distance / self.accel)
        return distance / self.speed + self.speed / self.accel

    def time_at(self, s, distance):
        """When a straight move of this distance has covered s of it."""
        v = min(self.speed, math.sqrt(distance * self.accel))
        t_acc = v / self.accel
        d_acc = 0.5 * self.accel * t_acc * t_acc
        if s <= d_acc:
            return math.sqrt(2 * s / self.accel)
        if s <= distance - d_acc:
            return t_acc + (s - d_acc) / v
        return self.travel_time(distance) - math.sqrt(2 * max(0.0, distance - s) / self.accel)


def actions(configs, first_id=1, leaving=None):
    """Return per-xbot (id, from, to, dependency) steps. IDs start at first_id; leaving maps cells to prior
    moves that must vacate them.
    """
    n = len(configs[0])
    out = [[] for _ in range(n)]
    last_leave = dict(leaving or {})
    next_id = first_id - 1
    for t in range(1, len(configs)):
        before, after = configs[t - 1], configs[t]
        moves = [(i, before[i], after[i]) for i in range(n) if before[i] != after[i]]
        leaves = {}
        for i, frm, to in moves:
            next_id += 1
            leaves[frm] = next_id
        for i, frm, to in moves:
            out[i].append((leaves[frm], frm, to, leaves.get(to, last_leave.get(to))))
        last_leave.update(leaves)
    return out


def execution_time(configs, motion=Motion()):
    """Estimate makespan and summed completion times using dependency-gated straight runs; motion is shared
    or supplied per xbot.
    """
    per_xbot = actions(configs)
    motions = motion if isinstance(motion, list) else [motion] * len(per_xbot)
    done = {}  # action id -> when it was made (the xbot reached the next slot centre)
    free = [0.0] * len(per_xbot)
    nxt = [0] * len(per_xbot)
    progress = True
    while progress:
        progress = False
        for i, acts in enumerate(per_xbot):
            if nxt[i] >= len(acts):
                continue
            first = acts[nxt[i]]
            if first[3] is not None and first[3] not in done:
                continue
            start = max(free[i], done.get(first[3], 0.0))
            run = [first]
            step = (first[2][0] - first[1][0], first[2][1] - first[1][1])
            for a in acts[nxt[i] + 1:]:
                if a[1] != run[-1][2] or (a[2][0] - a[1][0], a[2][1] - a[1][1]) != step:
                    break
                if a[3] is not None and done.get(a[3], math.inf) > start:
                    break
                run.append(a)
            m = motions[i]
            distance = len(run) * m.pitch
            for k, a in enumerate(run):
                done[a[0]] = start + m.time_at((k + 1) * m.pitch, distance)
            free[i] = start + m.travel_time(distance)
            nxt[i] += len(run)
            progress = True
    if any(nxt[i] < len(acts) for i, acts in enumerate(per_xbot)):
        return math.inf, math.inf  # cannot happen for a valid plan
    return max(free, default=0.0), sum(free)


def plan(instance, timeout=0.5, budget=0.05, seed=0, motion=Motion(), max_seeds=16):
    """Try LaCAM seeds within timeout and budget, choosing the complete plan with the shortest estimated
    execution time.
    """
    t0 = time.monotonic()
    best, best_key = None, None
    for k in range(max_seeds):
        left = timeout - (time.monotonic() - t0)
        if left <= 0:
            break
        result = lacam(instance, timeout=left, seed=seed + k)
        key = (not result.complete, *execution_time(result.configs, motion))
        if best is None or key < best_key:
            best, best_key = result, key
        if not result.complete or time.monotonic() - t0 >= budget:
            break  # incomplete: it ran out of time, or there is no way
    best.seconds = time.monotonic() - t0
    return best
