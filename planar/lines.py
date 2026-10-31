"""
Line mode: the xbots travel on lanes laid out on the slot lattice, so the way between two places
is always the same one, instead of anywhere on the table (free mode).

The layout is JSON, in mm in the table's coordinates (as the skill's X and Y):

    {
      "lanes": [
        {"name": "loop", "points": [[60, 60], [900, 60], [900, 660], [60, 660], [60, 60]], "oneWay": true},
        {"name": "cross", "points": [[420, 60], [420, 660]]}
      ],
      "turnSpots": [[120, 600]],
      "idleArea": [[180, 180], [180, 540]]
    }

- lanes: each a line through slot centres, every stretch of it along a row or a column. A one-way
  lane is driven from its first point to its last, the others both ways. Where lanes meet or
  cross an xbot may go from one onto the other; between two lane slots side by side there is no
  way but along a lane. The lanes together must lead from every lane slot to every other.
- A target off the lanes (a station beside them) is reached from the lane slot nearest to it,
  the shortest way across slots of no lane, and left the same way. Only the xbot going there or
  coming from there drives on those slots.
- turnSpots: points in the flyways xbots turn on (the PMC turns an xbot only on a flyway centre);
  none: any flyway. A target turned off the axes is still turned on its own flyway.
- idleArea: a line like a lane's: the loading queue in order, its first point the loading
  position. The free xbots wait there, reached and left like a target off the lanes.

The planner then plans on the directed graph of slots this gives (graph), and keeps every xbot
off the slots it may not use (keep_out).
"""

import json
import math
import urllib.request
from collections import deque
from dataclasses import dataclass


class LineError(ValueError):
    pass


@dataclass(frozen=True)
class Lane:
    name: str
    points: tuple      # ((x, y), ...) in m
    one_way: bool


@dataclass(frozen=True)
class LineSpec:
    """The layout as given, before it is put on a table."""
    lanes: tuple
    turn_spots: tuple  # ((x, y), ...) in m
    idle_area: tuple   # ((x, y), ...) in m
    text: str          # the JSON it was read from


def load(source):
    """The layout from a file or an http(s) URL (e.g. a file element of an AAS submodel). OSError
    if it cannot be read, LineError if it is no layout."""
    if source.startswith(("http://", "https://")):
        with urllib.request.urlopen(source, timeout=10) as response:
            data = response.read()
    else:
        with open(source, "rb") as f:
            data = f.read()
    try:
        return parse(data.decode("utf-8-sig"))
    except UnicodeDecodeError as e:
        raise LineError(f"the line layout is not UTF-8 text: {e}") from None


def parse(text):
    try:
        data = json.loads(text)
        if isinstance(data, str):  # a JSON string holding it (an AAS Property's $value)
            text, data = data, json.loads(data)
    except json.JSONDecodeError as e:
        raise LineError(f"the line layout is not JSON: {e}") from None
    if not isinstance(data, dict):
        raise LineError("the line layout must be a JSON object with lanes, turnSpots and idleArea")
    _known(data, {"lanes", "turnSpots", "idleArea"}, "the line layout")
    lanes = data.get("lanes")
    if not isinstance(lanes, list) or not lanes:
        raise LineError("lanes: a list of at least one lane")
    out = []
    for k, lane in enumerate(lanes):
        if not isinstance(lane, dict):
            raise LineError(f"lanes[{k}]: an object with points (and name, oneWay)")
        name = str(lane.get("name", f"lane {k + 1}"))
        _known(lane, {"name", "points", "oneWay"}, name)
        one_way = lane.get("oneWay", False)
        if not isinstance(one_way, bool):
            raise LineError(f"{name}: oneWay is true or false")
        out.append(Lane(name, _points(lane.get("points"), f"{name}: points", 2), one_way))
    return LineSpec(tuple(out), _points(data.get("turnSpots", []), "turnSpots", 0),
                    _points(data.get("idleArea", []), "idleArea", 0), text)


def _known(data, keys, what):
    unknown = set(data) - keys
    if unknown:
        raise LineError(f"{what}: unknown {', '.join(sorted(unknown))} (known: {', '.join(sorted(keys))})")


def _points(value, what, minimum):
    if not isinstance(value, list) or len(value) < minimum:
        raise LineError(f"{what}: a list of at least {minimum} [x, y] points in mm")
    out = []
    for p in value:
        if not (isinstance(p, list) and len(p) == 2
                and all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in p)):
            raise LineError(f"{what}: {p!r} is not an [x, y] point in mm")
        out.append((p[0] / 1000.0, p[1] / 1000.0))
    return tuple(out)


class Lines:
    """A layout put on a table: the slot graph and what each xbot may use. LineError if it does
    not fit the table."""

    def __init__(self, spec, layout):
        self.spec = spec
        self.layout = layout
        self.lanes = [(lane, self._trace(lane.points, lane.name)) for lane in spec.lanes]
        self.lane_cells = frozenset(c for _, cells in self.lanes for c in cells)
        self.idle = self._trace(spec.idle_area, "idleArea") if spec.idle_area else []
        if len(set(self.idle)) != len(self.idle):
            raise LineError("idleArea: passes a slot twice")
        self.turn_spots = [self._flyway(p) for p in spec.turn_spots]
        self.graph = self._graph()
        self.reverse = {c: [] for c in self.graph}
        for c, out in self.graph.items():
            for d in out:
                self.reverse[d].append(c)
        self._check_connected()
        self._spurs = {}
        self.idle_region = frozenset(self.idle).union(*[self.spur(c) for c in self.idle])

    def _mm(self, cell):
        x, y = self.layout.center(cell)
        return f"({x * 1000:.0f}, {y * 1000:.0f}) mm"

    def _cell(self, point, what):
        cell = self.layout.cell_at(*point)
        if cell is None:
            raise LineError(f"{what}: ({point[0] * 1000:g}, {point[1] * 1000:g}) mm is not a slot centre on the table")
        return cell

    def _trace(self, points, what):
        """The slots a line passes, in order."""
        cells = [self._cell(points[0], what)]
        for point in points[1:]:
            a, b = cells[-1], self._cell(point, what)
            if a[0] != b[0] and a[1] != b[1]:
                raise LineError(f"{what}: from {self._mm(a)} to {self._mm(b)} is not along a row or a column")
            step = ((b[0] > a[0]) - (b[0] < a[0]), (b[1] > a[1]) - (b[1] < a[1]))
            while cells[-1] != b:
                c = (cells[-1][0] + step[0], cells[-1][1] + step[1])
                if c not in self.layout.cells:
                    raise LineError(f"{what}: crosses a hole at {self._mm(c)}")
                cells.append(c)
        return cells

    def _flyway(self, point):
        fs = self.layout.flyway_size
        flyway = (math.floor(point[0] / fs), math.floor(point[1] / fs))
        if flyway not in self.layout.flyways:
            raise LineError(f"turnSpots: ({point[0] * 1000:g}, {point[1] * 1000:g}) mm is not on a flyway")
        return flyway

    def _graph(self):
        """Between two lane slots only along a lane (and only forward on a one-way one); any other
        step between neighbouring slots both ways (who may take it, keep_out decides)."""
        along = set()
        for lane, cells in self.lanes:
            for a, b in zip(cells, cells[1:]):
                along.add((a, b))
                if not lane.one_way:
                    along.add((b, a))
        lanes = self.lane_cells
        return {c: [d for d in self.layout.adjacency[c] if not (c in lanes and d in lanes) or (c, d) in along]
                for c in self.layout.cells}

    def _check_connected(self):
        lanes = self.lane_cells
        first = min(lanes)
        for edges, way in ((self.graph, "to"), (self.reverse, "from")):
            seen, queue = {first}, deque([first])
            while queue:
                v = queue.popleft()
                for u in edges[v]:
                    if u in lanes and u not in seen:
                        seen.add(u)
                        queue.append(u)
            missing = sorted(lanes - seen)
            if missing:
                a, b = (first, missing[0]) if way == "to" else (missing[0], first)
                raise LineError(f"the lanes lead nowhere from {self._mm(a)} to {self._mm(b)}: "
                                "every lane slot must lead to every other (mind the one-way lanes)")

    def spur(self, cell):
        """This slot and the shortest way from it across slots of no lane to the nearest lane slot
        (that one not counted); nothing for a lane slot."""
        if cell in self.lane_cells:
            return frozenset()
        if cell not in self._spurs:
            came = {cell: None}
            queue = deque([cell])
            path = [cell]
            while queue:
                v = queue.popleft()
                if any(u in self.lane_cells for u in self.layout.adjacency[v]):
                    path = []
                    while v is not None:
                        path.append(v)
                        v = came[v]
                    break
                for u in self.layout.adjacency[v]:
                    if u not in came:
                        came[u] = v
                        queue.append(u)
            self._spurs[cell] = frozenset(path)
        return self._spurs[cell]

    def keep_out(self, cells, idle=False):
        """The slots an xbot may not enter that starts on or goes to these slots (idle: one nobody
        holds, which may also wait in the idle area)."""
        allowed = set(self.lane_cells)
        for cell in cells:
            if cell is not None:
                allowed |= self.spur(cell)
        if idle:
            allowed |= self.idle_region
        return self.layout.cells - allowed

    def describe(self):
        """The table as text, top row first: '>' '<' '^' 'v' a one-way lane, '+' a two-way lane
        slot or a crossing, 'I' the idle area, 'T' a turn spot's slot, '#' another slot, '.' a hole."""
        n = self.layout.per_flyway
        marks = {}
        arrows = {(1, 0): ">", (-1, 0): "<", (0, 1): "^", (0, -1): "v"}
        for lane, cells in self.lanes:
            own = {}
            for a, b in zip(cells, cells[1:]):  # the way on from each slot
                own[a] = arrows[(b[0] - a[0], b[1] - a[1])] if lane.one_way else "+"
            own.setdefault(cells[-1], own[cells[-2]] if len(cells) > 1 else "+")
            for c, mark in own.items():
                marks[c] = mark if marks.get(c, mark) == mark else "+"
        for c in self.idle:
            marks.setdefault(c, "I")
        for flyway in self.turn_spots:
            for c in self.layout.flyway_cells(flyway):
                marks.setdefault(c, "T")
        rows = []
        for j in reversed(range(self.layout.rows * n)):
            rows.append("".join(marks.get((i, j), "#") if (i, j) in self.layout.cells else "."
                                for i in range(self.layout.cols * n)))
        return "\n".join(rows)
