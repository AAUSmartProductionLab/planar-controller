"""
Geometry of the planar table.

The table is a grid of flyways (240 mm), some of which may be missing (holes). The planner does
not work in millimetres: it works on a lattice of slots one xbot wide (120 mm), so a flyway holds
2 x 2 slots. An xbot parked on a slot centre with its sides along the axes covers exactly that
slot, and two xbots on neighbouring slots touch without overlapping, as in the PMC's own home
layout.

Positions are in metres and radians, as pmclib uses them.
"""

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from functools import cached_property

Cell = tuple[int, int]

# Touching is not overlapping: a footprint edge that lies within this distance of a slot border
# does not count the slot beyond it (position noise, rounding).
EDGE_TOLERANCE_M = 0.0002
# An xbot this close to a slot centre is on that slot and covers it alone. The same as the edge
# tolerance, so that an xbot a little off a slot centre covers the slot it reaches into.
SLOT_TOLERANCE_M = EDGE_TOLERANCE_M
# A requested pose is a slot or flyway centre only if it is exactly one (but for rounding).
EXACT_M = 1e-6
# How close a move has to end to its target to have arrived, and an xbot to a flyway centre to turn.
POSITION_TOLERANCE_M = 0.0005
ANGLE_TOLERANCE_RAD = 0.005


def wrap_angle(rz):
    """The angle in (-pi, pi]."""
    rz = math.fmod(rz, 2 * math.pi)
    if rz <= -math.pi:
        rz += 2 * math.pi
    elif rz > math.pi:
        rz -= 2 * math.pi
    return rz


def angle_difference(a, b):
    return abs(wrap_angle(a - b))


def overlap(a, b):
    """Whether two boxes overlap by more than touching."""
    return (a[0] < b[2] - EDGE_TOLERANCE_M and b[0] < a[2] - EDGE_TOLERANCE_M
            and a[1] < b[3] - EDGE_TOLERANCE_M and b[1] < a[3] - EDGE_TOLERANCE_M)


def hull(*boxes):
    """The smallest box around boxes: what a straight move from one to another sweeps, and more."""
    return (min(b[0] for b in boxes), min(b[1] for b in boxes), max(b[2] for b in boxes), max(b[3] for b in boxes))


def is_square_aligned(rz, tolerance=ANGLE_TOLERANCE_RAD):
    """True when the xbot's sides lie along the axes (rz a multiple of 90 degrees)."""
    quarter = math.pi / 2
    return abs(rz - round(rz / quarter) * quarter) < tolerance


@dataclass(frozen=True)
class Layout:
    cols: int
    rows: int
    holes: frozenset = frozenset()
    flyway_size: float = 0.24
    xbot_size: float = 0.12

    def __post_init__(self):
        per = self.flyway_size / self.xbot_size
        if abs(per - round(per)) > 1e-9:
            raise ValueError("the flyway size must be a whole number of xbot widths")

    # --- the lattice -----------------------------------------------------------------------

    @property
    def pitch(self):
        return self.xbot_size

    @property
    def per_flyway(self):
        return round(self.flyway_size / self.xbot_size)

    @property
    def width(self):
        return self.cols * self.flyway_size

    @property
    def height(self):
        return self.rows * self.flyway_size

    @cached_property
    def cells(self):
        n = self.per_flyway
        return frozenset(
            (i, j)
            for i in range(self.cols * n)
            for j in range(self.rows * n)
            if (i // n, j // n) not in self.holes
        )

    @cached_property
    def adjacency(self):
        return {c: [d for d in self._four(c) if d in self.cells] for c in self.cells}

    @staticmethod
    def _four(c):
        i, j = c
        return [(i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)]

    def center(self, cell):
        return ((cell[0] + 0.5) * self.pitch, (cell[1] + 0.5) * self.pitch)

    def cell_at(self, x, y, rz=0.0, tolerance=SLOT_TOLERANCE_M):
        """The slot an xbot at this pose is parked on, or None if it is not on one."""
        if not is_square_aligned(rz):
            return None
        cell = (math.floor(x / self.pitch), math.floor(y / self.pitch))
        cx, cy = self.center(cell)
        if cell in self.cells and abs(x - cx) < tolerance and abs(y - cy) < tolerance:
            return cell
        return None

    def footprint(self, x, y, rz=0.0):
        """Every slot the xbot's square overlaps at this pose (some may not exist)."""
        half = self.xbot_size / 2 * (abs(math.cos(rz)) + abs(math.sin(rz)))
        i0 = math.floor((x - half + EDGE_TOLERANCE_M) / self.pitch)
        i1 = math.floor((x + half - EDGE_TOLERANCE_M) / self.pitch)
        j0 = math.floor((y - half + EDGE_TOLERANCE_M) / self.pitch)
        j1 = math.floor((y + half - EDGE_TOLERANCE_M) / self.pitch)
        return frozenset((i, j) for i in range(i0, i1 + 1) for j in range(j0, j1 + 1))

    def on_table(self, x, y, rz=0.0):
        return self.footprint(x, y, rz) <= self.cells

    # --- boxes: (x0, y0, x1, y1), what an xbot covers or a move sweeps -----------------------

    def square(self, x, y, rz=0.0):
        """The axis-aligned box around the xbot at this pose."""
        half = self.xbot_size / 2 * (abs(math.cos(rz)) + abs(math.sin(rz)))
        return (x - half, y - half, x + half, y + half)

    def cell_box(self, cell):
        x, y = self.center(cell)
        return self.square(x, y)

    def flyway_box(self, flyway):
        c, r = flyway
        s = self.flyway_size
        return (c * s, r * s, (c + 1) * s, (r + 1) * s)

    def footprint_of(self, box):
        """Every slot a box overlaps, whether it exists or not (on_table for boxes)."""
        x0, y0, x1, y1 = box
        i0, i1 = math.floor((x0 + EDGE_TOLERANCE_M) / self.pitch), math.floor((x1 - EDGE_TOLERANCE_M) / self.pitch)
        j0, j1 = math.floor((y0 + EDGE_TOLERANCE_M) / self.pitch), math.floor((y1 - EDGE_TOLERANCE_M) / self.pitch)
        return frozenset((i, j) for i in range(i0, i1 + 1) for j in range(j0, j1 + 1))

    def cells_in(self, box):
        """The existing slots a box overlaps (by more than touching)."""
        return self.footprint_of(box) & self.cells

    # --- flyways ---------------------------------------------------------------------------

    @cached_property
    def flyways(self):
        return [(c, r) for c in range(self.cols) for r in range(self.rows) if (c, r) not in self.holes]

    def flyway_cells(self, flyway):
        n = self.per_flyway
        c, r = flyway
        return frozenset((c * n + a, r * n + b) for a in range(n) for b in range(n))

    def flyway_center(self, flyway):
        return ((flyway[0] + 0.5) * self.flyway_size, (flyway[1] + 0.5) * self.flyway_size)

    def flyway_at(self, x, y, tolerance=POSITION_TOLERANCE_M):
        """The flyway whose centre this position is on (where the PMC allows rotation), or None."""
        flyway = (math.floor(x / self.flyway_size), math.floor(y / self.flyway_size))
        if flyway not in self.flyways:
            return None
        cx, cy = self.flyway_center(flyway)
        if abs(x - cx) < tolerance and abs(y - cy) < tolerance:
            return flyway
        return None

    def describe(self):
        """The table as text, top row first: '#' a slot, '.' a hole."""
        n = self.per_flyway
        lines = []
        for j in reversed(range(self.rows * n)):
            lines.append("".join("#" if (i, j) in self.cells else "." for i in range(self.cols * n)))
        return "\n".join(lines)


def parse_pmc_config(path):
    """The table layout from a configuration the PMC saved (save_pmc_config_xml_file)."""
    with open(path, "rb") as f:
        content = f.read().replace(b"\x00", b"").decode("ISO-8859-1").strip()
    root = ET.fromstring(content)
    layout = root.find(".//flw/layout")
    if layout is None:
        raise ValueError(f"no flyway layout in {path}")
    cols = int(layout.findtext("mcol"))
    rows = int(layout.findtext("mrow"))
    mapping = layout.find("mapping")
    present = set()
    if mapping is not None:
        col_values = [int(v.text) for v in mapping.find("col").findall("value")]
        row_values = [int(v.text) for v in mapping.find("row").findall("value")]
        present = set(zip(col_values, row_values))
    holes = frozenset((c, r) for c in range(cols) for r in range(rows) if (c, r) not in present)
    return Layout(cols=cols, rows=rows, holes=holes)
