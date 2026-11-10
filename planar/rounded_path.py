"""One axis-aligned XY corner, rounded without changing the mover's orientation."""

import math
from dataclasses import dataclass

from .layout import EXACT_M, hull


@dataclass(frozen=True)
class RoundedPath:
    """Two perpendicular XY legs joined by a quarter-circle; distances are in metres."""
    start: tuple
    corner: tuple
    end: tuple
    radius: float

    def __post_init__(self):
        """Validate finite coordinates, perpendicular axis-aligned legs and a feasible radius."""
        if not all(math.isfinite(v) for p in (self.start, self.corner, self.end) for v in p):
            raise ValueError('path coordinates must be finite')
        a = tuple(c-s for s, c in zip(self.start, self.corner))
        b = tuple(e-c for c, e in zip(self.corner, self.end))
        if not ((abs(a[1]) < EXACT_M and abs(b[0]) < EXACT_M and abs(a[0]) > EXACT_M and abs(b[1]) > EXACT_M)
                or (abs(a[0]) < EXACT_M and abs(b[1]) < EXACT_M and abs(a[1]) > EXACT_M and abs(b[0]) > EXACT_M)):
            raise ValueError('path must contain two perpendicular axis-aligned legs')
        if not math.isfinite(self.radius) or not 0.00001 < self.radius <= min(self.leg_lengths)/2:
            raise ValueError('radius must exceed 10 um and be at most half the shorter leg')

    @property
    def leg_lengths(self):
        """Return the lengths of the original sharp-corner legs in metres."""
        return math.dist(self.start, self.corner), math.dist(self.corner, self.end)

    @property
    def directions(self):
        """Return unit vectors along the incoming and outgoing legs."""
        a, b = self.leg_lengths
        return (tuple((c-s)/a for s, c in zip(self.start, self.corner)),
                tuple((e-c)/b for c, e in zip(self.corner, self.end)))

    @property
    def tangents(self):
        """Return the entry and exit points where the arc meets the straight legs."""
        u, v = self.directions
        return (tuple(c-self.radius*d for c, d in zip(self.corner, u)),
                tuple(c+self.radius*d for c, d in zip(self.corner, v)))

    @property
    def length(self):
        """Return total line-arc-line distance in metres."""
        return sum(self.leg_lengths) - 2*self.radius + math.pi*self.radius/2

    def point(self, distance):
        """Position at distance along the line / quarter-circle / line path."""
        u, v = self.directions
        entry, exit_ = self.tangents
        first = self.leg_lengths[0] - self.radius
        arc = math.pi*self.radius/2
        distance = min(self.length, max(0.0, distance))
        if distance <= first:
            return tuple(s+distance*d for s, d in zip(self.start, u))
        if distance <= first+arc:
            angle = (distance-first)/self.radius
            return tuple(entry[i]+self.radius*(math.sin(angle)*u[i]+(1-math.cos(angle))*v[i])
                         for i in range(2))
        return tuple(exit_[i]+(distance-first-arc)*v[i] for i in range(2))

    def boxes(self, layout, rz=0.0):
        """Conservative swept footprint, including the inside of the rounded corner."""
        entry, exit_ = self.tangents
        return tuple(hull(layout.square(*a, rz), layout.square(*b, rz))
                     for a, b in ((self.start, entry), (entry, exit_), (exit_, self.end)))
