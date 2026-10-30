"""
A kinematic stand-in for the PMC, for testing the coordinator without one.

Xbots move in straight lines with a trapezoidal speed profile and rotate only at flyway centres
(the PMC refuses rotation anywhere else). The PMC's auto routing is stood in for by the slot
planner, every xbot stepping from slot to slot at the same pace. Time is whatever the caller says
it is (advance()), so tests run faster than real time. Every overlap of two xbots is recorded as
a collision.
"""

import math

from . import mapf
from .coordinator import Driver, DriverError, Reading
from .layout import POSITION_TOLERANCE_M, wrap_angle

OVERLAP_TOLERANCE_M = 0.0005


def _travel_time(distance, speed, accel):
    if distance <= 0:
        return 0.0
    if distance < speed * speed / accel:
        return 2 * math.sqrt(distance / accel)
    return distance / speed + speed / accel


def _fraction(t, total, distance, speed, accel):
    """How far along a trapezoidal move of this distance we are after t seconds (0..1)."""
    if t >= total or distance <= 0:
        return 1.0
    v_peak = min(speed, math.sqrt(distance * accel))
    t_acc = v_peak / accel
    d_acc = 0.5 * accel * t_acc * t_acc
    if t < t_acc:
        s = 0.5 * accel * t * t
    elif t < total - t_acc:
        s = d_acc + v_peak * (t - t_acc)
    else:
        r = total - t
        s = distance - 0.5 * accel * r * r
    return min(1.0, s / distance)


class SimXbot:
    def __init__(self, x, y, rz=0.0):
        self.x, self.y, self.rz = x, y, rz
        self.motion = None  # (kind, start, target, t0, total, distance)
        self.fault = None


class SimDriver(Driver):
    def __init__(self, layout, xbots, speed=0.5, accel=2.0, turn_speed=3.0, turn_accel=10.0):
        self.layout = layout
        self.xbots = {xid: SimXbot(*pose) for xid, pose in xbots.items()}
        self.speed, self.accel = speed, accel
        self.turn_speed, self.turn_accel = turn_speed, turn_accel
        self.now = 0.0
        self.collisions = []
        self.commands = []
        self.auto = None  # auto routing in progress: (xids, stages, poses, t0)

    # --- Driver --------------------------------------------------------------------------

    def connect(self):
        return self.layout

    def read(self):
        out = {}
        for xid, xb in self.xbots.items():
            if xb.fault:
                state = "fault"
            elif xb.motion:
                state = "moving"
            else:
                state = "idle"
            out[xid] = Reading(xb.x, xb.y, xb.rz, state, xb.fault or "")
        return out

    def move(self, xid, x, y):
        xb = self._ready(xid)
        if not self.layout.on_table(x, y, xb.rz):
            raise DriverError(f"xbot {xid}: ({x}, {y}) is off the table")
        distance = math.hypot(x - xb.x, y - xb.y)
        total = _travel_time(distance, self.speed, self.accel)
        xb.motion = ("move", (xb.x, xb.y), (x, y), self.now, total, distance)
        self.commands.append((self.now, xid, "move", x, y))
        return total

    def rotate(self, xid, rz):
        xb = self._ready(xid)
        if self.layout.flyway_at(xb.x, xb.y) is None:
            raise DriverError(f"xbot {xid}: WRONGXBOTPOSITION")
        distance = abs(rz - xb.rz)
        total = _travel_time(distance, self.turn_speed, self.turn_accel)
        xb.motion = ("rotate", xb.rz, wrap_angle(rz), self.now, total, distance)
        self.commands.append((self.now, xid, "rotate", rz))
        return total

    def stop(self, xid=0):
        for k, xb in self.xbots.items():
            if xid in (0, k):
                xb.motion = None
        if self.auto and (xid == 0 or xid in self.auto[0]):
            for k in self.auto[0]:
                self.xbots[k].motion = None
            self.auto = None

    def auto_drive(self, xids, targets):
        layout = self.layout
        starts = []
        for xid in xids:
            xb = self._ready(xid)
            cell = layout.cell_at(xb.x, xb.y, xb.rz)
            if cell is None:
                raise DriverError(f"xbot {xid} is not on a slot (this simulation routes slot to slot)")
            starts.append(cell)
        others = set()
        for k, xb in self.xbots.items():
            if k not in xids:
                others |= layout.footprint(xb.x, xb.y, xb.rz)
        cells = layout.cells - others
        goals = []
        free = set(starts)
        for x, y in targets:
            goal = layout.cell_at(x, y)
            if goal is None or goal not in cells:
                raise DriverError(f"target ({x}, {y}) is not a free slot")
            # any xbot to any target: the nearest one not given a target yet
            k = min((i for i, s in enumerate(starts) if s in free),
                    key=lambda i: abs(starts[i][0] - goal[0]) + abs(starts[i][1] - goal[1]))
            free.discard(starts[k])
            goals.append((k, goal))
        order = [k for k, _ in goals]
        inst = mapf.Instance(cells, layout.adjacency, [starts[k] for k in order], [g for _, g in goals])
        result = mapf.plan(inst, timeout=2.0)
        if not result.complete:
            raise DriverError("auto routing found no way")
        # Each step slot by slot, and in it an xbot moving into a slot another leaves only after
        # that one has left it (else their corners would meet when one turns off).
        stages, poses = [], [list(result.configs[0])]
        for a, b in zip(result.configs, result.configs[1:]):
            leaver = {a[i]: i for i in range(len(a)) if a[i] != b[i]}
            depth = {}

            def level(i):
                if i not in depth:
                    j = leaver.get(b[i])
                    depth[i] = 0 if j is None else level(j) + 1
                return depth[i]

            moving = [i for i in range(len(a)) if a[i] != b[i]]
            for lv in range(max(level(i) for i in moving) + 1):
                stage = [i for i in moving if depth[i] == lv]
                stages.append(stage)
                nxt = list(poses[-1])
                for i in stage:
                    nxt[i] = b[i]
                poses.append(nxt)
        self.auto = ([xids[k] for k in order], stages, poses, self.now)
        for xid in xids:
            self.xbots[xid].motion = ("auto",)
        self.commands.append((self.now, 0, "auto", tuple(xids), tuple(targets)))

    # --- simulation ----------------------------------------------------------------------

    def _ready(self, xid):
        xb = self.xbots.get(xid)
        if xb is None:
            raise DriverError(f"xbot {xid} not detected")
        if xb.fault:
            raise DriverError(f"xbot {xid}: {xb.fault}")
        if xb.motion:
            raise DriverError(f"xbot {xid} is still moving")
        return xb

    def advance(self, now):
        self.now = now
        if self.auto:
            self._advance_auto(now)
        for xb in self.xbots.values():
            if not xb.motion or xb.motion[0] == "auto":
                continue
            kind, start, target, t0, total, distance = xb.motion
            if kind == "move":
                f = _fraction(now - t0, total, distance, self.speed, self.accel)
                xb.x = start[0] + (target[0] - start[0]) * f
                xb.y = start[1] + (target[1] - start[1]) * f
            else:
                f = _fraction(now - t0, total, distance, self.turn_speed, self.turn_accel)
                xb.rz = start + (target - start) * f
            if f >= 1.0:
                xb.motion = None
        self._check_overlaps()

    def _advance_auto(self, now):
        xids, stages, poses, t0 = self.auto
        steps = (now - t0) * self.speed / self.layout.pitch  # a stage takes one slot's pitch at speed
        k = min(int(steps), len(stages))
        f = steps - k if k < len(stages) else 0.0
        for i, xid in enumerate(xids):
            a = self.layout.center(poses[k][i])
            b = self.layout.center(poses[min(k + 1, len(stages))][i])
            xb = self.xbots[xid]
            xb.x, xb.y = a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f
        if k == len(stages):
            for xid in xids:
                self.xbots[xid].motion = None
            self.auto = None

    def _half(self, xb):
        if xb.motion and xb.motion[0] == "rotate":
            return self.layout.xbot_size / math.sqrt(2)  # the circle it sweeps
        return self.layout.xbot_size / 2 * (abs(math.cos(xb.rz)) + abs(math.sin(xb.rz)))

    def _check_overlaps(self):
        items = list(self.xbots.items())
        for a in range(len(items)):
            for b in range(a + 1, len(items)):
                (ia, xa), (ib, xbb) = items[a], items[b]
                reach = self._half(xa) + self._half(xbb) - OVERLAP_TOLERANCE_M
                if abs(xa.x - xbb.x) < reach and abs(xa.y - xbb.y) < reach:
                    self.collisions.append((self.now, ia, ib, (xa.x, xa.y), (xbb.x, xbb.y)))

    def at(self, xid, x, y, rz=None):
        xb = self.xbots[xid]
        ok = abs(xb.x - x) < POSITION_TOLERANCE_M and abs(xb.y - y) < POSITION_TOLERANCE_M
        return ok and (rz is None or abs(wrap_angle(xb.rz - rz)) < 0.01)
