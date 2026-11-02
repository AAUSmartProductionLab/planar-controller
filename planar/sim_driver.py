"""
A kinematic stand-in for the PMC, for testing the coordinator without one.

Xbots move in straight lines with a trapezoidal speed profile and rotate only at flyway centres
and where four flyways meet (the PMC refuses rotation anywhere else). The PMC's auto routing is stood in for by the slot
planner, every xbot stepping from slot to slot at the same pace. Time is whatever the caller says
it is (advance()), so tests run faster than real time. Every overlap of two xbots is recorded as
a collision.

As the Planar Motor simulator does: a paused xbot reads as idle where it stopped, and a motion
sent to it waits too; a payload over MAX_PAYLOAD_KG is refused; the xbots in a zone switched off
read as off; a move through a zone is slowed by its speed (all the way, here), though the time
it is said to take is without; a move into the trail of another's move in flight
(Layout.trail) is stopped with XBOT_OBSTACLE_DETECTED while that one has longer than TRAIL_S to
go. An accident (accident(xid)) switches an xbot off until it is recovered.
"""

import math

from . import mapf
from .driver import Driver, DriverError, Reading
from .layout import POSITION_TOLERANCE_M, hull, overlap, wrap_angle
from .statemachines import ErrorID

MAX_PAYLOAD_KG = 0.5  # an M3-06's, as the simulator takes it
TRAIL_S = 0.65  # a move into another's trail is stopped while that one has longer to go (the simulator's)

OVERLAP_TOLERANCE_M = 0.0005


def _travel_time(distance, speed, accel):
    return mapf.Motion(speed=speed, accel=accel).travel_time(distance)


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
        self.motion = None  # (kind, start, target, t0, total, distance, speed, accel)
        self.fault = None
        self.paused = False
        self.off = False    # switched off: in a zone switched off, or after an accident
        self.accident = False
        self.payload = (0.0, 0.0)


class SimDriver(Driver):
    def __init__(self, layout, xbots, turn_speed=3.0, turn_accel=10.0):
        self.layout = layout
        self.xbots = {xid: SimXbot(*pose) for xid, pose in xbots.items()}
        self.turn_speed, self.turn_accel = turn_speed, turn_accel
        self.now = 0.0
        self.collisions = []
        self.commands = []
        self.turns = []  # (xid, how far it turned, rad, + counterclockwise)
        self.auto = None  # auto routing in progress: (xids, stages, poses, t0, speed)
        self.zones = {}   # zid: [box, speed, state]
        self.max_accel = lambda kg: 20.0  # m/s^2 with a payload of kg (the simulator's: always 20)
        self.flyway_status = {}
        self.master = True  # False: another program has taken mastership
        self.closed = False

    # --- Driver --------------------------------------------------------------------------

    def connect(self):
        if not self.master:
            raise DriverError("another program is the PMC's master", ErrorID.MASTERSHIP_LOST)
        self.recover(list(self.xbots))
        for xb in self.xbots.values():
            xb.paused = False
        self.closed = False
        return self.layout

    def read(self):
        if not self.master:
            raise DriverError("another program has taken mastership of the PMC", ErrorID.MASTERSHIP_LOST)
        out = {}
        for xid, xb in self.xbots.items():
            if xb.off:
                state, detail = "off", "ACCIDENT (XBOT_DISABLED)" if xb.accident else "XBOT_DISABLED"
            elif xb.fault:
                state, detail = "fault", xb.fault
            elif xb.motion and not xb.paused:
                state, detail = "moving", ""
            else:
                state, detail = "idle", ""
            out[xid] = Reading(xb.x, xb.y, xb.rz, state, detail)
        return out

    def move(self, xid, x, y, speed, accel):
        xb = self._ready(xid)
        if not self.layout.on_table(x, y, xb.rz):
            raise DriverError(f"xbot {xid}: ({x}, {y}) is off the table")
        distance = math.hypot(x - xb.x, y - xb.y)
        self.commands.append((self.now, xid, "move", x, y, speed, accel))
        ahead = self.layout.ahead((xb.x, xb.y), (x, y))
        if any(self._stops(o, ahead) for o in self.xbots.values() if o is not xb):
            xb.fault = "XBOT_OBSTACLE_DETECTED"
            return _travel_time(distance, speed, accel)
        swept = hull(self.layout.square(xb.x, xb.y, xb.rz), self.layout.square(x, y, xb.rz))
        f = min((zone[1] for zone in self.zones.values() if overlap(swept, zone[0])), default=1.0)  # [box, speed, state]
        total = _travel_time(distance, speed * f, accel * f)
        xb.motion = ("move", (xb.x, xb.y), (x, y), self.now, total, distance, speed * f, accel * f)
        return _travel_time(distance, speed, accel)

    def _stops(self, other, ahead):
        """Whether a move into ahead is stopped for other's move in flight: it reaches into that
        one's trail, and that one has longer than TRAIL_S to go."""
        m = other.motion
        return (m is not None and m[0] == "move" and m[4] - (self.now - m[3]) > TRAIL_S
                and overlap(ahead, self.layout.trail(m[1], m[2])))

    def rotate(self, xid, rz):
        xb = self._ready(xid)
        if self.layout.spot_at(xb.x, xb.y) is None:
            raise DriverError(f"xbot {xid}: WRONGXBOTPOSITION")
        turn = wrap_angle(rz - xb.rz)  # the shortest way, as PmcDriver turns
        distance = abs(turn)
        total = _travel_time(distance, self.turn_speed, self.turn_accel)
        xb.motion = ("rotate", xb.rz, xb.rz + turn, self.now, total, distance, self.turn_speed, self.turn_accel)
        self.turns.append((xid, turn))
        self.commands.append((self.now, xid, "rotate", rz))
        return total

    def stop(self, xid=0):
        for k, xb in self.xbots.items():
            if xid in (0, k):
                xb.motion = None
                if xb.fault == "XBOT_OBSTACLE_DETECTED":  # as the PMC: idle again
                    xb.fault = None
        if self.auto and (xid == 0 or xid in self.auto[0]):
            for k in self.auto[0]:
                self.xbots[k].motion = None
            self.auto = None

    def auto_drive(self, xids, targets, speed, accel):
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
        self.auto = ([xids[k] for k in order], stages, poses, self.now, speed)
        for xid in xids:
            self.xbots[xid].motion = ("auto",)
        self.commands.append((self.now, 0, "auto", tuple(xids), tuple(targets), speed, accel))

    def close(self):
        self.closed = True

    def pause(self, xid):
        self._xbot(xid).paused = True
        self.commands.append((self.now, xid, "pause"))

    def resume(self, xid):
        self._xbot(xid).paused = False
        self.commands.append((self.now, xid, "resume"))

    def payload(self, xid):
        xb = self._xbot(xid)
        return (*xb.payload, self.max_accel(xb.payload[0]))

    def set_payload(self, xid, kg, cg_height):
        if kg > MAX_PAYLOAD_KG:
            raise DriverError("write_xbot_payload_settings command failed with code INVALIDPARAMS", ErrorID.OUT_OF_RANGE)
        xb = self._xbot(xid)
        xb.payload = (kg, cg_height)
        return self.max_accel(kg)

    def accident(self, xid):
        """The xbot hit something: it stops and the PMC switches it off."""
        xb = self.xbots[xid]
        xb.motion, xb.off, xb.accident = None, True, True

    def recover(self, xids):
        out = []
        for xid in xids:
            xb = self.xbots.get(xid)
            if xb is not None and xb.accident:
                xb.off = xb.accident = False
                out.append(xid)
        return out

    def define_zone(self, zid, box, speed):
        state = self.zones.get(zid, [None, None, "ACTIVATED_UNFENCED"])[2]
        self.zones[zid] = [tuple(box), speed, state]

    def _inside(self, box):
        return [xid for xid, xb in self.xbots.items()
                if overlap(self.layout.square(xb.x, xb.y, xb.rz), box)]

    def zone_status(self, zid):
        box, _, state = self.zones[zid]
        return state, self._inside(box)

    def zone_control(self, zid, operation):
        zone = self.zones[zid]
        box = zone[0]
        if operation == "deactivate":
            zone[2] = "LOADING_FENCED"
            for xid in self._inside(box):
                self.xbots[xid].off = True
        elif operation == "activate":
            zone[2] = "ACTIVATED_FENCED"
            for xid in self._inside(box):
                if not self.xbots[xid].accident:
                    self.xbots[xid].off = False
        elif operation == "unfence":
            if zone[2] != "ACTIVATED_FENCED":
                raise DriverError(f"zone {zid} is {zone[2]}")
            zone[2] = "ACTIVATED_UNFENCED"

    def flyways(self):
        return dict(self.flyway_status)

    # --- simulation ----------------------------------------------------------------------

    def put(self, xid, x, y, rz=0.0):
        """Put an xbot on the table (by hand, in a zone switched off); it is off until the zone is on."""
        xb = self.xbots[xid] = SimXbot(x, y, rz)
        xb.off = True

    def take(self, xid):
        """Take an xbot off the table."""
        del self.xbots[xid]

    def _xbot(self, xid):
        xb = self.xbots.get(xid)
        if xb is None:
            raise DriverError(f"xbot {xid} not detected")
        return xb

    def _ready(self, xid):
        xb = self._xbot(xid)
        if xb.off:
            raise DriverError(f"xbot {xid} is switched off")
        if xb.fault:
            raise DriverError(f"xbot {xid}: {xb.fault}")
        if xb.motion:
            raise DriverError(f"xbot {xid} is still moving")
        return xb

    def advance(self, now):
        dt, self.now = now - self.now, now
        if self.auto:
            self._advance_auto(now)
        for xb in self.xbots.values():
            if not xb.motion or xb.motion[0] == "auto":
                continue
            kind, start, target, t0, total, distance, speed, accel = xb.motion
            if xb.paused:  # the time paused does not count
                xb.motion = (kind, start, target, t0 + dt, total, distance, speed, accel)
                continue
            f = _fraction(now - t0, total, distance, speed, accel)
            if kind == "move":
                xb.x = start[0] + (target[0] - start[0]) * f
                xb.y = start[1] + (target[1] - start[1]) * f
            else:
                xb.rz = start + (target - start) * f
            if f >= 1.0:
                if kind == "rotate":
                    xb.rz = wrap_angle(xb.rz)
                xb.motion = None
        self._check_overlaps()

    def _advance_auto(self, now):
        xids, stages, poses, t0, speed = self.auto
        steps = (now - t0) * speed / self.layout.pitch  # a stage takes one slot's pitch at speed
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
