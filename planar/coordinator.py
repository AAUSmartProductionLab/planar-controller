"""Coordinate multi-xbot routes, reservations and motion; geometry uses metres and radians."""

import logging
import math
import threading
import time
from collections import deque
from dataclasses import dataclass, field

from . import mapf
from .driver import DriverError
from .layout import (ANGLE_TOLERANCE_RAD, EXACT_M, POSITION_TOLERANCE_M, angle_difference, hull,
                     is_square_aligned, overlap, steps_from, wrap_angle)
from .lines import LineError, Lines
from .statemachines import ErrorID

log = logging.getLogger(__name__)

PROGRESS_TOLERANCE_M = 0.0005
# A straight move from where it stands to its target is taken when nothing is in the way and it
# sweeps no more slots than this; anything longer goes through the planner.
DIRECT_MAX_SLOTS = 16
MOTION_START_GRACE_S = 0.5
# A claim that has waited this long for its slots to clear gives way to an older one still waiting
# (and claims again after as long), so claims that need each other's room cannot wait for ever.
CLAIM_YIELD_S = 5.0
# The PMC's auto routing takes at most this many xbots in one command.
AUTO_DRIVE_MAX = 10
GATHER_TIMEOUT_S = 120.0
ZONE_READ_S = 0.5      # how often the zones' states are read from the PMC
FLYWAY_READ_S = 5.0    # how often the flyways' power and temperatures are
# A move into the trail of another's move in flight (Layout.trail) waits while that one still has
# longer than this to go: Planar Motor's simulator stops it with an obstacle at about 0.65 s.
TRAIL_WAIT_S = 0.4


@dataclass
class Pose:
    """An xbot position in metres and its Rz orientation in radians."""
    x: float
    y: float
    rz: float = 0.0


@dataclass
class Action:
    """A one-slot move and the action IDs that must finish before it starts."""
    id: int
    xid: int
    frm: tuple
    to: tuple
    deps: frozenset


@dataclass
class Motion:
    """An active motion's target, timing, grid actions and reserved trail."""
    kind: str                      # nav, move or rotate
    target: tuple                  # (x, y) or (rz,)
    sent: float
    expected: float                # s it should take: as the PMC says, longer in a slow zone
    actions: list = field(default_factory=list)
    origin: tuple = None
    direction: tuple = None
    trail: tuple = None            # a move's: the box no other xbot is sent into meanwhile (Layout.trail)


@dataclass
class Task:
    """A requested pose, completion callback and scheduling state."""
    target: Pose
    finished: object               # finished(ok, error, results)
    started: float
    seq: int = 0                   # the order requests were made in (the clock may not tell)
    halting: bool = False
    spot: tuple = None             # where it turns, (x, y)
    progress_at: float = 0.0
    claim_after: float = 0.0       # gave way with its claim: not to claim again before this
    internal: bool = False         # the coordinator's own (turning an idle xbot): a request replaces it


@dataclass
class Zone:
    """A configured PMC zone, its covered cells and observed state."""
    id: int                        # the PMC's zone id
    spec: object                   # zones.ZoneSpec
    cells: frozenset               # the slots it overlaps
    state: str = "UNKNOWN"         # the PMC's ZONESTATE name
    xbots: tuple = ()              # inside, as the PMC says
    activating: bool = False       # switched on: unfenced once the PMC has levitated the xbots inside

    @property
    def closed(self):
        """Not to be entered: fenced, switched off, or on its way between."""
        return self.state != "ACTIVATED_UNFENCED"


@dataclass
class Gathering:
    """Track loading-queue placement and subsequent idle-orientation turns."""
    xids: list
    targets: list                  # slots
    sent: float
    done: object                   # done(ok, error)
    routing: bool = True           # the PMC's auto routing; then the turns to the idle orientation


class Agent:
    """Track one xbot's feedback, tasks, reservations and motion limits."""
    def __init__(self, xid):
        """Initialize an xbot with no feedback, task or active motion."""
        self.xid = xid
        self.pose = None
        self.state = "off"
        self.detail = ""
        self.cell = None
        self.footprint = frozenset()
        self.home = None
        self.pinned = False        # the planner may not push it (pin(); or as pinned_by() says)
        self.fault = None
        self.task = None
        self.motion = None
        self.actions = deque()
        self.goal = None           # what the current plan takes it to
        self.want = None           # what it should be planned to now
        self.claim = None          # slots a macro sweeps: others are planned off them
        self.claim_boxes = ()      # the boxes it sweeps: nothing else may be in them
        self.claim_since = 0.0     # when it claimed them
        self.desired = None        # where the last plan wanted it (its goal may stand in for it)
        self.turn_after = 0.0      # a turn to the idle orientation failed: not again before this
        self.ops = deque()
        self.ops_done = None
        self.paused_at = None      # when it was paused (a Hold); None: not paused
        self.motion_paused = False  # the PMC has paused its motion in flight
        self.payload = None        # (kg, centre of gravity height m) as last set or read; None: unknown
        self.max_accel = None      # m/s^2 the PMC allows with it; None: unknown
        self.speed_limit = (None, None)  # (m/s, m/s^2) it keeps to below the table's (set_speed); None: none

    @property
    def present(self):
        """Whether feedback locates the xbot and it is not switched off."""
        return self.pose is not None and self.state != "off"

    @property
    def in_macro(self):
        """Whether docking, rotation or another compound operation is pending or active."""
        return bool(self.ops) or (self.motion is not None and self.motion.kind != "nav")

    @property
    def flying(self):
        """Whether a grid-navigation motion is active."""
        return self.motion is not None and self.motion.kind == "nav"

    @property
    def start(self):
        """The slot a new plan starts it from: where its motion in flight ends, else where it is."""
        return self.motion.actions[-1].to if self.flying else self.cell

    def describe(self):
        """Describe the xbot's grid cell or off-grid footprint for logging."""
        where = self.cell if self.cell is not None else f"docked {sorted(self.footprint)}"
        return f"xbot {self.xid} at {where}"


class Coordinator:
    """Plan and execute dependency-gated motion with shared locking and per-xbot claims."""
    def __init__(self, driver, lock, clock=time.monotonic, *, speed=0.5, accel=2.0, plan_timeout=0.5,
                 plan_budget=0.05, patience_s=None, loading_station=None, idle_rz=None, line_spec=None,
                 line_source=None, zones=()):
        """Configure the driver, clock, routing constraints and SI motion limits."""
        self.driver = driver
        if not (0 < speed < math.inf and 0 < accel < math.inf):
            raise ValueError(f"the speed and acceleration must be more than 0, not {speed:g} m/s, {accel:g} m/s^2")
        self.speed, self.accel = speed, accel  # m/s, m/s^2: every xbot's moves at most (set_speed)
        self._start_speed = (speed, accel)
        self.zone_specs = list(zones)  # zones.ZoneSpec, defined on the PMC at every connect
        self.zones = {}            # name: Zone, once connected
        self.zone_problem = ""     # why the last connect did not define the zones
        self.flyway_status = {}    # (column, row): (power W, CPU, amplifier, motor deg C), read now and then
        self._zones_read_at = 0.0
        self._flyways_read_at = None
        self.line_spec = line_spec  # line mode: the lanes they travel on (see lines.py); None: free
        # line mode: () -> LineSpec, to read the layout again at every connect (a Reset); OSError
        # when it cannot be read (the one read before is used then), LineError when it is no layout.
        self.line_source = line_source
        self.line_problem = ""     # why the last connect did not take the line layout as it is now
        self.lines = None          # the line spec on the table, once connected
        self.graph = None          # the steps the planner may take: {slot: [slots]}
        self.graph_reverse = None
        self.lock = lock
        self.clock = clock
        self.plan_timeout = plan_timeout
        self.plan_budget = plan_budget  # how long to look for a faster plan once one is found
        self.patience_s = patience_s
        self.loading_station = loading_station  # (x, y): idle xbots queue up there
        self.queue = []            # the queue's slots, the one on the station first
        self.idle_rz = None        # idle xbots are turned to this (radians, a multiple of 90 deg)
        if idle_rz is not None and self.set_idle_rz(idle_rz) != ErrorID.NONE:
            raise ValueError(f"the idle orientation must be a multiple of 90 degrees, not {math.degrees(idle_rz):g}")
        self._gathering = None
        # pinned_by(xid) -> bool, asked whenever it matters (the modules know); else pin() decides.
        self.pinned_by = None
        self.layout = None
        self.agents = {}
        self.connected = False
        self.on_connection_lost = None
        self.on_tick = []
        self._done = set()
        self._next_action = 0
        self._signature = None
        self._retry_at = 0.0
        self._requests = 0
        self._jobs = deque()
        self._thread = None
        self._running = False

    # --- the loop ----------------------------------------------------------------------------

    def start(self, period_s=0.02):
        """Start the background control loop with the requested tick period."""
        self._running = True
        self._thread = threading.Thread(target=self._loop, args=(period_s,), name="coordinator", daemon=True)
        self._thread.start()

    def shutdown(self):
        """Stop the control thread and disconnect the driver."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)
        try:
            self.driver.close()
        except DriverError as e:
            log.warning("disconnecting from the PMC: %s", e)

    def _loop(self, period_s):
        """Run periodic ticks, logging failures without terminating the loop."""
        while self._running:
            t0 = time.monotonic()
            try:
                self.tick(self.clock())
            except Exception:
                log.exception("coordinator tick")
            time.sleep(max(0.0, period_s - (time.monotonic() - t0)))

    def run_job(self, job):
        """Run job() in the coordinator's thread before its next tick, without the lock held."""
        self._jobs.append(job)

    def tick(self, now):
        """Read feedback, advance tasks and motions, then replan and dispatch ready moves."""
        while self._jobs:
            job = self._jobs.popleft()
            try:
                job()
            except Exception:
                log.exception("coordinator job")
        with self.lock:
            for hook in list(self.on_tick):  # the modules: their state decides who may be pushed
                hook()
            if self.connected:
                try:
                    readings = self.driver.read()
                except DriverError as e:
                    self._lost(e)
                    readings = None
                if readings is not None:
                    self._update(readings, now)
                    if self.zones and now - self._zones_read_at >= ZONE_READ_S:
                        self._read_zones(now)
                    if self._flyways_read_at is None or now - self._flyways_read_at >= FLYWAY_READ_S:
                        self._flyways_read_at = now
                        self.run_job(self._read_flyways)
                    if self._gathering is not None and self._gathering.routing:
                        self._advance_gathering(now)  # the PMC drives them: nothing else moves
                        return
                    self._advance_motions(now)
                    self._advance_tasks(now)
                    self._plan_if_needed(now)
                    self._dispatch(now)
                    if self._gathering is not None:
                        self._advance_gathering(now)

    # --- connection --------------------------------------------------------------------------

    def _layout_invalid(self, problem):
        """Record a line-layout problem and return LINE_LAYOUT_INVALID."""
        log.error("%s", problem)
        self.line_problem = problem
        return ErrorID.LINE_LAYOUT_INVALID

    def connect(self, xids=None):
        """Connect and initialize layout, xbots and zones; return an ErrorID. Reload line_source if set."""
        try:
            layout = self.driver.connect()
            readings = self.driver.read()
        except DriverError as e:
            log.error("connecting to the PMC: %s", e)
            return e.code
        spec, problem = self.line_spec, ""
        if self.line_source is not None:
            try:
                spec = self.line_source()
            except LineError as e:
                return self._layout_invalid(f"the line layout is no layout: {e}")
            except OSError as e:
                problem = f"the line layout could not be read again ({e}); the one read before is used"
                log.warning("%s", problem)
        lines = None
        if spec is not None:
            try:
                lines = Lines(spec, layout)
            except LineError as e:
                return self._layout_invalid(f"the line layout does not fit the table: {e}")
        zones = {}
        for zid, zone in enumerate(self.zone_specs, start=1):
            x0, y0, x1, y1 = zone.box
            cells = layout.cells_in(zone.box)
            if x0 < -EXACT_M or y0 < -EXACT_M or x1 > layout.width + EXACT_M or y1 > layout.height + EXACT_M or not cells:
                return self._zones_invalid(f"zone {zone.name} is not on the table ({layout.width * 1000:.0f} x "
                                           f"{layout.height * 1000:.0f} mm)")
            try:
                self.driver.define_zone(zid, zone.box, zone.speed)
                state, inside = self.driver.zone_status(zid)
            except DriverError as e:
                return self._zones_invalid(f"zone {zone.name}: {e}")
            zones[zone.name] = Zone(zid, zone, cells, state, tuple(inside))
        payloads = {}
        for xid in readings:
            try:
                payloads[xid] = self.driver.payload(xid)
            except DriverError as e:
                log.warning("xbot %d: reading its payload: %s", xid, e)
        with self.lock:
            self.zones, self.zone_problem = zones, ""
            if spec is not None and self.line_spec is not None and spec.text != self.line_spec.text:
                log.info("line mode: the line layout has changed since it was read before")
            self.line_spec, self.line_problem = spec, problem
            self.layout = layout
            self.lines = lines
            self.graph = lines.graph if lines else layout.adjacency
            self.graph_reverse = lines.reverse if lines else layout.adjacency
            # Every xbot on the table takes part, whether it is offered as a shuttle or not.
            for xid in set(readings) | set(xids or ()):
                self.agents.setdefault(xid, Agent(xid))
            for xid, known in payloads.items():
                if known is not None:
                    kg, cg, max_accel = known
                    self.agents[xid].payload, self.agents[xid].max_accel = (kg, cg), max_accel
            self.connected = True
            self._signature = None
            if lines:
                if self.loading_station:
                    log.warning("line mode: the loading queue is the line layout's idleArea, not LOADING_STATION")
                self.queue = list(lines.idle)
            else:
                self.queue = self._queue_slots() if self.loading_station else []
            self._update(readings, self.clock())
            log.info("PMC connected: table %dx%d flyways, holes %s, xbots %s", layout.cols, layout.rows,
                     sorted(layout.holes), sorted(readings))
            if lines:
                spots = ", ".join(f"({x * 1000:.0f}, {y * 1000:.0f})" for x, y in lines.turn_spots)
                log.info("line mode, %d lanes, turn spots %s:\n%s", len(lines.lanes),
                         f"{spots} mm" if spots else "any flyway centre", lines.describe())
                if len(lines.idle) < len(self.agents):
                    log.warning("line mode: the idle area has %d slots for %d xbots; the free xbots that do not "
                                "fit wait where they are, on a lane in the way", len(lines.idle), len(self.agents))
            else:
                log.info("table slots:\n%s", layout.describe())
            if self.queue:
                log.info("loading queue: %s", self.queue[:len(self.agents)])
            for name, zone in zones.items():
                log.info("zone %s (%d): slots %s, %s%s", name, zone.id, sorted(zone.cells), zone.state,
                         f", speed x{zone.spec.speed:g}" if zone.spec.speed != 1 else "")
        return ErrorID.NONE

    def _zones_invalid(self, problem):
        """Record a zone-configuration problem and return ZONES_INVALID."""
        log.error("%s", problem)
        self.zone_problem = problem
        return ErrorID.ZONES_INVALID

    def _lost(self, error):
        """Drop active work and notify the connection-loss handler with the driver error."""
        if error.code == ErrorID.MASTERSHIP_LOST:
            log.error("%s", error)
        else:
            log.error("PMC connection lost: %s", error)
            error.code = ErrorID.PMC_NOT_CONNECTED
        self.connected = False
        self._gathering = None
        for agent in self.agents.values():
            agent.paused_at = None
            self._drop(agent, error.code)
        if self.on_connection_lost:
            self.on_connection_lost(error.code)

    # --- commands (from the modules, with the lock held) -------------------------------------

    def check_target(self, xid, target) -> ErrorID:
        """Return an ErrorID for an unavailable xbot or an invalid target pose."""
        error = self.check_xbot(xid, clear=False)
        if error != ErrorID.NONE:
            return error
        agent = self.agents[xid]
        if agent.task is not None and not agent.task.internal:
            return ErrorID.BUSY
        if not is_square_aligned(target.rz) and self.layout.spot_at(target.x, target.y, EXACT_M) is None:
            # It can only turn to an angle like that on a spot, and only stay there.
            return ErrorID.ANGLE_NOT_ALLOWED
        if not self.layout.on_table(target.x, target.y, target.rz):
            return ErrorID.NOT_ON_TABLE
        if self.layout.footprint(target.x, target.y, target.rz) & self._closed_cells():
            return ErrorID.ZONE_CLOSED
        if not is_square_aligned(agent.pose.rz) and agent.cell is None \
                and self.layout.spot_at(agent.pose.x, agent.pose.y) is None:
            return ErrorID.XBOT_CROOKED
        return ErrorID.NONE

    def submit(self, xid, target, finished) -> ErrorID:
        """Validate and queue a pose request with a completion callback; return an ErrorID."""
        error = self.check_target(xid, target)
        if error != ErrorID.NONE:
            return error
        agent = self.agents[xid]
        if agent.task is not None:  # its turn to the idle orientation: the request goes first
            agent.task = None
            if not agent.in_macro:
                agent.claim, agent.claim_boxes = None, ()
        self._new_task(agent, Pose(target.x, target.y, wrap_angle(target.rz)), finished)
        log.info("xbot %d: to (%.3f, %.3f, %.1f deg)", xid, target.x, target.y, math.degrees(target.rz))
        return ErrorID.NONE

    def _new_task(self, agent, target, finished, internal=False):
        """Assign a task with its request sequence and initial progress timestamp."""
        now = self.clock()
        self._requests += 1
        agent.task = Task(target, finished, now, seq=self._requests, progress_at=now, internal=internal)

    # --- the idle orientation ----------------------------------------------------------------

    def set_idle_rz(self, rz):
        """Set the idle orientation in quarter-turn radians; None or NaN disables it."""
        if rz is None or math.isnan(rz):
            self.idle_rz = None
            return ErrorID.NONE
        if not is_square_aligned(rz):
            return ErrorID.OUT_OF_RANGE
        quarter = math.pi / 2
        self.idle_rz = wrap_angle(round(rz / quarter) * quarter)
        return ErrorID.NONE

    def _turned_wrong(self, agent):
        """Whether an xbot differs from the configured idle orientation."""
        return (self.idle_rz is not None and agent.pose is not None
                and angle_difference(agent.pose.rz, self.idle_rz) >= ANGLE_TOLERANCE_RAD)

    def _turn_idle(self, agent):
        """Turn it to the idle orientation, back onto the slot it is on."""
        def finished(ok, error, results):
            """Delay another idle-turn attempt if this one fails."""
            if not ok:
                agent.turn_after = self.clock() + CLAIM_YIELD_S * 2
        log.info("xbot %d: turns to the idle orientation %.0f deg", agent.xid, math.degrees(self.idle_rz))
        self._new_task(agent, Pose(*self.layout.center(agent.cell), self.idle_rz), finished, internal=True)

    def halt(self, xid):
        """Finish the committed motion, then fail the task with INTERRUPTED; paused motion waits for
        resume.
        """
        agent = self.agents.get(xid)
        if agent and agent.task:
            agent.task.halting = True

    def abort(self, xid):
        """Stop at once, wherever it is (paused or not); the task is dropped without a word."""
        agent = self.agents.get(xid)
        if agent is None:
            return
        try:
            self.driver.stop(xid)
        except DriverError as e:
            log.error("xbot %d: stop: %s", xid, e)
        agent.task, agent.paused_at = None, None
        self._clear_motion(agent)

    def abort_all(self):
        """Stop every xbot and discard all tasks, gathering and reservations."""
        self._gathering = None
        try:
            self.driver.stop(0)
        except DriverError as e:
            log.error("stop all: %s", e)
        for agent in self.agents.values():
            agent.task, agent.paused_at = None, None
            self._clear_motion(agent)

    def pin(self, xid, pinned=True):
        """Whether the planner must leave the xbot where it is (without pinned_by)."""
        agent = self.agents.get(xid)
        if agent is not None:
            agent.pinned = pinned

    # --- pausing (a module's Hold) -----------------------------------------------------------

    def _agents(self, xid):
        """Xbot xid's agent, or every agent (None)."""
        if xid is None:
            return list(self.agents.values())
        return [self.agents[xid]] if xid in self.agents else []

    def pause(self, xid=None) -> ErrorID:
        """Pause one xbot or all (xid=None), retaining claims and suspending dispatch and timeouts."""
        if not self.connected:
            return ErrorID.PMC_NOT_CONNECTED
        now = self.clock()
        for agent in self._agents(xid):
            if agent.paused_at is not None:
                continue  # (its shuttle was held before the table)
            if agent.motion is not None:
                try:
                    self.driver.pause(agent.xid)
                except DriverError as e:
                    log.error("xbot %d: pausing its motion: %s", agent.xid, e)
                    return e.code
                agent.motion_paused = True
            agent.paused_at = now
        log.info("%s paused", f"xbot {xid}" if xid else "every xbot")
        return ErrorID.NONE

    def resume(self, xid=None) -> ErrorID:
        """Carry on where pause() stopped (None: every xbot); the time paused does not count."""
        error, now = ErrorID.NONE, self.clock()
        resumed = [a for a in self._agents(xid) if a.paused_at is not None]
        for agent in resumed:
            problem = self._resume_motion(agent)
            if problem != ErrorID.NONE:
                error = problem
            dt, agent.paused_at = now - agent.paused_at, None
            if agent.motion is not None:
                agent.motion.sent += dt
            if agent.task is not None:
                agent.task.progress_at += dt
                agent.task.claim_after += dt
            agent.claim_since += dt
            agent.turn_after += dt
        if resumed:
            log.info("%s resumed", f"xbot {xid}" if xid else "every xbot")
        return error

    def _resume_motion(self, agent) -> ErrorID:
        """The PMC carries on with the motion it paused, if it did."""
        if not agent.motion_paused:
            return ErrorID.NONE
        agent.motion_paused = False
        if not self.connected:
            return ErrorID.NONE
        try:
            self.driver.resume(agent.xid)
        except DriverError as e:
            log.error("xbot %d: resuming its motion: %s", agent.xid, e)
            return e.code
        return ErrorID.NONE

    # --- speed -------------------------------------------------------------------------------

    def set_speed(self, xid, speed, accel) -> ErrorID:
        """Set speed and acceleration limits in m/s and m/s^2 for future moves; xid=None sets table limits.
        Zero or NaN restores defaults.
        """
        values = []
        for value in (speed, accel):
            if value is None or math.isnan(value) or value == 0:
                values.append(None)
            elif 0 < value < math.inf:
                values.append(float(value))
            else:
                return ErrorID.OUT_OF_RANGE
        if xid is None:
            self.speed, self.accel = (v or start for v, start in zip(values, self._start_speed))
            log.info("speed %g m/s, acceleration %g m/s^2", self.speed, self.accel)
            return ErrorID.NONE
        agent = self.agents.get(xid)
        if agent is None:
            return ErrorID.XBOT_NOT_DETECTED
        speed, accel = agent.speed_limit = tuple(values)
        log.info("xbot %d: speed at most %s, acceleration at most %s", xid,
                 f"{speed:g} m/s" if speed else "the table's", f"{accel:g} m/s^2" if accel else "the table's")
        return ErrorID.NONE

    def speed_of(self, agent):
        """Return effective speed and acceleration, bounded by table, xbot and payload limits."""
        speed, accel = agent.speed_limit
        return min(self.speed, speed or math.inf), min(self.accel, accel or math.inf, agent.max_accel or math.inf)

    # --- the payload -------------------------------------------------------------------------

    def set_payload(self, xid, kg, cg_height) -> ErrorID:
        """Set an idle xbot's payload in kg and centre-of-gravity height in metres; refresh its
        acceleration limit.
        """
        if not self.connected:
            return ErrorID.PMC_NOT_CONNECTED
        agent = self.agents.get(xid)
        if agent is None or not agent.present:
            return ErrorID.XBOT_NOT_DETECTED
        if agent.motion is not None or agent.ops or agent.actions or agent.paused_at is not None:
            return ErrorID.BUSY
        try:
            max_accel = self.driver.set_payload(xid, kg, cg_height)
        except DriverError as e:
            log.error("xbot %d: payload %.3f kg: %s", xid, kg, e)
            return e.code
        agent.payload, agent.max_accel = (kg, cg_height), max_accel
        log.info("xbot %d: payload %.3f kg, centre of gravity %.0f mm up%s", xid, kg, cg_height * 1000,
                 f"; max acceleration {max_accel:g} m/s^2" if max_accel is not None else "")
        return ErrorID.NONE

    # --- accidents ---------------------------------------------------------------------------

    def recover(self, xid) -> bool:
        """Recover the xbot if the PMC switched it off after an accident; whether it did."""
        try:
            recovered = xid in self.driver.recover([xid])
        except DriverError as e:
            log.error("xbot %d: recovering it after an accident: %s", xid, e)
            return False
        if recovered:
            log.info("xbot %d: recovered after an accident", xid)
        return recovered

    # --- zones -------------------------------------------------------------------------------

    def _closed_cells(self):
        """Return cells covered by fenced or deactivated zones."""
        return frozenset().union(*[z.cells for z in self.zones.values() if z.closed])

    def _closed_boxes(self):
        """Return the bounding boxes of closed zones."""
        return [z.spec.box for z in self.zones.values() if z.closed]

    def _zone_speed(self, cells):
        """Return the lowest zone speed factor over these cells, or 1 if unrestricted."""
        return min((z.spec.speed for z in self.zones.values() if z.cells & cells), default=1.0)

    def deactivate_zone(self, name) -> ErrorID:
        """Fence and deactivate an active zone only when no xbot crosses its boundary or moves through it."""
        zone = self.zones.get(name)
        if not self.connected or zone is None:
            return ErrorID.PMC_NOT_CONNECTED
        if zone.closed:
            return ErrorID.NOT_READY
        for agent in self.agents.values():
            if not agent.present:
                continue
            inside = agent.footprint & zone.cells
            if inside and inside != agent.footprint:
                log.info("zone %s: xbot %d stands across its edge", name, agent.xid)
                return ErrorID.BUSY
            on_the_way = (agent.motion is not None or agent.ops or agent.task is not None) and inside
            planned = any(a.to in zone.cells or a.frm in zone.cells for a in agent.actions)
            flying = agent.flying and any(a.to in zone.cells for a in agent.motion.actions)
            going = agent.task is not None and self._target_cells(agent.task) & zone.cells
            claim = agent.claim and agent.claim & zone.cells
            if on_the_way or planned or flying or going or claim:
                log.info("zone %s: xbot %d is on its way in, out or across", name, agent.xid)
                return ErrorID.BUSY
        try:
            self.driver.zone_control(zone.id, "deactivate")
        except DriverError as e:
            log.error("zone %s: switching it off: %s", name, e)
            return e.code
        zone.state, zone.activating = "DEACTIVATING", False
        self._signature = None
        log.info("zone %s: switched off (xbots inside %s)", name,
                 sorted(a.xid for a in self.agents.values() if a.present and a.footprint & zone.cells))
        return ErrorID.NONE

    def activate_zone(self, name) -> ErrorID:
        """Switch the zone on again: the PMC levitates the xbots inside, then it is unfenced."""
        zone = self.zones.get(name)
        if not self.connected or zone is None:
            return ErrorID.PMC_NOT_CONNECTED
        if zone.state == "ACTIVATED_UNFENCED":
            return ErrorID.NOT_READY
        try:
            if zone.state == "ACTIVATED_FENCED":
                self.driver.zone_control(zone.id, "unfence")
            else:
                self.driver.zone_control(zone.id, "activate")
        except DriverError as e:
            log.error("zone %s: switching it on: %s", name, e)
            return e.code
        zone.activating = True
        log.info("zone %s: switched on", name)
        return ErrorID.NONE

    def _read_zones(self, now):
        """Refresh zone states and unfence activated zones once their xbots have levitated."""
        self._zones_read_at = now
        for name, zone in self.zones.items():
            try:
                state, inside = self.driver.zone_status(zone.id)
            except DriverError as e:
                log.debug("zone %s: %s", name, e)
                continue
            if zone.activating and state == "ACTIVATED_FENCED":
                try:
                    self.driver.zone_control(zone.id, "unfence")
                except DriverError as e:
                    log.error("zone %s: removing its fence: %s", name, e)
            if state == "ACTIVATED_UNFENCED":
                zone.activating = False
            if state != zone.state:
                log.info("zone %s: %s, xbots inside %s", name, state, sorted(inside))
                self._signature = None
            zone.state, zone.xbots = state, tuple(inside)

    def _read_flyways(self):
        """Read flyway telemetry outside the coordinator lock, then publish it under the lock."""
        try:
            status = self.driver.flyways()
        except DriverError as e:
            log.debug("reading the flyways: %s", e)
            return
        with self.lock:
            self.flyway_status = status

    def check_xbot(self, xid, clear=True) -> ErrorID:
        """Whether the xbot is there and fit to move; with clear, forgets an earlier fault if it is."""
        if not self.connected:
            return ErrorID.PMC_NOT_CONNECTED
        agent = self.agents.get(xid)
        if agent is None or not agent.present:
            return ErrorID.XBOT_NOT_DETECTED
        if agent.state == "fault":
            return ErrorID.XBOT_FAULT
        if clear:
            agent.fault = None
        elif agent.fault:
            return agent.fault
        return ErrorID.NONE

    def snapshot(self, xid):
        """Return one xbot's (pose, state, detail), or None before feedback is available."""
        agent = self.agents.get(xid)
        if agent is None or agent.pose is None:
            return None
        return agent.pose, agent.state, agent.detail

    # --- the loading queue -------------------------------------------------------------------

    def _queue_slots(self):
        """Order loading-queue cells by distance, favouring table and hole edges."""
        layout = self.layout
        sx, sy = self.loading_station
        head = layout.cell_at(sx, sy, tolerance=layout.pitch)
        if head is None:
            head = min(sorted(layout.cells), key=lambda c: math.dist(layout.center(c), (sx, sy)))
            log.warning("the loading station (%.3f, %.3f) is not over a slot; the queue starts at %s", sx, sy, head)
        steps = steps_from(layout.adjacency, head)
        inside = lambda c: len(layout.adjacency[c]) == 4
        return sorted(steps, key=lambda c: (steps[c] + 2 * inside(c), len(layout.adjacency[c]),
                                            math.dist(layout.center(c), (sx, sy)), c))

    def queue_order(self):
        """The xbots waiting in the loading queue, the one on the station first."""
        at = {a.cell: a.xid for a in self.agents.values()
              if a.cell is not None and a.task is None and a.motion is None and self._is_mover(a)
              and not self._turned_wrong(a)}
        return [at[slot] for slot in self.queue if slot in at]

    def gather(self, xids, done):
        """Place xbots in the loading queue and idle orientation, then call done(ok, error). Fall back to
        planned moves if PMC routing is unavailable.
        """
        agents = [self.agents[x] for x in xids if x in self.agents and self.agents[x].present]
        for agent in agents:
            agent.task = None
            self._clear_motion(agent)
        others = frozenset().union(*[a.footprint for a in self.agents.values() if a.present and a not in agents])
        others |= self._closed_cells()
        targets = [s for s in self.queue if s not in others][:len(agents)]
        self._gathering = Gathering([a.xid for a in agents], targets, self.clock(), done, routing=False)
        if not targets or {a.cell for a in agents} == set(targets):
            return
        if len(agents) > AUTO_DRIVE_MAX:
            log.info("%d xbots: more than one auto routing command takes; the planner lines them up", len(agents))
            return
        speeds = [self.speed_of(a) for a in agents]  # they move as one: the slowest one's
        try:
            self.driver.auto_drive([a.xid for a in agents], [self.layout.center(c) for c in targets],
                                   min(s for s, _ in speeds), min(a for _, a in speeds))
        except DriverError as e:
            log.warning("auto routing into the loading queue: %s; the planner lines them up", e)
            return
        log.info("auto routing xbots %s into the loading queue %s", [a.xid for a in agents], targets)
        self._gathering.routing = True

    def cancel_gather(self):
        """Stop gathering motions and discard the pending gathering operation."""
        g, self._gathering = self._gathering, None
        if g is None:
            return
        try:
            self.driver.stop(0)
        except DriverError as e:
            log.error("stop all: %s", e)
        for xid in g.xids:
            agent = self.agents[xid]
            if agent.task is not None and agent.task.internal:
                agent.task = None
            self._clear_motion(agent)

    def _advance_gathering(self, now):
        """Monitor gathering, perform remaining placement or turns, and report completion or failure."""
        g = self._gathering
        agents = [self.agents[x] for x in g.xids]
        if g.routing and now - g.sent < MOTION_START_GRACE_S:
            return
        if any(not a.present or a.state == "fault" or a.fault for a in agents):
            self.cancel_gather()
            g.done(False, ErrorID.XBOT_FAULT)
        elif now - g.sent > GATHER_TIMEOUT_S:
            log.error("lining up and turning the xbots did not finish in %.0f s", GATHER_TIMEOUT_S)
            self.cancel_gather()
            g.done(False, ErrorID.MOTION_TIMEOUT)
        elif g.routing:
            if all(a.state == "idle" for a in agents):
                g.routing = False
                missing = [a.xid for a in agents if a.cell not in g.targets]
                if missing:
                    log.warning("auto routing left xbots %s outside the loading queue; the planner lines them up",
                                missing)
                for agent in agents:
                    self._classify(agent)  # the turns follow next time round
        else:
            # The planner lines them up meanwhile; each turns as soon as it is on a slot (a turn
            # that failed, once its pause is over).
            for agent in agents:
                if (agent.task is None and agent.cell is not None and self._turned_wrong(agent)
                        and now >= agent.turn_after):
                    self._turn_idle(agent)
            if any(a.task is not None or a.in_macro or a.flying or a.actions or self._turned_wrong(a)
                   for a in agents):
                return
            self._gathering = None
            g.done(True, ErrorID.NONE)

    # --- reading the table -------------------------------------------------------------------

    def _update(self, readings, now):
        """Apply driver readings and fail tasks whose xbots disappear or fault."""
        for xid in readings.keys() - self.agents.keys():  # put on the table in a zone switched off
            log.info("xbot %d is new on the table", xid)
            self.agents[xid] = Agent(xid)
            self._signature = None
        for xid, agent in self.agents.items():
            r = readings.get(xid)
            if r is None:
                agent.state, agent.detail = "off", "not detected"
                if agent.motion or agent.task:
                    self._fail(agent, ErrorID.XBOT_NOT_DETECTED)
                continue
            agent.pose = Pose(r.x, r.y, r.rz)
            agent.state, agent.detail = r.state, r.detail
            if r.state == "fault" and agent.fault is None:
                log.error("xbot %d: %s", xid, r.detail)
                self._fail(agent, ErrorID.XBOT_FAULT)
            elif r.state == "off" and (agent.motion or agent.ops or agent.task):
                log.error("xbot %d: switched off (%s) on its way", xid, r.detail)
                self._fail(agent, ErrorID.XBOT_NOT_DETECTED)
            if agent.motion is None and not agent.ops:
                self._classify(agent)

    def _classify(self, agent):
        """Update an xbot's grid cell, footprint and initial home from its pose."""
        p = agent.pose
        cell = self.layout.cell_at(p.x, p.y, p.rz)
        agent.cell = cell
        if cell is not None:
            agent.footprint = frozenset([cell])
            if agent.home is None:
                agent.home = cell
        else:
            agent.footprint = self.layout.footprint(p.x, p.y, p.rz) & self.layout.cells

    # --- motions in flight -------------------------------------------------------------------

    def _advance_motions(self, now):
        """Release completed grid steps and detect arrival, timeout or position deviation."""
        for agent in self.agents.values():
            m = agent.motion
            if m is None or agent.pose is None or agent.paused_at is not None:
                continue  # (paused, it reads as idle where it stopped)
            p = agent.pose
            if m.kind == "nav":
                along = (p.x - m.origin[0]) * m.direction[0] + (p.y - m.origin[1]) * m.direction[1]
                for k, action in enumerate(m.actions):
                    if along >= (k + 1) * self.layout.pitch - PROGRESS_TOLERANCE_M:
                        self._done.add(action.id)
            if m.kind == "rotate":
                arrived = agent.state == "idle" and angle_difference(p.rz, m.target[0]) < ANGLE_TOLERANCE_RAD
            else:
                arrived = (agent.state == "idle" and abs(p.x - m.target[0]) < POSITION_TOLERANCE_M
                           and abs(p.y - m.target[1]) < POSITION_TOLERANCE_M)
            if arrived:
                agent.motion = None
                if agent.task:
                    agent.task.progress_at = now
                if m.kind == "nav":
                    self._done.update(a.id for a in m.actions)
                    agent.cell = m.actions[-1].to
                    agent.footprint = frozenset([agent.cell])
                elif not agent.ops:
                    then, agent.ops_done = agent.ops_done, None
                    if then:
                        then()
            elif now - m.sent > m.expected * 1.5 + 2.0:
                log.error("xbot %d: %s to %s did not arrive in %.1f s", agent.xid, m.kind, m.target, now - m.sent)
                self._fail(agent, ErrorID.MOTION_TIMEOUT)
            elif agent.state == "idle" and now - m.sent > max(MOTION_START_GRACE_S, m.expected + 0.5):
                log.error("xbot %d: %s to %s ended at (%.4f, %.4f)", agent.xid, m.kind, m.target, p.x, p.y)
                self._fail(agent, ErrorID.MOTION_DEVIATED)

    # --- tasks -------------------------------------------------------------------------------

    def _advance_tasks(self, now):
        """Advance unpaused tasks and manage idle xbots once their motions finish."""
        for agent in self.agents.values():
            task = agent.task
            if agent.paused_at is not None:
                continue
            if task is None:
                agent.want = None
                if (agent.present and not agent.fault and agent.motion is None and not agent.ops
                        and not self._pinned(agent)):
                    if agent.cell is None:
                        # Nobody holds it and it is off the lattice, a wall for everyone: step back on.
                        self._leave_dock(agent)
                    elif self._turned_wrong(agent) and now >= agent.turn_after and self._gathering is None:
                        self._turn_idle(agent)
                continue
            if agent.motion is not None or agent.ops or agent.fault:
                continue  # keeps what it wants until it has arrived
            if task.halting:
                self._finish(agent, False, ErrorID.INTERRUPTED)
                continue
            if self.patience_s and now - task.progress_at > self.patience_s:
                log.error("xbot %d: no way to its target for %.0f s", agent.xid, now - task.progress_at)
                self._finish(agent, False, ErrorID.BLOCKED)
                continue
            self._advance_task(agent, task, now)

    def _advance_task(self, agent, task, now):
        """Choose the next route, rotation or docking operation for a task."""
        layout = self.layout
        target = task.target
        p = agent.pose
        turn = angle_difference(p.rz, target.rz) >= ANGLE_TOLERANCE_RAD
        at_target_xy = abs(p.x - target.x) < POSITION_TOLERANCE_M and abs(p.y - target.y) < POSITION_TOLERANCE_M

        if self._queued(agent, task):
            # An older request for the same place goes first. This one waits on a slot: off one
            # it would be a wall, perhaps in the way of the very request it waits for.
            agent.want = agent.cell
            if agent.cell is None:
                self._leave_dock(agent)
            return
        if at_target_xy and not turn:
            self._finish(agent, True)
            return
        if not turn and not self.lines and self._straight(agent, task):  # (in line mode: the lanes)
            return

        if agent.cell is None:
            # Docked: turn here if this is a spot, else back onto the lattice.
            here = layout.spot_at(p.x, p.y)
            if here is not None and (at_target_xy or (turn and is_square_aligned(target.rz))):
                self._turn_here(agent, here, target.rz)
                return
            self._leave_dock(agent, self._toward(agent, task), target)
            return

        if turn:
            if task.spot is None:
                task.spot = self._choose_spot(agent, task)
                if task.spot is None:
                    agent.want = agent.cell
                    return
            cells = layout.spot_cells(task.spot)
            if agent.cell not in cells:
                agent.want = self._nearest(agent, cells)
                return
            boxes = (layout.spot_box(task.spot),)
            if not self._claim(agent, boxes):
                if not self._fixed_spots:
                    task.spot = None  # someone else has it now; look again next time
                agent.want = agent.cell
                return
            if not self._claim_clear(agent):
                self._wait_for_claim(agent, now)
                return
            sx, sy = task.spot
            ops = [("move", sx, sy), ("rotate", target.rz)]
            if not (abs(sx - target.x) < EXACT_M and abs(sy - target.y) < EXACT_M):
                # Toward the target, around the others; with every way blocked for now, toward it
                # anyway, not back into the xbot behind.
                targets = self._target_cells(task)
                exit_cell = min(sorted(cells), key=lambda c: (self._static_distance(agent, c, targets),
                                                             self._static_distance(agent, c, targets, False)))
                ops.append(("move", *layout.center(exit_cell)))
            self._run_ops(agent, ops, boxes)
            task.spot = None
            return

        target_cell = layout.cell_at(target.x, target.y, target.rz, EXACT_M)  # else the final move goes there
        if target_cell is not None:
            if agent.cell == target_cell:
                self._finish(agent, True)
            else:
                agent.want = target_cell
            return
        ways = self._ways_in(agent, task)
        if agent.cell not in ways:
            agent.want = self._nearest(agent, ways or self._target_cells(task))
            return
        ops, boxes = ways[agent.cell]
        if not self._claim(agent, boxes):
            agent.want = agent.cell
            return
        if not self._claim_clear(agent):
            self._wait_for_claim(agent, now)
            return
        self._run_ops(agent, ops, boxes)

    def _wait_for_claim(self, agent, now):
        """Yield a stalled claim after CLAIM_YIELD_S so competing requests can progress."""
        agent.want = agent.cell
        if now - agent.claim_since < CLAIM_YIELD_S:
            return
        if self.lines:
            # On lanes the ones in its way may have no way out while it stays put: let go, so the
            # planner may move it too, and claim again later.
            log.info("xbot %d: its claim did not clear in %.0f s; lets go of it for now", agent.xid, CLAIM_YIELD_S)
            agent.claim, agent.claim_boxes = None, ()
            if not self._fixed_spots:
                agent.task.spot = None
            agent.task.claim_after = now + CLAIM_YIELD_S
        elif any(o is not agent and o.claim_boxes and not o.in_macro and o.claim_since < agent.claim_since
                 for o in self.agents.values()):
            log.info("xbot %d: gives way to an older claim", agent.xid)
            agent.claim, agent.claim_boxes = None, ()
            agent.task.spot = None
            agent.task.claim_after = now + CLAIM_YIELD_S

    def _queued(self, agent, task):
        """Whether an older request still on its way wants (part of) the same place."""
        t = task.target
        box = self.layout.square(t.x, t.y, t.rz)
        for other in self.agents.values():
            if other is agent or other.task is None or other.task.seq > task.seq or other.task.internal:
                continue  # a turn to the idle orientation makes no request wait
            o = other.task.target
            if overlap(box, self.layout.square(o.x, o.y, o.rz)):
                return True
        return False

    def _straight(self, agent, task):
        """Claim and start a direct move if its swept area is usable and unoccupied."""
        p, t = agent.pose, task.target
        if not is_square_aligned(p.rz):
            return False
        box = hull(self.layout.square(p.x, p.y, p.rz), self.layout.square(t.x, t.y, t.rz))
        if not self.layout.footprint_of(box) <= self.layout.cells - self._closed_cells():
            return False
        if len(self.layout.cells_in(box)) > DIRECT_MAX_SLOTS:
            return False
        if not self._claim(agent, (box,), immediate=True):
            return False
        log.info("xbot %d: straight there", agent.xid)
        self._run_ops(agent, [("move", t.x, t.y)], (box,))
        return True

    def _ways_in(self, agent, task):
        """Return candidate docking cells with final moves and swept boxes for an off-grid target."""
        layout = self.layout
        t = task.target
        target_box = layout.square(t.x, t.y, t.rz)
        walls = self._walls(exclude=agent)
        blocked = self._statics(agent) + self._claims(agent)

        def clear(boxes):
            """Whether docking boxes avoid stationary xbots and existing claims."""
            return not any(overlap(b, s) for b in boxes for s in blocked)

        ways = {}
        for cell in sorted(self._target_cells(task)):
            if cell in walls and cell != agent.cell:
                continue
            boxes = (hull(layout.cell_box(cell), target_box),)
            if clear(boxes):
                ways.setdefault(cell, ([("move", t.x, t.y)], boxes))
        for dx, dy in ((0, 1), (0, -1), (1, 0), (-1, 0)):
            sx, sy = t.x + dx * layout.pitch, t.y + dy * layout.pitch
            stage = layout.square(sx, sy, t.rz)
            if not layout.footprint_of(stage) <= layout.cells:
                continue
            for cell in sorted(layout.cells_in(stage)):
                if cell in walls and cell != agent.cell:
                    continue
                boxes = (hull(layout.cell_box(cell), stage), hull(stage, target_box))
                if clear(boxes):
                    ways.setdefault(cell, ([("move", sx, sy), ("move", t.x, t.y)], boxes))
        return ways

    def _statics(self, agent):
        """The squares of the other xbots that stay where they are."""
        return [self.layout.square(o.pose.x, o.pose.y, o.pose.rz) for o in self.agents.values()
                if o is not agent and self._is_wall(o)]

    def _claims(self, agent):
        """Others' claims, and the zones closed: boxes nothing may sweep."""
        return [b for o in self.agents.values() if o is not agent for b in o.claim_boxes] + self._closed_boxes()

    def _turn_here(self, agent, spot, rz):
        """Turn on the spot it stands on, once its neighbours are clear of the turn."""
        boxes = (self.layout.spot_box(spot),)
        if self._claim(agent, boxes) and self._claim_clear(agent):
            log.info("xbot %d: turns to %.0f deg where it is, on (%.0f, %.0f) mm", agent.xid, math.degrees(rz),
                     spot[0] * 1000, spot[1] * 1000)
            self._run_ops(agent, [("rotate", rz)], boxes)

    def _leave_dock(self, agent, toward=None, target=None):
        """Return an off-grid xbot to a usable cell, straightening or backing out first if required."""
        layout = self.layout
        p = agent.pose
        if not is_square_aligned(p.rz):
            spot = layout.spot_at(p.x, p.y)
            if spot is None:
                log.error("xbot %d is turned %.1f deg off a spot and cannot travel",
                          agent.xid, math.degrees(p.rz))
                self._fail(agent, ErrorID.XBOT_CROOKED)
                return
            if target is not None and is_square_aligned(target.rz):
                rz = target.rz
            else:
                quarter = math.pi / 2
                rz = wrap_angle(round(p.rz / quarter) * quarter)
            self._turn_here(agent, spot, rz)
            return
        here = layout.square(p.x, p.y, p.rz)
        taken = [layout.square(o.pose.x, o.pose.y, o.pose.rz) for o in self.agents.values()
                 if o is not agent and o.pose is not None] + self._claims(agent)

        def clear(boxes):
            """Whether proposed undocking boxes avoid occupied and claimed space."""
            return not any(overlap(b, s) for b in boxes for s in taken)

        options = []
        for cell in sorted(agent.footprint):
            boxes = (hull(here, layout.cell_box(cell)),)
            if clear(boxes):
                options.append((cell, [("move", *layout.center(cell))], boxes))
        if not options:
            for dx, dy in ((0, 1), (0, -1), (1, 0), (-1, 0)):
                sx, sy = p.x + dx * layout.pitch, p.y + dy * layout.pitch
                stage = layout.square(sx, sy, p.rz)
                if not layout.footprint_of(stage) <= layout.cells:
                    continue
                for cell in sorted(layout.cells_in(stage)):
                    boxes = (hull(here, stage), hull(stage, layout.cell_box(cell)))
                    if clear(boxes):
                        options.append((cell, [("move", sx, sy), ("move", *layout.center(cell))], boxes))
        if not options:
            return  # boxed in on every side: wait
        if toward:
            cell, ops, boxes = min(options, key=lambda o: (self._static_distance(agent, o[0], toward), len(o[1])))
        else:
            cell, ops, boxes = min(options, key=lambda o: (len(o[1]), math.dist(layout.center(o[0]), (p.x, p.y))))
        if self._claim(agent, boxes, immediate=True):
            self._run_ops(agent, ops, boxes)

    def _target_cells(self, task):
        """Return existing cells covered by the requested target pose."""
        t = task.target
        return self.layout.footprint(t.x, t.y, t.rz) & self.layout.cells

    def _toward(self, agent, task):
        """Return the task's turn-spot cells, or its target cells if no turn is pending."""
        if task.spot is not None:
            return self.layout.spot_cells(task.spot)
        return self._target_cells(task)

    def _finish(self, agent, ok, error=ErrorID.NONE):
        """Complete a task with its final pose and update the xbot's home."""
        task, agent.task = agent.task, None
        if not agent.in_macro:
            agent.claim, agent.claim_boxes = None, ()  # a claim it was still waiting on
        agent.home = agent.cell
        p = agent.pose
        results = {"X": p.x * 1000.0, "Y": p.y * 1000.0, "Rz": math.degrees(p.rz)}
        log.info("xbot %d: %s%s", agent.xid, "arrived" if ok else "failed", "" if ok else f" ({ErrorID(error).name})")
        task.finished(ok, error, results)

    def _fail(self, agent, error):
        """Stop the xbot, record its fault and fail its pending task."""
        try:
            self.driver.stop(agent.xid)
        except DriverError:
            pass
        agent.fault = error
        self._drop(agent, error)

    def _drop(self, agent, error):
        """Discard an xbot's motion state and fail any pending task."""
        self._clear_motion(agent)
        if agent.task:
            self._finish(agent, False, error)

    def _clear_motion(self, agent):
        """Clear operations, actions and claims, release pause state and invalidate the plan."""
        self._resume_motion(agent)  # paused on the PMC: else the motions sent to it later would wait too
        agent.motion = None
        agent.ops.clear()
        agent.ops_done = None
        agent.actions.clear()
        agent.claim, agent.claim_boxes = None, ()
        agent.goal = None
        self._signature = None  # others may wait on moves it will not make now: plan again

    # --- macros: rotation, docking, undocking ------------------------------------------------
    # A macro sweeps boxes (claim_boxes). Nothing else may be in them; the planner keeps the
    # xbots it moves off the slots they overlap (claim). Xbots that stay where they are only have
    # to be clear of the boxes themselves: two xbots off the lattice may share a slot as long as
    # they do not overlap.

    def _run_ops(self, agent, ops, boxes):
        """Reserve swept boxes and queue a docking, rotation or compound operation sequence."""
        agent.claim_boxes = tuple(boxes)
        agent.claim = frozenset().union(*(self.layout.cells_in(b) for b in boxes))
        agent.ops = deque(ops)

        def done():
            """Release the macro claim and classify the xbot at its final pose."""
            agent.claim, agent.claim_boxes = None, ()
            self._classify(agent)
        agent.ops_done = done

    def _send_op(self, agent, now):
        """Dispatch the next macro operation and record its expected duration and trail."""
        op = agent.ops[0]
        here = (agent.pose.x, agent.pose.y)
        if op[0] == "move" and self._in_trail(agent, self.layout.ahead(here, op[1:]), now):
            return
        agent.ops.popleft()
        zone_speed = self._zone_speed(agent.claim or agent.footprint)
        try:
            if op[0] == "move":
                expected = self.driver.move(agent.xid, op[1], op[2], *self.speed_of(agent))
                agent.motion = Motion("move", (op[1], op[2]), now, expected / zone_speed,
                                      trail=self.layout.trail(here, op[1:]))
            else:
                expected = self.driver.rotate(agent.xid, op[1])
                agent.motion = Motion("rotate", (op[1],), now, expected / zone_speed)
        except DriverError as e:
            log.error("xbot %d: %s refused: %s", agent.xid, op, e)
            self._fail(agent, ErrorID.PMC_ERROR)

    def _claim(self, agent, boxes, immediate=False):
        """Reserve swept boxes if other xbots can clear them; immediate claims require them already clear
        and unplanned by others.
        """
        boxes = tuple(boxes)
        if agent.claim_boxes == boxes and not immediate:
            return True  # made already (an immediate claim is always checked afresh)
        if any(overlap(z, b) for z in self._closed_boxes() for b in boxes):
            return False
        cells = frozenset().union(*(self.layout.cells_in(b) for b in boxes))
        waiting = False  # another claim is waiting for its slots to clear
        for other in self.agents.values():
            if other is agent:
                continue
            if any(overlap(o, b) for o in other.claim_boxes for b in boxes):
                return False
            waiting = waiting or bool(other.claim_boxes and not other.in_macro)
            square = (self.layout.square(other.pose.x, other.pose.y, other.pose.rz)
                      if other.pose is not None else None)
            if square is not None and any(overlap(square, b) for b in boxes):
                if immediate or self._is_wall(other):
                    return False
            if immediate:
                if other.motion is not None and other.motion.kind == "nav":
                    if any(a.to in cells or a.frm in cells for a in other.motion.actions):
                        return False
                if any(a.to in cells for a in other.actions):
                    return False  # planned to come through: let the plan go first
        if (not immediate and (waiting or self.lines) and agent.task is not None
                and self.clock() < agent.task.claim_after):
            return False  # it gave way (to the older claim, or in line mode to the ones in its way)
        if not immediate and (waiting or self.lines) and not self._room_to_clear(agent, cells):
            return False
        agent.claim_boxes = boxes
        agent.claim = cells
        if not immediate:
            agent.claim_since = self.clock()
            log.info("xbot %d: claims %s", agent.xid, sorted(cells))
        return True

    def _room_to_clear(self, agent, cells):
        """Whether displaced xbots can escape the proposed claims into enough free cells."""
        claims = {o.xid: o.claim for o in self.agents.values() if o.claim and o is not agent}
        claims[agent.xid] = cells
        claimed = frozenset().union(*claims.values())
        open_cells = self.layout.cells - self._walls(exclude=agent)
        outside = open_cells - claimed
        movers = [m for m in self.agents.values() if self._is_mover(m)]
        if len(outside) < sum(1 for m in movers if m.start not in claims.get(m.xid, ())):
            return False
        if self.lines:  # the claimers stay where they are while they wait (see _plan)
            open_cells -= {o.start for o in movers if o.xid in claims and o.start is not None}
        for m in movers:
            start = m.start
            if start not in claimed or start in claims.get(m.xid, ()):
                continue
            # As the planner sees it: it may cross the claims it stands in, not the others.
            passable = open_cells - frozenset().union(*[z for z in claims.values() if start not in z])
            if self.lines:
                passable -= self.lines.keep_out([start, m.want], idle=m.task is None)
            if not outside & self._steps_from(start, passable).keys():
                return False
        return True

    def _claim_clear(self, agent):
        """Whether other xbots and committed motions have cleared a macro's claimed boxes."""
        boxes = agent.claim_boxes
        for other in self.agents.values():
            if other is agent:
                continue
            if other.pose is not None:
                square = self.layout.square(other.pose.x, other.pose.y, other.pose.rz)
                if any(overlap(square, b) for b in boxes):
                    return False
            if other.in_macro and any(overlap(o, b) for o in other.claim_boxes for b in boxes):
                return False
            if other.motion is not None and other.motion.kind == "nav":
                if any(a.to in agent.claim or a.frm in agent.claim for a in other.motion.actions):
                    return False
        return True

    # --- planning ----------------------------------------------------------------------------

    def _is_mover(self, agent):
        """Whether the xbot may participate in grid planning."""
        if not agent.present or agent.fault or agent.cell is None or agent.in_macro:
            return False
        if agent.cell in self._closed_cells():
            return False  # fenced in
        return agent.task is not None or not self._pinned(agent)

    def _pinned(self, agent):
        """Whether the module callback or explicit pin prevents displacement."""
        if self._gathering is not None and agent.xid in self._gathering.xids:
            return False  # being lined up and turned: the planner may move it
        if self.pinned_by is not None:
            return self.pinned_by(agent.xid)
        return agent.pinned

    def _is_wall(self, agent):
        """Whether a located xbot must be treated as a stationary obstacle."""
        # Landed or lost xbots too: they are still where they were last seen.
        return agent.pose is not None and not self._is_mover(agent)

    def _walls(self, exclude=None):
        """Return closed-zone cells and footprints or claims of xbots unavailable to planning."""
        walls = set(self._closed_cells())
        for other in self.agents.values():
            if other is not exclude and self._is_wall(other):
                walls |= other.footprint
                if other.claim:
                    walls |= other.claim
                if other.flying:  # pinned while on its way: every slot of the way, until it is there
                    walls |= {c for a in other.motion.actions for c in (a.frm, a.to)}
        return walls

    def _static_distance(self, agent, cell, toward, around_walls=True):
        """Steps from cell to the nearest of toward, around walls only (in line mode: on the lanes)."""
        toward = set(toward)
        if cell in toward:
            return 0
        free = self.layout.cells - (self._walls(exclude=agent) if around_walls else self._closed_cells())
        if self.lines:
            free -= self.lines.keep_out([cell, *toward], idle=agent.task is None)
        steps = self._steps_from(cell, free)
        return min((steps[c] for c in toward if c in steps), default=mapf.INF)

    def _nearest(self, agent, cells):
        """Choose the target cell with the shortest static route from the xbot."""
        return min(sorted(cells), key=lambda c: self._static_distance(agent, agent.cell, [c]))

    @property
    def _fixed_spots(self):
        """Line mode with turn spots: each xbot turns on the one nearest to it, and waits for it."""
        return bool(self.lines and self.lines.turn_spots)

    def _choose_spot(self, agent, task):
        """Choose a legal rotation spot with minimal detour; configured line-mode spots are selected by
        proximity even when occupied.
        """
        layout = self.layout
        target_cells = self._target_cells(task)
        if not is_square_aligned(task.target.rz):
            # It may not travel turned like that: it turns on the target's own spot.
            fixed, candidates = False, [layout.spot_at(task.target.x, task.target.y, EXACT_M)]
        elif self._fixed_spots:
            fixed, candidates = True, self.lines.turn_spots
        else:
            fixed, candidates = False, [layout.flyway_center(f) for f in layout.flyways]
        blocked = self._closed_boxes() if fixed else self._statics(agent) + self._claims(agent)
        best, best_cost = None, (mapf.INF, mapf.INF)
        for spot in candidates:
            if any(overlap(layout.spot_box(spot), b) for b in blocked):
                continue
            cells = layout.spot_cells(spot)
            there = self._static_distance(agent, agent.cell, cells, around_walls=not fixed)
            if there >= mapf.INF:
                continue
            on = min(self._static_distance(agent, c, target_cells, around_walls=not fixed) for c in cells)
            if on >= mapf.INF:
                continue
            cost = (there, there + on) if fixed else (there + on, 0)
            if cost < best_cost:
                best, best_cost = spot, cost
        if best is not None:
            log.info("xbot %d: turns on (%.0f, %.0f) mm", agent.xid, best[0] * 1000, best[1] * 1000)
        return best

    def _plan_if_needed(self, now):
        """Replan when inputs change or an incomplete plan is due for retry."""
        movers = [a for a in self.agents.values() if self._is_mover(a)]
        signature = self._signature_of(movers)
        finished = all(not a.actions and a.motion is None for a in movers)
        stale = signature != self._signature
        unfinished = finished and any(a.cell != (a.desired or a.cell) for a in movers)
        if not stale and not (unfinished and now >= self._retry_at):
            return
        self._plan(movers, now)
        self._signature = signature

    def _signature_of(self, movers):
        """Summarize walls, claims and mover goals to detect stale plans."""
        walls = frozenset(self._walls())
        claims = frozenset((a.xid, a.claim) for a in self.agents.values() if a.claim)
        roles = frozenset((a.xid, a.task is not None, a.want) for a in movers)
        return walls, claims, roles

    def _plan(self, movers, now):
        """Plan from committed endpoints, preserving dependencies on cells still being vacated."""
        for agent in self.agents.values():
            agent.actions.clear()
            agent.goal = None
            agent.desired = None
        # The motions in flight go on: the plan starts each xbot on its way from the slot it is
        # going to, and the slots it has still to leave on the way are entered only after it.
        leaving, flying = {}, set()
        for agent in self.agents.values():
            if agent.flying:
                for a in agent.motion.actions:
                    flying.add(a.id)
                    if a.id not in self._done:
                        leaving[a.frm] = a.id
        self._done &= flying
        if not movers:
            return
        start = {a.xid: a.start for a in movers}
        walls = self._walls()
        # Its own slot is always one it may start from, even inside another's claim: it may
        # leave a claim but not come back into it (keep_out).
        cells = (self.layout.cells - walls) | set(start.values())
        claims = {a.xid: a.claim for a in self.agents.values() if a.claim}
        # Others' claimed slots: never a goal; not to be entered, except by one already inside,
        # which may cross them on its way out.
        claimed = {a.xid: frozenset().union(*[z for x, z in claims.items() if x != a.xid]) for a in movers}
        keep_out = {a.xid: frozenset().union(*[z for x, z in claims.items() if x != a.xid and start[a.xid] not in z])
                    for a in movers}
        passable = cells
        if self.lines:
            # Line mode: the lanes, and off them only the way from where it is and to where it goes
            # (one nobody holds: and the idle area).
            for a in movers:
                own = [start[a.xid], a.want] if a.task else [start[a.xid], None if self.queue else a.home]
                keep_out[a.xid] |= self.lines.keep_out(own, idle=a.task is None)
                if a.claim:
                    # Waiting for its claim to clear, it stays put: pushed off along a one-way
                    # lane it would have to go all the way round, its claim blocking the lane.
                    keep_out[a.xid] = self.layout.cells - {start[a.xid]}
            passable = cells - self.lines.keep_out(list(start.values()), idle=True)

        goals = {}
        taken = set()

        def settle(agent, want):
            """Assign a distinct reachable goal, substituting a nearby free cell when necessary."""
            agent.desired = want
            here, free = start[agent.xid], cells - keep_out[agent.xid]
            usable = cells - claimed[agent.xid] - taken
            if (want is None or want not in usable or (want not in free and want != here)
                    or not self._reachable(here, want, free)):
                if agent.task is not None and want is not None:
                    log.debug("xbot %d: waits, %s is not free", agent.xid, want)
                want = self._closest_free(here, want or here, usable, free)
            if want in taken:
                # Boxed in, it can only stay where it is: the one given its slot goes elsewhere.
                other = self.agents[next(x for x, g in goals.items() if g == want)]
                goals[agent.xid] = want
                del goals[other.xid]
                settle(other, other.desired)
                return
            goals[agent.xid] = want
            taken.add(want)

        # In line mode, one standing in another's claim first goes to the nearest slot out of it it
        # can get to past the others (on a lane one wide the way to its own target may need others
        # to pass each other, which they cannot while the claim blocks the lane).
        leaving_claims = set()
        if self.lines:
            for agent in movers:
                way_out = self._way_out(agent, start, cells - keep_out[agent.xid], claimed[agent.xid] | taken)
                if way_out is not None:
                    settle(agent, way_out)
                    leaving_claims.add(agent.xid)
        # Xbots with a task first, the longest waiting first; then the idle ones: into the loading
        # queue if there is one, else back where they were.
        rest = [a for a in movers if a.xid not in leaving_claims]
        idle = [a for a in rest if a.task is None]
        for agent in sorted((a for a in rest if a.task), key=lambda a: a.task.seq):
            settle(agent, agent.want)
        if self.queue:
            unclaimed = cells - taken - frozenset().union(*claims.values())
            for agent, slot in self._line_up(idle, start, unclaimed, passable):
                settle(agent, slot)
        else:
            for agent in idle:
                settle(agent, agent.home)
                if agent.home is None:
                    agent.home = goals[agent.xid]

        order = [a.xid for a in movers]
        try:
            instance = mapf.Instance(cells, self.graph,
                                     [start[x] for x in order], [goals[x] for x in order],
                                     keep_out=[keep_out[x] for x in order])
        except ValueError as e:
            log.error("cannot plan %s: %s", order, e)
            self._retry_at = now + 1.0
            return
        motions = [mapf.Motion(self.layout.pitch, *self.speed_of(self.agents[x])) for x in order]
        result = mapf.plan(instance, timeout=self.plan_timeout, budget=self.plan_budget, motion=motions)
        moved = len(result.configs) > 1
        substituted = any(goals[a.xid] != a.desired for a in movers)
        log.log(logging.INFO if moved else logging.DEBUG, "plan for %s: %d steps%s, %d expanded, %.0f ms",
                order, len(result.configs) - 1, "" if result.complete else " (incomplete)",
                result.expanded, result.seconds * 1000)
        if not result.complete or substituted:
            self._retry_at = now + 1.0  # some cannot get where they want yet: look again later
        if not moved:
            return
        self._load(order, result.configs, goals, leaving)

    def _line_up(self, idle, start, usable, passable):
        """Assign idle xbots to queue slots, favouring nearby movers that leave room for others."""
        left = {a.xid: a for a in idle}
        needed = set([s for s in self.queue if s in usable][:len(left)])
        out = []
        for slot in self.queue:
            if not left:
                break
            if slot not in usable:
                continue
            steps = self._steps_from(slot, passable, reverse=True)  # from the xbots to the slot
            reach = [x for x in left if start[x] in steps]
            if reach:
                xid = min(reach, key=lambda x: (steps[start[x]], start[x] in needed, x))
                out.append((left.pop(xid), slot))
        return out + [(a, start[a.xid]) for a in left.values()]

    def _way_out(self, agent, start, free, claimed):
        """Find the nearest reachable cell outside others' claims, or None if none is needed or reachable."""
        here = start[agent.xid]
        if here not in claimed:
            return None
        others = {s for x, s in start.items() if x != agent.xid}
        return next((u for u in self._steps_from(here, free - others) if u not in claimed), None)

    def _steps_from(self, origin, passable, reverse=False):
        """Steps from origin to every slot it can reach across passable ones (reverse: to origin)."""
        return steps_from(self.graph_reverse if reverse else self.graph, origin, passable)

    def _reachable(self, start, goal, free):
        """Whether goal can be reached from start across free slots (goal itself need not be free)."""
        return goal == start or goal in self._steps_from(start, free | {goal})

    def _closest_free(self, start, near, usable, free):
        """The usable cell reachable from start that is closest to near (start itself if it is usable)."""
        candidates = [c for c in self._steps_from(start, free) if c in usable]
        if not candidates:
            return start
        if self.lines:
            # On the lanes: the one fewest steps short of it on the way there, so it queues behind
            # a taken place (a turn spot another turns on) rather than beside or past it.
            short = self._steps_from(near, free | {near}, reverse=True)
            return min(sorted(candidates), key=lambda c: (short.get(c, mapf.INF), abs(c[0] - near[0]) + abs(c[1] - near[1]),
                                                          c != start))
        return min(sorted(candidates), key=lambda c: (abs(c[0] - near[0]) + abs(c[1] - near[1]), c != start))

    def _load(self, order, configs, goals, leaving):
        """Convert configurations into per-xbot actions, including dependencies on in-flight moves."""
        per_xbot = mapf.actions(configs, first_id=self._next_action + 1, leaving=leaving)
        for k, xid in enumerate(order):
            agent = self.agents[xid]
            for aid, frm, to, dep in per_xbot[k]:
                agent.actions.append(Action(aid, xid, frm, to, frozenset() if dep is None else frozenset([dep])))
                self._next_action = max(self._next_action, aid)
            agent.goal = goals[xid]

    # --- sending motions ---------------------------------------------------------------------

    def _in_trail(self, agent, box, now):
        """Whether a proposed move overlaps another motion's protected trail before it is safe to follow."""
        for other in self.agents.values():
            m = other.motion
            if other is agent or m is None or m.trail is None or not overlap(box, m.trail):
                continue
            if other.paused_at is not None or m.expected - (now - m.sent) > TRAIL_WAIT_S:
                return True
        return False

    def _dispatch(self, now):
        """Send ready macros or merge dependency-cleared grid steps into straight runs."""
        for agent in self.agents.values():
            if agent.motion is not None or agent.fault or agent.paused_at is not None:
                continue
            if agent.ops:
                if agent.claim is None or self._claim_clear(agent):
                    self._send_op(agent, now)
                continue
            if not agent.actions:
                continue
            first = agent.actions[0]
            if not first.deps <= self._done:
                continue
            run = [first]
            step = (first.to[0] - first.frm[0], first.to[1] - first.frm[1])
            for action in list(agent.actions)[1:]:
                if action.frm != run[-1].to or (action.to[0] - action.frm[0], action.to[1] - action.frm[1]) != step:
                    break
                if not action.deps <= self._done:
                    break
                run.append(action)
            x, y = self.layout.center(run[-1].to)
            origin = self.layout.center(run[0].frm)
            if self._in_trail(agent, self.layout.ahead(origin, (x, y)), now):
                continue
            try:
                expected = self.driver.move(agent.xid, x, y, *self.speed_of(agent))
            except DriverError as e:
                log.error("xbot %d: move refused: %s", agent.xid, e)
                self._fail(agent, ErrorID.PMC_ERROR)
                continue
            for _ in run:
                agent.actions.popleft()
            expected /= self._zone_speed({a.frm for a in run} | {a.to for a in run})
            agent.motion = Motion("nav", (x, y), now, expected, run, origin, step, self.layout.trail(origin, (x, y)))
