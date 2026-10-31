"""
The traffic coordinator: every xbot on the table moves as part of one plan.

Each xbot is in one of these situations:

- on the lattice: parked on a slot centre with its sides along the axes. It covers that slot
  and the planner may move it.
- docked: anywhere else (at a station pose between slots, at a flyway centre, stopped half way
  by an abort). It covers every slot its square overlaps, and the planner treats those as walls.
- busy with a macro: a rotation (only possible at a flyway centre) or the last move onto or off
  a pose between slots. It first claims the slots it will sweep, waits until the others have
  left them, and covers them until it is done.

The planner (LaCAM with PIBT, see mapf.py) moves the xbots that may move: those with a task,
and idle ones that nobody holds, which it may push out of the way and brings back to where they
were. An xbot that is held (its shuttle is occupied and not moving) stays where it is: a station
may be working on it.

Plans are executed by dependencies, not by time: an xbot starts into a slot only once every
xbot planned to be there before it has fully left. Consecutive steps in one direction whose
dependencies are met are sent as one motion. A new plan is made at once whenever the set of
xbots, their goals or the walls change: an xbot on its way is planned from the slot its motion
ends on, and nobody enters a slot it has still to leave before it has left it, so the others
need not stop for the new plan.

With a loading station, the xbots nobody holds line up in a queue on the slots nearest to it
(along the table's edges first), the first one on the station; the table's Resetting sends them
there with the PMC's own auto routing (gather). With an idle orientation, an xbot nobody holds
is turned to it (on a flyway centre: the PMC turns nowhere else, and its auto routing does not
turn at all), by a task of the coordinator's own that any request replaces.

In line mode (lines.py) the planner plans on the lanes' graph instead of the whole lattice, and
keeps every xbot off the slots of no lane but the way to and from its own target (and the idle
area, the loading queue, for the ones nobody holds); turns are made on the turn spots, and there
is no straight move off the lanes.

Units: metres and radians.
"""

import logging
import math
import threading
import time
from collections import deque
from dataclasses import dataclass, field

from . import mapf
from .layout import (ANGLE_TOLERANCE_RAD, EXACT_M, POSITION_TOLERANCE_M, angle_difference, hull,
                     is_square_aligned, overlap, wrap_angle)
from .lines import LineError, Lines
from .statemachines import ErrorID

log = logging.getLogger(__name__)

PROGRESS_TOLERANCE_M = 0.0005
# A straight move from where it stands to its target is taken when nothing is in the way and it
# sweeps no more slots than this; anything longer goes through the planner.
DIRECT_MAX_SLOTS = 16
MOTION_START_GRACE_S = 0.5
# A claim that has waited this long for its zone to clear gives way to an older one still waiting
# (and claims again after as long), so claims that need each other's room cannot wait for ever.
CLAIM_YIELD_S = 5.0
# The PMC's auto routing takes at most this many xbots in one command.
AUTO_DRIVE_MAX = 10
GATHER_TIMEOUT_S = 120.0


@dataclass
class Pose:
    x: float
    y: float
    rz: float = 0.0


@dataclass
class Reading:
    """One xbot as the driver reads it. state: idle, moving, fault or off."""
    x: float
    y: float
    rz: float
    state: str
    detail: str = ""


class DriverError(Exception):
    def __init__(self, message, code=ErrorID.PMC_ERROR):
        super().__init__(message)
        self.code = ErrorID(code)


class Driver:
    """What the coordinator needs from the PMC. Called from the coordinator's thread only,
    except stop(), which an abort may call from anywhere."""

    def connect(self):
        """Connect, take mastership, levitate; return the Layout."""
        raise NotImplementedError

    def read(self) -> dict:
        raise NotImplementedError

    def move(self, xid, x, y) -> float:
        raise NotImplementedError

    def rotate(self, xid, rz) -> float:
        raise NotImplementedError

    def stop(self, xid=0):
        raise NotImplementedError

    def auto_drive(self, xids, targets):
        """Route these xbots with the PMC's own auto routing, each to one of the targets ((x, y),
        any to any); the xbots read as moving until they are there."""
        raise NotImplementedError


@dataclass
class Action:
    id: int
    xid: int
    frm: tuple
    to: tuple
    deps: frozenset


@dataclass
class Motion:
    kind: str                      # nav, move or rotate
    target: tuple                  # (x, y) or (rz,)
    sent: float
    expected: float
    actions: list = field(default_factory=list)
    origin: tuple = None
    direction: tuple = None


@dataclass
class Task:
    target: Pose
    finished: object               # finished(ok, error, results)
    started: float
    seq: int = 0                   # the order requests were made in (the clock may not tell)
    halting: bool = False
    flyway: tuple = None           # where it rotates
    progress_at: float = 0.0
    claim_after: float = 0.0       # gave way with its claim: not to claim again before this
    internal: bool = False         # the coordinator's own (turning an idle xbot): a request replaces it


@dataclass
class Gathering:
    xids: list
    targets: list                  # slots
    sent: float
    done: object                   # done(ok, error)
    routing: bool = True           # the PMC's auto routing; then the turns to the idle orientation


class Agent:
    def __init__(self, xid):
        self.xid = xid
        self.pose = None
        self.state = "off"
        self.detail = ""
        self.cell = None
        self.footprint = frozenset()
        self.home = None
        self.held = False
        self.fault = None
        self.task = None
        self.motion = None
        self.actions = deque()
        self.goal = None           # what the current plan takes it to
        self.want = None           # what it should be planned to now
        self.zone = None           # slots a macro sweeps: others are planned off them
        self.zone_boxes = ()       # the boxes it sweeps: nothing else may be in them
        self.zone_since = 0.0      # when it claimed them
        self.desired = None        # where the last plan wanted it (its goal may stand in for it)
        self.turn_after = 0.0      # a turn to the idle orientation failed: not again before this
        self.ops = deque()
        self.ops_done = None

    @property
    def present(self):
        return self.pose is not None and self.state != "off"

    @property
    def in_macro(self):
        return bool(self.ops) or (self.motion is not None and self.motion.kind != "nav")

    @property
    def flying(self):
        return self.motion is not None and self.motion.kind == "nav"

    @property
    def start(self):
        """The slot a new plan starts it from: where its motion in flight ends, else where it is."""
        return self.motion.actions[-1].to if self.flying else self.cell

    def describe(self):
        where = self.cell if self.cell is not None else f"docked {sorted(self.footprint)}"
        return f"xbot {self.xid} at {where}"


class Coordinator:
    def __init__(self, driver, lock, clock=time.monotonic, *, plan_timeout=0.5, plan_budget=0.05,
                 patience_s=None, displace_held=False, loading_station=None, idle_rz=None, line_spec=None):
        self.driver = driver
        self.line_spec = line_spec  # line mode: the lanes they travel on (see lines.py); None: free
        self.lines = None          # the line spec on the table, once connected
        self.graph = None          # the steps the planner may take: {slot: [slots]}
        self.graph_reverse = None
        self.lock = lock
        self.clock = clock
        self.plan_timeout = plan_timeout
        self.plan_budget = plan_budget  # how long to look for a faster plan once one is found
        self.patience_s = patience_s
        self.displace_held = displace_held
        self.loading_station = loading_station  # (x, y): idle xbots queue up there
        self.queue = []            # the queue's slots, the one on the station first
        self.idle_rz = None        # idle xbots are turned to this (radians, a multiple of 90 deg)
        if idle_rz is not None and self.set_idle_rz(idle_rz) != ErrorID.NONE:
            raise ValueError(f"the idle orientation must be a multiple of 90 degrees, not {math.degrees(idle_rz):g}")
        self._gathering = None
        # held_by(xid) -> bool, asked whenever it matters (the modules know); else hold() decides.
        self.held_by = None
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
        self._running = True
        self._thread = threading.Thread(target=self._loop, args=(period_s,), name="coordinator", daemon=True)
        self._thread.start()

    def shutdown(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)

    def _loop(self, period_s):
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
        while self._jobs:
            job = self._jobs.popleft()
            try:
                job()
            except Exception:
                log.exception("coordinator job")
        with self.lock:
            for hook in self.on_tick:  # the modules: their state decides who may be pushed
                hook()
            if self.connected:
                try:
                    readings = self.driver.read()
                except DriverError as e:
                    self._lost(e)
                    readings = None
                if readings is not None:
                    self._update(readings, now)
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

    def connect(self, xids=None):
        """Connect to the PMC (blocking; from a job or before start()). Returns an ErrorID."""
        try:
            layout = self.driver.connect()
            readings = self.driver.read()
        except DriverError as e:
            log.error("connecting to the PMC: %s", e)
            return e.code
        lines = None
        if self.line_spec is not None:
            try:
                lines = Lines(self.line_spec, layout)
            except LineError as e:
                log.error("the line layout does not fit the table: %s", e)
                return ErrorID.LINE_LAYOUT_INVALID
        with self.lock:
            self.layout = layout
            self.lines = lines
            self.graph = lines.graph if lines else layout.adjacency
            self.graph_reverse = lines.reverse if lines else layout.adjacency
            # Every xbot on the table takes part, whether it is offered as a shuttle or not.
            for xid in set(readings) | set(xids or ()):
                self.agents.setdefault(xid, Agent(xid))
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
                log.info("line mode, %d lanes, turn spots %s:\n%s", len(lines.lanes),
                         lines.turn_spots or "any flyway", lines.describe())
                if len(lines.idle) < len(self.agents):
                    log.warning("line mode: the idle area has %d slots for %d xbots; the free xbots that do not "
                                "fit wait where they are, on a lane in the way", len(lines.idle), len(self.agents))
            else:
                log.info("table slots:\n%s", layout.describe())
            if self.queue:
                log.info("loading queue: %s", self.queue[:len(self.agents)])
        return ErrorID.NONE

    def _lost(self, error):
        log.error("PMC connection lost: %s", error)
        self.connected = False
        self._gathering = None
        for agent in self.agents.values():
            self._drop(agent, ErrorID.PMC_NOT_CONNECTED)
        if self.on_connection_lost:
            self.on_connection_lost()

    # --- commands (from the modules, with the lock held) -------------------------------------

    def check_target(self, xid, target) -> ErrorID:
        error = self.check_xbot(xid, clear=False)
        if error != ErrorID.NONE:
            return error
        agent = self.agents[xid]
        if agent.task is not None and not agent.task.internal:
            return ErrorID.BUSY
        if not is_square_aligned(target.rz) and self.layout.flyway_at(target.x, target.y, EXACT_M) is None:
            # It can only turn to an angle like that at a flyway centre, and only stay there.
            return ErrorID.ANGLE_NOT_ALLOWED
        if not self.layout.on_table(target.x, target.y, target.rz):
            return ErrorID.NOT_ON_TABLE
        if not is_square_aligned(agent.pose.rz) and agent.cell is None \
                and self.layout.flyway_at(agent.pose.x, agent.pose.y) is None:
            return ErrorID.XBOT_CROOKED
        return ErrorID.NONE

    def submit(self, xid, target, finished) -> ErrorID:
        error = self.check_target(xid, target)
        if error != ErrorID.NONE:
            return error
        agent = self.agents[xid]
        if agent.task is not None:  # its turn to the idle orientation: the request goes first
            agent.task = None
            if not agent.in_macro:
                agent.zone, agent.zone_boxes = None, ()
        self._new_task(agent, Pose(target.x, target.y, wrap_angle(target.rz)), finished)
        log.info("xbot %d: to (%.3f, %.3f, %.1f deg)", xid, target.x, target.y, math.degrees(target.rz))
        return ErrorID.NONE

    def _new_task(self, agent, target, finished, internal=False):
        now = self.clock()
        self._requests += 1
        agent.task = Task(target, finished, now, seq=self._requests, progress_at=now, internal=internal)

    # --- the idle orientation ----------------------------------------------------------------

    def set_idle_rz(self, rz):
        """The orientation idle xbots are turned to (radians); None (or NaN): as they are. Only a
        multiple of 90 degrees, the angles an xbot can travel at."""
        if rz is None or math.isnan(rz):
            self.idle_rz = None
            return ErrorID.NONE
        if not is_square_aligned(rz):
            return ErrorID.OUT_OF_RANGE
        quarter = math.pi / 2
        self.idle_rz = wrap_angle(round(rz / quarter) * quarter)
        return ErrorID.NONE

    def _turned_wrong(self, agent):
        return (self.idle_rz is not None and agent.pose is not None
                and angle_difference(agent.pose.rz, self.idle_rz) >= ANGLE_TOLERANCE_RAD)

    def _turn_idle(self, agent):
        """Turn it to the idle orientation, back onto the slot it is on."""
        def finished(ok, error, results):
            if not ok:
                agent.turn_after = self.clock() + CLAIM_YIELD_S * 2
        log.info("xbot %d: turns to the idle orientation %.0f deg", agent.xid, math.degrees(self.idle_rz))
        self._new_task(agent, Pose(*self.layout.center(agent.cell), self.idle_rz), finished, internal=True)

    def halt(self, xid):
        agent = self.agents.get(xid)
        if agent and agent.task:
            agent.task.halting = True

    def abort(self, xid):
        """Stop at once, wherever it is; the task is dropped without a word."""
        agent = self.agents.get(xid)
        if agent is None:
            return
        try:
            self.driver.stop(xid)
        except DriverError as e:
            log.error("xbot %d: stop: %s", xid, e)
        agent.task = None
        self._clear_motion(agent)

    def abort_all(self):
        self._gathering = None
        try:
            self.driver.stop(0)
        except DriverError as e:
            log.error("stop all: %s", e)
        for agent in self.agents.values():
            agent.task = None
            self._clear_motion(agent)

    def hold(self, xid, held):
        agent = self.agents.get(xid)
        if agent is not None:
            agent.held = held

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
        agent = self.agents.get(xid)
        if agent is None or agent.pose is None:
            return None
        return agent.pose, agent.state, agent.detail

    # --- the loading queue -------------------------------------------------------------------

    def _queue_slots(self):
        """
        Every slot in the order the queue fills them: the one under the loading station (or the
        nearest to it), then by steps from there, a slot inside the table counting two steps more
        than one on its edge (or a hole's), so the queue lines up along the edge rather than
        spreading into the table where the others pass.
        """
        layout = self.layout
        sx, sy = self.loading_station
        head = layout.cell_at(sx, sy, tolerance=layout.pitch)
        if head is None:
            head = min(sorted(layout.cells), key=lambda c: math.dist(layout.center(c), (sx, sy)))
            log.warning("the loading station (%.3f, %.3f) is not over a slot; the queue starts at %s", sx, sy, head)
        steps = {head: 0}
        queue = deque([head])
        while queue:
            v = queue.popleft()
            for u in layout.adjacency[v]:
                if u not in steps:
                    steps[u] = steps[v] + 1
                    queue.append(u)
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
        """
        Send these xbots into the loading queue with the PMC's auto routing, then turn them to
        the idle orientation, then done(ok, error). With more xbots than one command takes or
        when the PMC will not route them, the planner lines them up instead (it may move them
        until done, whoever holds them); without a loading station they are only turned.
        """
        agents = [self.agents[x] for x in xids if x in self.agents and self.agents[x].present]
        for agent in agents:
            agent.task = None
            self._clear_motion(agent)
        others = frozenset().union(*[a.footprint for a in self.agents.values() if a.present and a not in agents])
        targets = [s for s in self.queue if s not in others][:len(agents)]
        self._gathering = Gathering([a.xid for a in agents], targets, self.clock(), done, routing=False)
        if not targets or {a.cell for a in agents} == set(targets):
            return
        if len(agents) > AUTO_DRIVE_MAX:
            log.info("%d xbots: more than one auto routing command takes; the planner lines them up", len(agents))
            return
        try:
            self.driver.auto_drive([a.xid for a in agents], [self.layout.center(c) for c in targets])
        except (NotImplementedError, DriverError) as e:
            log.warning("auto routing into the loading queue: %s; the planner lines them up", e or "not supported")
            return
        log.info("auto routing xbots %s into the loading queue %s", [a.xid for a in agents], targets)
        self._gathering.routing = True

    def cancel_gather(self):
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
            if agent.motion is None and not agent.ops:
                self._classify(agent)

    def _classify(self, agent):
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
        for agent in self.agents.values():
            m = agent.motion
            if m is None or agent.pose is None:
                continue
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
        for agent in self.agents.values():
            task = agent.task
            if task is None:
                agent.want = None
                if (agent.present and not agent.fault and agent.motion is None and not agent.ops
                        and not self._held(agent)):
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
            # Docked: turn here if this is a flyway centre, else back onto the lattice.
            here = layout.flyway_at(p.x, p.y)
            if here is not None and (at_target_xy or (turn and is_square_aligned(target.rz))):
                self._turn_here(agent, here, target.rz)
                return
            self._leave_dock(agent, self._toward(agent, task), target)
            return

        if turn:
            if task.flyway is None:
                task.flyway = self._choose_flyway(agent, task)
                if task.flyway is None:
                    agent.want = agent.cell
                    return
            cells = layout.flyway_cells(task.flyway)
            if agent.cell not in cells:
                agent.want = self._nearest(agent, cells)
                return
            boxes = (layout.flyway_box(task.flyway),)
            if not self._claim(agent, boxes):
                task.flyway = None  # someone else has it now; look again next time
                agent.want = agent.cell
                return
            if not self._zone_clear(agent):
                self._wait_for_zone(agent, now)
                return
            fx, fy = layout.flyway_center(task.flyway)
            ops = [("move", fx, fy), ("rotate", target.rz)]
            if not (abs(fx - target.x) < EXACT_M and abs(fy - target.y) < EXACT_M):
                exit_cell = min(sorted(cells), key=lambda c: self._static_distance(agent, c, self._target_cells(task)))
                ops.append(("move", *layout.center(exit_cell)))
            self._run_ops(agent, ops, boxes)
            task.flyway = None
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
        if not self._zone_clear(agent):
            self._wait_for_zone(agent, now)
            return
        self._run_ops(agent, ops, boxes)

    def _wait_for_zone(self, agent, now):
        """It waits on its slot for its claim to clear; after CLAIM_YIELD_S it gives way to an
        older claim still waiting too (they may need each other's room to clear)."""
        agent.want = agent.cell
        if now - agent.zone_since < CLAIM_YIELD_S:
            return
        if self.lines:
            # On lanes the ones in its way may have no way out while it stays put: let go, so the
            # planner may move it too, and claim again later.
            log.info("xbot %d: its claim did not clear in %.0f s; lets go of it for now", agent.xid, CLAIM_YIELD_S)
            agent.zone, agent.zone_boxes = None, ()
            agent.task.flyway = None
            agent.task.claim_after = now + CLAIM_YIELD_S
        elif any(o is not agent and o.zone_boxes and not o.in_macro and o.zone_since < agent.zone_since
                 for o in self.agents.values()):
            log.info("xbot %d: gives way to an older claim", agent.xid)
            agent.zone, agent.zone_boxes = None, ()
            agent.task.flyway = None
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
        """
        One straight move from where it stands to its target, when the way is free: the shortest
        and most obvious route. Free means the box the move sweeps is on the table, no other
        xbot is in it or on its way into it, and no other claim overlaps it.
        """
        p, t = agent.pose, task.target
        if not is_square_aligned(p.rz):
            return False
        box = hull(self.layout.square(p.x, p.y, p.rz), self.layout.square(t.x, t.y, t.rz))
        if not self.layout.footprint_of(box) <= self.layout.cells:
            return False
        if len(self.layout.cells_in(box)) > DIRECT_MAX_SLOTS:
            return False
        if not self._claim(agent, (box,), immediate=True):
            return False
        log.info("xbot %d: straight there", agent.xid)
        self._run_ops(agent, [("move", t.x, t.y)], (box,))
        return True

    def _ways_in(self, agent, task):
        """
        The slots it can make its last moves to an off-lattice target from, each with those moves
        and the boxes they sweep, which xbots that stay where they are must be clear of:

        - from a slot the target covers, straight there;
        - from a slot under a pose one xbot beside the target, to that pose and straight in: the
          way into a place between two xbots it touches on both sides.
        """
        layout = self.layout
        t = task.target
        target_box = layout.square(t.x, t.y, t.rz)
        walls = self._walls(exclude=agent)
        blocked = self._statics(agent) + self._claims(agent)

        def clear(boxes):
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
        return [b for o in self.agents.values() if o is not agent for b in o.zone_boxes]

    def _turn_here(self, agent, flyway, rz):
        """Turn on the flyway centre it stands on, once its neighbours are clear of the turn."""
        boxes = (self.layout.flyway_box(flyway),)
        if self._claim(agent, boxes) and self._zone_clear(agent):
            self._run_ops(agent, [("rotate", rz)], boxes)

    def _leave_dock(self, agent, toward=None, target=None):
        """
        From a pose off the lattice onto the slot it covers nearest to where it goes next. It
        only travels square to the axes, so a crooked xbot straightens up first (which it can
        only do on a flyway centre). A slot another xbot reaches into is not one to step onto;
        when every slot it covers is one, it backs out sideways first (one xbot width) onto a
        pose with a free slot under it.
        """
        layout = self.layout
        p = agent.pose
        if not is_square_aligned(p.rz):
            flyway = layout.flyway_at(p.x, p.y)
            if flyway is None:
                log.error("xbot %d is turned %.1f deg off a flyway centre and cannot travel",
                          agent.xid, math.degrees(p.rz))
                self._fail(agent, ErrorID.XBOT_CROOKED)
                return
            if target is not None and is_square_aligned(target.rz):
                rz = target.rz
            else:
                quarter = math.pi / 2
                rz = wrap_angle(round(p.rz / quarter) * quarter)
            self._turn_here(agent, flyway, rz)
            return
        here = layout.square(p.x, p.y, p.rz)
        taken = [layout.square(o.pose.x, o.pose.y, o.pose.rz) for o in self.agents.values()
                 if o is not agent and o.pose is not None] + self._claims(agent)

        def clear(boxes):
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
        t = task.target
        return self.layout.footprint(t.x, t.y, t.rz) & self.layout.cells

    def _toward(self, agent, task):
        if task.flyway is not None:
            return self.layout.flyway_cells(task.flyway)
        return self._target_cells(task)

    def _finish(self, agent, ok, error=ErrorID.NONE):
        task, agent.task = agent.task, None
        if not agent.in_macro:
            agent.zone, agent.zone_boxes = None, ()  # a claim it was still waiting on
        agent.home = agent.cell
        p = agent.pose
        results = {"X": p.x * 1000.0, "Y": p.y * 1000.0, "Rz": math.degrees(p.rz)}
        log.info("xbot %d: %s%s", agent.xid, "arrived" if ok else "failed", "" if ok else f" ({ErrorID(error).name})")
        task.finished(ok, error, results)

    def _fail(self, agent, error):
        try:
            self.driver.stop(agent.xid)
        except DriverError:
            pass
        agent.fault = error
        self._drop(agent, error)

    def _drop(self, agent, error):
        self._clear_motion(agent)
        if agent.task:
            self._finish(agent, False, error)

    def _clear_motion(self, agent):
        agent.motion = None
        agent.ops.clear()
        agent.ops_done = None
        agent.actions.clear()
        agent.zone, agent.zone_boxes = None, ()
        agent.goal = None
        self._signature = None  # others may wait on moves it will not make now: plan again

    # --- macros: rotation, docking, undocking ------------------------------------------------
    # A macro sweeps boxes (zone_boxes). Nothing else may be in them; the planner keeps the
    # xbots it moves off the slots they overlap (zone). Xbots that stay where they are only have
    # to be clear of the boxes themselves: two xbots off the lattice may share a slot as long as
    # they do not overlap.

    def _run_ops(self, agent, ops, boxes):
        agent.zone_boxes = tuple(boxes)
        agent.zone = frozenset().union(*(self.layout.cells_in(b) for b in boxes))
        agent.ops = deque(ops)

        def done():
            agent.zone, agent.zone_boxes = None, ()
            self._classify(agent)
        agent.ops_done = done

    def _send_op(self, agent, now):
        op = agent.ops.popleft()
        try:
            if op[0] == "move":
                expected = self.driver.move(agent.xid, op[1], op[2])
                agent.motion = Motion("move", (op[1], op[2]), now, expected)
            else:
                expected = self.driver.rotate(agent.xid, op[1])
                agent.motion = Motion("rotate", (op[1],), now, expected)
        except DriverError as e:
            log.error("xbot %d: %s refused: %s", agent.xid, op, e)
            self._fail(agent, ErrorID.PMC_ERROR)

    def _claim(self, agent, boxes, immediate=False):
        """
        Claim the boxes a macro will sweep. A claim waits for the xbots the planner moves to be
        cleared out of it. Several may wait at once, but a claim is only made while every xbot the
        planner has to clear out of a claim can still get out of all of them, with room enough
        outside (two claims must not leave no room to clear either). An immediate claim is only
        granted when nothing is in the boxes or on its way into them, so it never waits.
        """
        boxes = tuple(boxes)
        if agent.zone_boxes == boxes and not immediate:
            return True  # held already (an immediate claim is always checked afresh)
        cells = frozenset().union(*(self.layout.cells_in(b) for b in boxes))
        waiting = False  # another claim is waiting for its zone to clear
        for other in self.agents.values():
            if other is agent:
                continue
            if any(overlap(o, b) for o in other.zone_boxes for b in boxes):
                return False
            waiting = waiting or bool(other.zone_boxes and not other.in_macro)
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
        agent.zone_boxes = boxes
        agent.zone = cells
        if not immediate:
            agent.zone_since = self.clock()
            log.info("xbot %d: claims %s", agent.xid, sorted(cells))
        return True

    def _room_to_clear(self, agent, cells):
        """Whether, with this claim added to the others, every xbot the planner moves that stands
        in a claim not its own can get out of all claims, and there is a slot outside for each."""
        zones = {o.xid: o.zone for o in self.agents.values() if o.zone and o is not agent}
        zones[agent.xid] = cells
        claimed = frozenset().union(*zones.values())
        open_cells = self.layout.cells - self._walls(exclude=agent)
        outside = open_cells - claimed
        movers = [m for m in self.agents.values() if self._is_mover(m)]
        if len(outside) < sum(1 for m in movers if m.start not in zones.get(m.xid, ())):
            return False
        if self.lines:  # the claimers stay where they are while they wait (see _plan)
            open_cells -= {o.start for o in movers if o.xid in zones and o.start is not None}
        for m in movers:
            start = m.start
            if start not in claimed or start in zones.get(m.xid, ()):
                continue
            # As the planner sees it: it may cross the claims it stands in, not the others.
            passable = open_cells - frozenset().union(*[z for z in zones.values() if start not in z])
            if self.lines:
                passable -= self.lines.keep_out([start, m.want], idle=m.task is None)
            if not outside & self._steps_from(start, passable).keys():
                return False
        return True

    def _zone_clear(self, agent):
        boxes = agent.zone_boxes
        for other in self.agents.values():
            if other is agent:
                continue
            if other.pose is not None:
                square = self.layout.square(other.pose.x, other.pose.y, other.pose.rz)
                if any(overlap(square, b) for b in boxes):
                    return False
            if other.in_macro and any(overlap(o, b) for o in other.zone_boxes for b in boxes):
                return False
            if other.motion is not None and other.motion.kind == "nav":
                if any(a.to in agent.zone or a.frm in agent.zone for a in other.motion.actions):
                    return False
        return True

    # --- planning ----------------------------------------------------------------------------

    def _is_mover(self, agent):
        if not agent.present or agent.fault or agent.cell is None or agent.in_macro:
            return False
        return agent.task is not None or not self._held(agent)

    def _held(self, agent):
        if self.displace_held:
            return False
        if self._gathering is not None and agent.xid in self._gathering.xids:
            return False  # being lined up and turned: the planner may move it
        if self.held_by is not None:
            return self.held_by(agent.xid)
        return agent.held

    def _is_wall(self, agent):
        # Landed or lost xbots too: they are still where they were last seen.
        return agent.pose is not None and not self._is_mover(agent)

    def _walls(self, exclude=None):
        walls = set()
        for other in self.agents.values():
            if other is not exclude and self._is_wall(other):
                walls |= other.footprint
                if other.zone:
                    walls |= other.zone
                if other.flying:  # held while on its way: every slot of the way, until it is there
                    walls |= {c for a in other.motion.actions for c in (a.frm, a.to)}
        return walls

    def _static_distance(self, agent, cell, toward):
        """Steps from cell to the nearest of toward, around walls only (in line mode: on the lanes)."""
        free = self.layout.cells - self._walls(exclude=agent)
        toward = set(toward)
        if cell in toward:
            return 0
        if self.lines:
            free -= self.lines.keep_out([cell, *toward], idle=agent.task is None)
        seen = {cell}
        queue = deque([(cell, 0)])
        while queue:
            v, d = queue.popleft()
            for u in self.graph[v]:
                if u in seen or u not in free:
                    continue
                if u in toward:
                    return d + 1
                seen.add(u)
                queue.append((u, d + 1))
        return mapf.INF

    def _nearest(self, agent, cells):
        return min(sorted(cells), key=lambda c: self._static_distance(agent, agent.cell, [c]))

    def _choose_flyway(self, agent, task):
        """The flyway to turn on: its whole square clear of xbots that stay put and of others'
        claims, with the least detour."""
        blocked = [self.layout.square(o.pose.x, o.pose.y, o.pose.rz) for o in self.agents.values()
                   if o is not agent and self._is_wall(o)]
        blocked += self._claims(agent)
        target_cells = self._target_cells(task)
        candidates = (self.lines.turn_spots if self.lines and self.lines.turn_spots else self.layout.flyways)
        if not is_square_aligned(task.target.rz):
            # It may not travel turned like that: it turns on the target's own flyway.
            candidates = [self.layout.flyway_at(task.target.x, task.target.y, EXACT_M)]
        best, best_cost = None, mapf.INF
        for flyway in candidates:
            if any(overlap(self.layout.flyway_box(flyway), b) for b in blocked):
                continue
            cells = self.layout.flyway_cells(flyway)
            cost = self._static_distance(agent, agent.cell, cells)
            if cost >= mapf.INF:
                continue
            cost += min(self._static_distance(agent, c, target_cells) for c in cells)
            if cost < best_cost:
                best, best_cost = flyway, cost
        if best is not None:
            log.info("xbot %d: turns on flyway %s", agent.xid, best)
        return best

    def _plan_if_needed(self, now):
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
        walls = frozenset(self._walls())
        zones = frozenset((a.xid, a.zone) for a in self.agents.values() if a.zone)
        roles = frozenset((a.xid, a.task is not None, a.want) for a in movers)
        return walls, zones, roles

    def _plan(self, movers, now):
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
        zones = {a.xid: a.zone for a in self.agents.values() if a.zone}
        # Others' claimed slots: never a goal; not to be entered, except by one already inside,
        # which may cross them on its way out.
        claimed = {a.xid: frozenset().union(*[z for x, z in zones.items() if x != a.xid]) for a in movers}
        keep_out = {a.xid: frozenset().union(*[z for x, z in zones.items() if x != a.xid and start[a.xid] not in z])
                    for a in movers}
        passable = cells
        if self.lines:
            # Line mode: the lanes, and off them only the way from where it is and to where it goes
            # (one nobody holds: and the idle area).
            for a in movers:
                own = [start[a.xid], a.want] if a.task else [start[a.xid], None if self.queue else a.home]
                keep_out[a.xid] |= self.lines.keep_out(own, idle=a.task is None)
                if a.zone:
                    # Waiting for its claim to clear, it stays put: pushed off along a one-way
                    # lane it would have to go all the way round, its claim blocking the lane.
                    keep_out[a.xid] = self.layout.cells - {start[a.xid]}
            passable = cells - self.lines.keep_out(list(start.values()), idle=True)

        goals = {}
        taken = set()

        def settle(agent, want):
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
            for agent, slot in self._line_up(idle, start, cells - taken - frozenset().union(*zones.values()), passable):
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
        motion = mapf.Motion(self.layout.pitch, getattr(self.driver, "speed", 0.5), getattr(self.driver, "accel", 2.0))
        result = mapf.plan(instance, timeout=self.plan_timeout, budget=self.plan_budget, motion=motion)
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
        """The idle xbots' places in the loading queue: its first usable slots, in order, each to
        the nearest xbot not placed yet; of as near ones, one not on a slot the queue needs, so
        when the first one leaves the queue closes up with as few moves as it can."""
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
        """For one standing in a claim not its own: the nearest free slot out of the claims that
        it can get to without passing another xbot; None if it stands in none (or there is none)."""
        here = start[agent.xid]
        if here not in claimed:
            return None
        others = {s for x, s in start.items() if x != agent.xid}
        seen = {here}
        queue = deque([here])
        while queue:
            v = queue.popleft()
            for u in self.graph[v]:
                if u in seen or u not in free or u in others:
                    continue
                if u not in claimed:
                    return u
                seen.add(u)
                queue.append(u)
        return None

    def _steps_from(self, origin, passable, reverse=False):
        """Steps from origin to every slot it can reach across passable ones (reverse: to origin)."""
        graph = self.graph_reverse if reverse else self.graph
        steps = {origin: 0}
        queue = deque([origin])
        while queue:
            v = queue.popleft()
            for u in graph[v]:
                if u in passable and u not in steps:
                    steps[u] = steps[v] + 1
                    queue.append(u)
        return steps

    def _reachable(self, start, goal, free):
        if start == goal:
            return True
        seen = {start}
        queue = deque([start])
        while queue:
            v = queue.popleft()
            for u in self.graph[v]:
                if u == goal:
                    return True
                if u in free and u not in seen:
                    seen.add(u)
                    queue.append(u)
        return False

    def _closest_free(self, start, near, usable, free):
        """The usable cell reachable from start that is closest to near (start itself if it is usable)."""
        reach = {start}
        queue = deque([start])
        while queue:
            v = queue.popleft()
            for u in self.graph[v]:
                if u in free and u not in reach:
                    reach.add(u)
                    queue.append(u)
        candidates = [c for c in reach if c in usable]
        if not candidates:
            return start
        return min(sorted(candidates), key=lambda c: (abs(c[0] - near[0]) + abs(c[1] - near[1]), c != start))

    def _load(self, order, configs, goals, leaving):
        """Turn the configurations into each xbot's actions and the dependencies between them;
        leaving: the slots xbots in flight have still to leave, by which of their moves."""
        per_xbot = mapf.actions(configs, first_id=self._next_action + 1, leaving=leaving)
        for k, xid in enumerate(order):
            agent = self.agents[xid]
            for aid, frm, to, dep in per_xbot[k]:
                agent.actions.append(Action(aid, xid, frm, to, frozenset() if dep is None else frozenset([dep])))
                self._next_action = max(self._next_action, aid)
            agent.goal = goals[xid]

    # --- sending motions ---------------------------------------------------------------------

    def _dispatch(self, now):
        for agent in self.agents.values():
            if agent.motion is not None or agent.fault:
                continue
            if agent.ops:
                if agent.zone is None or self._zone_clear(agent):
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
            try:
                expected = self.driver.move(agent.xid, x, y)
            except DriverError as e:
                log.error("xbot %d: move refused: %s", agent.xid, e)
                self._fail(agent, ErrorID.PMC_ERROR)
                continue
            for _ in run:
                agent.actions.popleft()
            origin = self.layout.center(run[0].frm)
            agent.motion = Motion("nav", (x, y), now, expected, run, origin, step)
