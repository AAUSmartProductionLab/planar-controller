"""Implement shared-lock occupation, skill lifecycles and PackML module states."""

import logging
import threading
from dataclasses import dataclass
from enum import IntEnum

log = logging.getLogger(__name__)

SUCCEEDED_HOLD_S = 1.5


class ErrorID(IntEnum):
    """What every refused command answers and every failed skill or procedure shows."""
    NONE = 0
    NOT_READY = 4           # the module or skill is not in a state that takes the command
    NOT_PERMITTED = 5       # the caller does not hold the occupation (or another session does)
    BUSY = 6                # the skill is running, or the xbot is already on its way somewhere
    INTERRUPTED = 7         # stopped before it finished
    OUT_OF_RANGE = 8        # a parameter outside its limits, or an empty session
    BLOCKED = 20            # no way to the target within BLOCKED_TIMEOUT_S (by default it waits)
    XBOT_FAULT = 21         # the PMC reports the xbot in error, obstacle detected or stopped
    PMC_ERROR = 22          # the PMC refused a command
    MOTION_TIMEOUT = 23     # a move did not arrive in time
    PROCEDURE_FAILED = 24   # Resetting or Stopping did not complete
    NOT_ON_TABLE = 25       # the target pose is not entirely on flyways (outside, or over a hole)
    ANGLE_NOT_ALLOWED = 26  # an angle that is not a multiple of 90 deg, away from a flyway centre
    PMC_NOT_CONNECTED = 27  # no connection to the PMC (never made, or lost)
    XBOT_NOT_DETECTED = 28  # the PMC does not report this xbot (missing or landed)
    XBOT_CROOKED = 29       # turned off the axes away from a flyway centre: it cannot travel
    MOTION_DEVIATED = 30    # a move ended somewhere else than it was sent
    LINE_LAYOUT_INVALID = 31  # line mode: the line layout is none or does not fit the table (LineLayoutProblem)
    MASTERSHIP_LOST = 32    # another program (PMT, another controller) has taken mastership of the PMC
    ZONE_CLOSED = 33        # the target is in a zone that is switched off or fenced (Equipment/Zones)
    ZONES_INVALID = 34      # a zone of ZONES is not on the table, or the PMC would not define it


class SkillState(IntEnum):
    """Numeric states exposed by each skill's execution lifecycle."""
    IDLE = 0
    RUNNING = 1
    STOPPING = 2
    SUCCEEDED = 3
    FAILED = 4
    ABORTED = 5


class ModuleState(IntEnum):
    """PackML state numbers (OMAC PackTags StateCurrent)."""
    CLEARING = 1
    STOPPED = 2
    STARTING = 3
    IDLE = 4
    EXECUTE = 6
    STOPPING = 7
    ABORTING = 8
    ABORTED = 9
    HOLDING = 10
    HELD = 11
    UNHOLDING = 12
    RESETTING = 15


_HOLD_STATES = (ModuleState.HOLDING, ModuleState.HELD, ModuleState.UNHOLDING)


Answer = tuple[bool, ErrorID]
OK: Answer = (True, ErrorID.NONE)


def refuse(error) -> Answer:
    """Return a rejected command result with a normalized ErrorID."""
    return (False, ErrorID(error))


class Occupation:
    """Track exclusive ownership by session without publishing the holder's identifier."""
    def __init__(self, lock, on_change=None):
        """Initialize an unoccupied resource with a shared lock and optional change callback."""
        self._lock = lock
        self._holder = None
        self._on_change = on_change

    @property
    def occupied(self):
        """Whether a session currently holds the resource."""
        return self._holder is not None

    def holds(self, session):
        """Whether the nonempty session matches the current holder."""
        return bool(session) and session == self._holder

    def allows(self, session):
        """Whether the session may command what is occupied: its occupant, or anyone while it is not
        occupied.
        """
        return self._holder is None or self.holds(session)

    def occupy(self, session) -> Answer:
        """Acquire ownership for a nonempty session, accepting repeated acquisition by its holder."""
        with self._lock:
            if not session:
                return refuse(ErrorID.OUT_OF_RANGE)
            if self._holder == session:
                return OK
            if self._holder is not None:
                return refuse(ErrorID.NOT_PERMITTED)
            self._holder = session
        self._changed()
        return OK

    def release(self, session) -> Answer:
        """Release ownership only for the current holder."""
        with self._lock:
            if not self.holds(session):
                return refuse(ErrorID.NOT_PERMITTED)
            self._holder = None
        self._changed()
        return OK

    def _changed(self):
        """Notify the optional callback of the current occupied state."""
        if self._on_change:
            self._on_change(self.occupied)


@dataclass(frozen=True)
class Parameter:
    """A named numeric skill parameter with inclusive bounds, default and unit."""
    name: str
    lower: float
    upper: float
    default: float = 0.0
    unit: str = ""

    def in_range(self, value):
        """Whether a value lies within the parameter's inclusive bounds."""
        return self.lower <= value <= self.upper


class SkillBody:
    """What a skill does. The skill core calls these with its lock held."""

    def check(self, values) -> ErrorID:
        """Refuse a start the parameters' ranges do not cover (equipment busy, target unusable)."""
        return ErrorID.NONE

    def begin(self, values, finished):
        """Start running; call finished(ok, error, results) once, from any thread."""
        raise NotImplementedError

    def halt(self):
        """Stop in an orderly way; finished(...) still follows."""

    def abort(self):
        """Switch off at once; finished(...) may or may not follow and is ignored."""


class Skill:
    """Gate commands and track asynchronous execution, results and errors for a skill body."""
    def __init__(self, name, lock, parameters, results, body, module):
        """Bind the skill body and parameters to a module's lock and clock."""
        self.name = name
        self._lock = lock
        self.parameters = list(parameters)
        self.result_names = list(results)
        self.body = body
        self.module = module
        self.state = SkillState.IDLE
        self.error = ErrorID.NONE
        self.values = {p.name: p.default for p in self.parameters}
        self.results = {r: 0.0 for r in self.result_names}
        self.started_over_ua = False
        self._run = 0
        self._succeeded_at = None
        self._clock = module.clock

    @property
    def active(self):
        """Whether the skill is Running or Stopping."""
        return self.state in (SkillState.RUNNING, SkillState.STOPPING)

    # --- OPC UA commands -----------------------------------------------------------------

    def start(self, session, values) -> Answer:
        """Check ownership, module readiness, activity, ranges and equipment before starting; return
        acceptance immediately.
        """
        with self._lock:
            if not self.module.occupation.holds(session):
                return refuse(ErrorID.NOT_PERMITTED)
            if not self.module.ready_for_skills():
                return refuse(ErrorID.NOT_READY)
            if self.active:
                return refuse(ErrorID.BUSY)
            if self.state == SkillState.ABORTED:
                return refuse(ErrorID.NOT_READY)
            merged = dict(self.values)
            merged.update(values)
            for p in self.parameters:
                if not p.in_range(merged[p.name]):
                    return refuse(ErrorID.OUT_OF_RANGE)
            error = self.body.check(merged)
            if error != ErrorID.NONE:
                return refuse(error)
            self._begin(merged, over_ua=True)
            return OK

    def stop(self, session) -> Answer:
        """Authorize the holder's request for an orderly halt."""
        with self._lock:
            if not self.module.occupation.holds(session):
                return refuse(ErrorID.NOT_PERMITTED)
            return self.halt()

    def abort(self, session) -> Answer:
        """Authorize the holder's request for immediate skill cancellation."""
        with self._lock:
            if not self.module.occupation.holds(session):
                return refuse(ErrorID.NOT_PERMITTED)
            return self.kill()

    def reset(self, session) -> Answer:
        """Return an aborted skill to Idle when requested by its holder."""
        with self._lock:
            if not self.module.occupation.holds(session):
                return refuse(ErrorID.NOT_PERMITTED)
            if self.state != SkillState.ABORTED:
                return refuse(ErrorID.NOT_READY)
            self._set(SkillState.IDLE, ErrorID.NONE)
            return OK

    # --- from the module (not gated again) -----------------------------------------------

    def halt(self) -> Answer:
        """Enter Stopping and request orderly body completion without rechecking ownership."""
        with self._lock:
            if self.state != SkillState.RUNNING:
                return refuse(ErrorID.NOT_READY)
            self._set(SkillState.STOPPING)
            self.body.halt()
            return OK

    def kill(self) -> Answer:
        """Enter Aborted, invalidate late callbacks and abort the body without an ownership check."""
        with self._lock:
            if self.state == SkillState.ABORTED:
                return refuse(ErrorID.NOT_READY)
            self._run += 1  # a late finished() of the aborted run is ignored
            self._set(SkillState.ABORTED)
            self.body.abort()
            return OK

    def poll(self):
        """Return a successful skill to Idle after SUCCEEDED_HOLD_S; failed skills remain failed."""
        with self._lock:
            if self.state == SkillState.SUCCEEDED and self._clock() - self._succeeded_at >= SUCCEEDED_HOLD_S:
                self._set(SkillState.IDLE)

    # --- internals -----------------------------------------------------------------------

    def _begin(self, values, over_ua):
        """Start a new run, reset results and bind its completion callback to the run ID."""
        self._run += 1
        run = self._run
        self.values = values
        self.results = {r: 0.0 for r in self.result_names}
        self.started_over_ua = over_ua
        self._set(SkillState.RUNNING, ErrorID.NONE)

        def finished(ok, error=ErrorID.NONE, results=None):
            """Report completion for the captured run, even when called from another thread."""
            self._finished(run, ok, error, results)

        self.body.begin(values, finished)

    def _finished(self, run, ok, error, results):
        """Apply results for the active run; ignore stale callbacks and mark orderly stops Interrupted."""
        with self._lock:
            if run != self._run or not self.active:
                return
            if results:
                self.results.update(results)
            if self.state == SkillState.STOPPING:
                self._set(SkillState.FAILED, ErrorID.INTERRUPTED)
            elif ok:
                self._succeeded_at = self._clock()
                self._set(SkillState.SUCCEEDED, ErrorID.NONE)
            else:
                self._set(SkillState.FAILED, ErrorID(error))

    def _set(self, state, error=None):
        """Update skill state and optional error, logging transitions."""
        if error is not None:
            self.error = ErrorID(error)
        if state != self.state:
            log.info("%s/%s: %s -> %s%s", self.module.name, self.name, self.state.name, state.name,
                     f" ({self.error.name})" if self.error else "")
        self.state = state


class Procedure:
    """A module procedure (Resetting, Stopping): run(done) and done(ok, error) once."""

    def __init__(self, run=None):
        """Store an optional asynchronous procedure implementation."""
        self._run = run

    def __call__(self, done):
        """Run the procedure with done(ok, error), or succeed immediately when none is configured."""
        if self._run is None:
            done(True, ErrorID.NONE)
        else:
            self._run(done)


class Module:
    """A PackML state manager with occupation, skills and optional child modules."""

    def __init__(self, name, lock, clock, resetting=None, stopping=None, aborting=None, parent=None,
                 holding=None, unholding=None, resuming=None):
        """Configure lifecycle procedures and parent linkage, starting in Stopped."""
        self.name = name
        self._lock = lock
        self.clock = clock
        self.state = ModuleState.STOPPED
        self.error = ErrorID.NONE
        self.occupation = Occupation(lock, on_change=self._occupation_changed)
        self.skills = {}
        self.children = []
        self.parent = parent
        if parent is not None:
            parent.children.append(self)
        self._resetting = resetting or Procedure()
        self._stopping = stopping or Procedure()
        self._aborting = aborting or (lambda: None)
        self._holding = holding or Procedure()
        self._unholding = unholding or Procedure()
        self._resuming = resuming or (lambda: None)
        self._procedure_run = 0
        self._stopping_procedure_started = False
        self.listeners = []  # called with the module after every state or occupation change

    def add_skill(self, name, parameters, results, body):
        """Create and register a skill sharing this module's lock."""
        skill = Skill(name, self._lock, parameters, results, body, self)
        self.skills[name] = skill
        return skill

    def ready_for_skills(self):
        """Whether this module and every parent are in Execute."""
        if self.state != ModuleState.EXECUTE:
            return False
        return self.parent is None or self.parent.ready_for_skills()

    # --- OPC UA commands -----------------------------------------------------------------
    # Only the occupant may command an occupied module; Abort is always taken.

    def command(self, name, session) -> Answer:
        """Authorize and dispatch a named command; Abort is allowed regardless of occupation."""
        with self._lock:
            if name != "Abort" and not self.occupation.allows(session):
                return refuse(ErrorID.NOT_PERMITTED)
            return {
                "Reset": self.reset,
                "Start": self.start,
                "Hold": self.hold,
                "Unhold": self.unhold,
                "Stop": self.stop,
                "Abort": self.abort,
                "Clear": self.clear,
            }[name]()

    def reset(self) -> Answer:
        """Enter Resetting from Stopped and run initialization if the parent permits it."""
        with self._lock:
            if self.state != ModuleState.STOPPED:
                return refuse(ErrorID.NOT_READY)
            if self.parent is not None and self.parent.state not in (ModuleState.IDLE, ModuleState.EXECUTE):
                return refuse(ErrorID.NOT_READY)
            self._set(ModuleState.RESETTING, ErrorID.NONE)
            self._run_procedure(self._resetting, self._reset_done)
            return OK

    def start(self) -> Answer:
        """Enter Execute from Idle and start idle children when the parent is executing."""
        with self._lock:
            if self.state != ModuleState.IDLE:
                return refuse(ErrorID.NOT_READY)
            if self.parent is not None and self.parent.state != ModuleState.EXECUTE:
                return refuse(ErrorID.NOT_READY)
            self._set(ModuleState.STARTING)
            self._set(ModuleState.EXECUTE)
            for child in self.children:
                if child.state == ModuleState.IDLE:
                    child.start()
            self._notify()
            return OK

    def hold(self) -> Answer:
        """Pause an executing module and its executing children through the Holding procedure."""
        with self._lock:
            if self.state != ModuleState.EXECUTE:
                return refuse(ErrorID.NOT_READY)
            self._set(ModuleState.HOLDING)
            for child in self.children:
                if child.state == ModuleState.EXECUTE:
                    child.hold()
            self._run_procedure(self._holding, self._hold_done)
            self._notify()
            return OK

    def unhold(self) -> Answer:
        """Run the resume procedure from Held when the parent permits execution."""
        with self._lock:
            if self.state != ModuleState.HELD:
                return refuse(ErrorID.NOT_READY)
            if self.parent is not None and self.parent.state not in (ModuleState.EXECUTE, ModuleState.UNHOLDING):
                return refuse(ErrorID.NOT_READY)
            self._set(ModuleState.UNHOLDING)
            self._run_procedure(self._unholding, self._unhold_done)
            self._notify()
            return OK

    def stop(self) -> Answer:
        """Resume held equipment, halt OPC UA skills and children, then stop once active skills finish."""
        with self._lock:
            if self.state not in (ModuleState.IDLE, ModuleState.STARTING, ModuleState.EXECUTE, ModuleState.RESETTING,
                                  *_HOLD_STATES):
                return refuse(ErrorID.NOT_READY)
            if self.state in _HOLD_STATES:
                try:
                    self._resuming()
                except Exception:
                    log.exception("%s: resuming", self.name)
            self._procedure_run += 1  # a Resetting, Holding or Unholding still running is dropped
            self._set(ModuleState.STOPPING)
            self._stopping_procedure_started = False
            for skill in self.skills.values():
                if skill.started_over_ua and skill.state == SkillState.RUNNING:
                    skill.halt()
            for child in self.children:
                child.stop()
            self._continue_stopping()
            self._notify()
            return OK

    def abort(self) -> Answer:
        """Invalidate procedures, abort active skills and equipment, and propagate Abort to all children."""
        with self._lock:
            if self.state in (ModuleState.ABORTING, ModuleState.ABORTED):
                return refuse(ErrorID.NOT_READY)
            self._procedure_run += 1
            self._set(ModuleState.ABORTING)
            for skill in self.skills.values():
                if skill.active:
                    skill.kill()
            try:
                self._aborting()
            except Exception:
                log.exception("%s: aborting", self.name)
            for child in self.children:
                child.abort()
            self._set(ModuleState.ABORTED)
            self._notify()
            return OK

    def clear(self) -> Answer:
        """Return an aborted module and its aborted children to Stopped."""
        with self._lock:
            if self.state != ModuleState.ABORTED:
                return refuse(ErrorID.NOT_READY)
            self._set(ModuleState.CLEARING)
            self._set(ModuleState.STOPPED)
            for child in self.children:
                if child.state == ModuleState.ABORTED:
                    child.clear()
            self._notify()
            return OK

    def fail(self, error):
        """Something the module depends on is gone (the PMC connection): abort with an error."""
        with self._lock:
            self.abort()
            self.error = ErrorID(error)
            self._notify()

    # --- driven by the controller's loop -------------------------------------------------

    def poll(self):
        """Poll skills and children, advancing orderly stopping when work has finished."""
        with self._lock:
            for skill in self.skills.values():
                skill.poll()
            if self.state == ModuleState.STOPPING:
                self._continue_stopping()
            for child in self.children:
                child.poll()

    # --- internals -----------------------------------------------------------------------

    def _continue_stopping(self):
        """Start the Stopping procedure after all skills finish and children stop or abort."""
        if self._stopping_procedure_started:
            return
        if any(s.active for s in self.skills.values()):
            return
        if any(c.state not in (ModuleState.STOPPED, ModuleState.ABORTED) for c in self.children):
            return
        self._stopping_procedure_started = True
        self._run_procedure(self._stopping, self._stop_done)

    def _run_procedure(self, procedure, then):
        """Invoke a procedure with a run-scoped callback that ignores stale completions."""
        self._procedure_run += 1
        run = self._procedure_run

        def done(ok, error=ErrorID.NONE):
            """Ignore stale procedure completions and apply current results under the shared lock."""
            with self._lock:
                if run == self._procedure_run:
                    then(ok, ErrorID(error))

        procedure(done)

    def _reset_done(self, ok, error):
        """Finish Resetting as Idle or return to Stopped on failure; reset eligible children on success."""
        if self.state != ModuleState.RESETTING:
            return
        if not ok:
            self._set(ModuleState.STOPPED, error or ErrorID.PROCEDURE_FAILED)
            self._notify()
            return
        self._set(ModuleState.IDLE)
        for child in self.children:
            if child.state == ModuleState.STOPPED:
                child.reset()
        self._notify()

    def _hold_done(self, ok, error):
        """Finish Holding as Held, or abort if the pause procedure failed."""
        if self.state != ModuleState.HOLDING:
            return
        if not ok:
            log.error("%s: holding failed (%s)", self.name, ErrorID(error or ErrorID.PROCEDURE_FAILED).name)
            self.fail(error or ErrorID.PROCEDURE_FAILED)
            return
        self._set(ModuleState.HELD)
        self._notify()

    def _unhold_done(self, ok, error):
        """Resume execution and held children, or abort if resuming failed."""
        if self.state != ModuleState.UNHOLDING:
            return
        if not ok:
            log.error("%s: unholding failed (%s)", self.name, ErrorID(error or ErrorID.PROCEDURE_FAILED).name)
            self.fail(error or ErrorID.PROCEDURE_FAILED)
            return
        self._set(ModuleState.EXECUTE)
        for child in self.children:
            if child.state == ModuleState.HELD:
                child.unhold()
        self._notify()

    def _stop_done(self, ok, error):
        """Finish Stopping as Stopped, preserving a procedure failure as the module error."""
        if self.state != ModuleState.STOPPING:
            return
        self._set(ModuleState.STOPPED, ErrorID.NONE if ok else (error or ErrorID.PROCEDURE_FAILED))
        self._notify()

    def _occupation_changed(self, occupied):
        """Notify listeners after an occupation change."""
        self._notify()

    def _set(self, state, error=None):
        """Update module state and optional error, logging transitions."""
        if error is not None:
            self.error = ErrorID(error)
        if state != self.state:
            log.info("%s: %s -> %s%s", self.name, self.state.name, state.name,
                     f" ({self.error.name})" if self.error else "")
        self.state = state

    def _notify(self):
        """Notify listeners of the current module state and occupation."""
        for listener in list(self.listeners):
            try:
                listener(self)
            except Exception:
                log.exception("%s: listener", self.name)


def make_lock():
    """Create the reentrant lock shared by modules, skills and the coordinator."""
    return threading.RLock()
