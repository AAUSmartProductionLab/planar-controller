"""
The generic module level, as the generated IEC 61499 modules have it (work.md, "A generated
module"):

- Occupation: Occupy/Release(Session). A client picks a UUID, occupies with it, and every
  command it sends carries it. Whether the module is occupied is published, the session never.
- Module: the PackML state manager. Stopped -Reset-> Resetting {procedure} -> Idle -Start->
  Execute (stays while the occupant runs skills); Stop -> Stopping (the skills started over
  OPC UA stop, wait until none is active, Stopping procedure) -> Stopped; Abort -> Aborted
  (skills and equipment off) -Clear-> Stopped.
- Skill: the skill core. Idle 0, Running 1, Stopping 2, Succeeded 3, Failed 4, Aborted 5. Start
  from Idle, Succeeded or Failed; Succeeded returns to Idle by itself after 1.5 s, Failed stays;
  Stop -> Stopping -> Failed with Interrupted; Abort -> Aborted at once; Reset only from Aborted.

Every command answers [Accepted, ErrorID] at once; what it starts finishes later and shows in
State. The skill's gate checks, in this order: the caller holds the occupation (NotPermitted),
the module is in Execute (NotReady), the skill is not running (Busy), the parameters are in
range (OutOfRange), the equipment is free (Busy).

All state is guarded by one lock shared by everything in the process; whatever runs a skill or a
procedure reports back through the callbacks it was handed, from any thread.
"""

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
    LINE_LAYOUT_INVALID = 31  # line mode: the line layout does not fit the table (see the log)


class SkillState(IntEnum):
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
    RESETTING = 15


Answer = tuple[bool, ErrorID]
OK: Answer = (True, ErrorID.NONE)


def refuse(error) -> Answer:
    return (False, ErrorID(error))


class Occupation:
    def __init__(self, lock, on_change=None):
        self._lock = lock
        self._holder = None
        self._on_change = on_change

    @property
    def occupied(self):
        return self._holder is not None

    def holds(self, session):
        return bool(session) and session == self._holder

    def occupy(self, session) -> Answer:
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
        with self._lock:
            if not self.holds(session):
                return refuse(ErrorID.NOT_PERMITTED)
            self._holder = None
        self._changed()
        return OK

    def _changed(self):
        if self._on_change:
            self._on_change(self.occupied)


@dataclass(frozen=True)
class Parameter:
    name: str
    lower: float
    upper: float
    default: float = 0.0
    unit: str = ""

    def in_range(self, value):
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
    def __init__(self, name, lock, parameters, results, body, module):
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
        return self.state in (SkillState.RUNNING, SkillState.STOPPING)

    # --- OPC UA commands -----------------------------------------------------------------

    def start(self, session, values) -> Answer:
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
        with self._lock:
            if not self.module.occupation.holds(session):
                return refuse(ErrorID.NOT_PERMITTED)
            return self.halt()

    def abort(self, session) -> Answer:
        with self._lock:
            if not self.module.occupation.holds(session):
                return refuse(ErrorID.NOT_PERMITTED)
            return self.kill()

    def reset(self, session) -> Answer:
        with self._lock:
            if not self.module.occupation.holds(session):
                return refuse(ErrorID.NOT_PERMITTED)
            if self.state != SkillState.ABORTED:
                return refuse(ErrorID.NOT_READY)
            self._set(SkillState.IDLE, ErrorID.NONE)
            return OK

    # --- from the module (not gated again) -----------------------------------------------

    def halt(self) -> Answer:
        with self._lock:
            if self.state != SkillState.RUNNING:
                return refuse(ErrorID.NOT_READY)
            self._set(SkillState.STOPPING)
            self.body.halt()
            return OK

    def kill(self) -> Answer:
        with self._lock:
            if self.state == SkillState.ABORTED:
                return refuse(ErrorID.NOT_READY)
            self._run += 1  # a late finished() of the aborted run is ignored
            self._set(SkillState.ABORTED)
            self.body.abort()
            return OK

    def poll(self):
        with self._lock:
            if self.state == SkillState.SUCCEEDED and self._clock() - self._succeeded_at >= SUCCEEDED_HOLD_S:
                self._set(SkillState.IDLE)

    # --- internals -----------------------------------------------------------------------

    def _begin(self, values, over_ua):
        self._run += 1
        run = self._run
        self.values = values
        self.results = {r: 0.0 for r in self.result_names}
        self.started_over_ua = over_ua
        self._set(SkillState.RUNNING, ErrorID.NONE)

        def finished(ok, error=ErrorID.NONE, results=None):
            self._finished(run, ok, error, results)

        self.body.begin(values, finished)

    def _finished(self, run, ok, error, results):
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
        if error is not None:
            self.error = ErrorID(error)
        if state != self.state:
            log.info("%s/%s: %s -> %s%s", self.module.name, self.name, self.state.name, state.name,
                     f" ({self.error.name})" if self.error else "")
        self.state = state


class Procedure:
    """A module procedure (Resetting, Stopping): run(done) and done(ok, error) once."""

    def __init__(self, run=None):
        self._run = run

    def __call__(self, done):
        if self._run is None:
            done(True, ErrorID.NONE)
        else:
            self._run(done)


class Module:
    """
    The PackML state manager of one module, with its occupation and skills.

    A module may have a parent (the table is the shuttles' parent): the parent's Stop and Abort
    reach every child, its Reset, Start and Clear the children in the matching state, and a
    child's skills run only while the parent is in Execute too.
    """

    def __init__(self, name, lock, clock, resetting=None, stopping=None, aborting=None, parent=None):
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
        self._procedure_run = 0
        self._stopping_procedure_started = False
        self.listeners = []  # called with the module after every state or occupation change

    def add_skill(self, name, parameters, results, body):
        skill = Skill(name, self._lock, parameters, results, body, self)
        self.skills[name] = skill
        return skill

    def ready_for_skills(self):
        if self.state != ModuleState.EXECUTE:
            return False
        return self.parent is None or self.parent.ready_for_skills()

    # --- OPC UA commands -----------------------------------------------------------------
    # Only the occupant may command an occupied module; Abort is always taken.

    def command(self, name, session) -> Answer:
        with self._lock:
            if name != "Abort" and self.occupation.occupied and not self.occupation.holds(session):
                return refuse(ErrorID.NOT_PERMITTED)
            return {
                "Reset": self.reset,
                "Start": self.start,
                "Stop": self.stop,
                "Abort": self.abort,
                "Clear": self.clear,
            }[name]()

    def reset(self) -> Answer:
        with self._lock:
            if self.state != ModuleState.STOPPED:
                return refuse(ErrorID.NOT_READY)
            if self.parent is not None and self.parent.state not in (ModuleState.IDLE, ModuleState.EXECUTE):
                return refuse(ErrorID.NOT_READY)
            self._set(ModuleState.RESETTING, ErrorID.NONE)
            self._run_procedure(self._resetting, self._reset_done)
            return OK

    def start(self) -> Answer:
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

    def stop(self) -> Answer:
        with self._lock:
            if self.state not in (ModuleState.IDLE, ModuleState.STARTING, ModuleState.EXECUTE, ModuleState.RESETTING):
                return refuse(ErrorID.NOT_READY)
            self._procedure_run += 1  # a Resetting still running is dropped
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
        with self._lock:
            for skill in self.skills.values():
                skill.poll()
            if self.state == ModuleState.STOPPING:
                self._continue_stopping()
            for child in self.children:
                child.poll()

    # --- internals -----------------------------------------------------------------------

    def _continue_stopping(self):
        if self._stopping_procedure_started:
            return
        if any(s.active for s in self.skills.values()):
            return
        if any(c.state not in (ModuleState.STOPPED, ModuleState.ABORTED) for c in self.children):
            return
        self._stopping_procedure_started = True
        self._run_procedure(self._stopping, self._stop_done)

    def _run_procedure(self, procedure, then):
        self._procedure_run += 1
        run = self._procedure_run

        def done(ok, error=ErrorID.NONE):
            with self._lock:
                if run == self._procedure_run:
                    then(ok, ErrorID(error))

        procedure(done)

    def _reset_done(self, ok, error):
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

    def _stop_done(self, ok, error):
        if self.state != ModuleState.STOPPING:
            return
        self._set(ModuleState.STOPPED, ErrorID.NONE if ok else (error or ErrorID.PROCEDURE_FAILED))
        self._notify()

    def _occupation_changed(self, occupied):
        self._notify()

    def _set(self, state, error=None):
        if error is not None:
            self.error = ErrorID(error)
        if state != self.state:
            log.info("%s: %s -> %s%s", self.name, self.state.name, state.name,
                     f" ({self.error.name})" if self.error else "")
        self.state = state

    def _notify(self):
        for listener in list(self.listeners):
            try:
                listener(self)
            except Exception:
                log.exception("%s: listener", self.name)


def make_lock():
    return threading.RLock()
