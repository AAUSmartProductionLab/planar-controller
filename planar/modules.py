"""
The planar table as modules, built like the generated IEC 61499 modules:

- PlanarTable: the table as a whole. Resetting connects to the PMC, takes mastership, levitates
  the xbots and, with a loading station, sends the shuttles into its queue with the PMC's auto
  routing; Hold pauses every xbot where it is, Unhold carries on; Abort stops every xbot at
  once. Its Stop and Abort reach every shuttle, its Reset, Start, Hold, Unhold and Clear the
  shuttles in the matching state, so one Reset and one Start of the table bring everything up.
  Another program taking mastership of the PMC aborts it with MastershipLost (32). SetSpeed sets
  how fast every xbot moves.
- PlanarShuttle<N>: one per xbot. An orchestrator occupies a shuttle, runs its MoveToPosition
  skill (X, Y in mm, Rz in degrees) as often as it likes, and releases it. Start answers at once;
  the skill's State turns Succeeded when the xbot has arrived (Failed with an ErrorID if it
  cannot). Hold pauses the xbot (the skill stays Running), Unhold carries on. Resetting recovers
  the xbot if the PMC switched it off after an accident, and checks that it is there and fit to
  move; Abort stops it at once. Its occupant may tell the PMC its payload (SetPayload), and
  whoever may command it may keep the xbot slower than the table (SetSpeed).

Whether the planner may push an xbot out of another's way follows from its shuttle: an occupied
shuttle that is not moving stays where it is (a station may be working on it), and so does one
whose module is not in Idle or Execute. With displace_held, occupied shuttles that are not
moving may be pushed too (never one whose module is stopped or aborted).
"""

import logging
import math

from .coordinator import Coordinator, Pose
from .statemachines import OK, ErrorID, Module, ModuleState, Parameter, Procedure, SkillBody, make_lock, refuse

log = logging.getLogger(__name__)

# How long a shuttle's Resetting waits for an xbot recovered after an accident to levitate.
RECOVER_TIMEOUT_S = 15.0


class MoveToPosition(SkillBody):
    PARAMETERS = [
        Parameter("X", 0.0, 10000.0, 0.0, "mm"),
        Parameter("Y", 0.0, 10000.0, 0.0, "mm"),
        Parameter("Rz", -180.0, 180.0, 0.0, "deg"),
    ]
    RESULTS = ["X", "Y", "Rz"]

    def __init__(self, coordinator, xid):
        self.coordinator = coordinator
        self.xid = xid

    @staticmethod
    def _pose(values):
        return Pose(values["X"] / 1000.0, values["Y"] / 1000.0, math.radians(values["Rz"]))

    def check(self, values):
        return self.coordinator.check_target(self.xid, self._pose(values))

    def begin(self, values, finished):
        error = self.coordinator.submit(self.xid, self._pose(values), finished)
        if error != ErrorID.NONE:
            finished(False, error)

    def halt(self):
        self.coordinator.halt(self.xid)

    def abort(self):
        self.coordinator.abort(self.xid)


class PlanarController:
    def __init__(self, driver, shuttle_ids, *, table_name="PlanarTable", shuttle_name="PlanarShuttle{}",
                 clock=None, displace_held=False, **coordinator_options):
        self.lock = make_lock()
        kwargs = dict(coordinator_options)
        if clock is not None:
            kwargs["clock"] = clock
        self.coordinator = Coordinator(driver, self.lock, **kwargs)
        self.shuttle_ids = list(shuttle_ids)
        self.displace_held = displace_held
        clock = self.coordinator.clock

        coordinator = self.coordinator
        self.table = Module(table_name, self.lock, clock,
                            resetting=Procedure(self._connect),
                            stopping=Procedure(self._stop),
                            aborting=coordinator.abort_all,
                            holding=Procedure(lambda done: self._answer(coordinator.pause(), done)),
                            unholding=Procedure(lambda done: self._answer(coordinator.resume(), done)),
                            resuming=coordinator.resume)
        self.shuttles = {}
        for xid in self.shuttle_ids:
            shuttle = Module(shuttle_name.format(xid), self.lock, clock,
                             resetting=Procedure(lambda done, xid=xid: self._check_xbot(xid, done)),
                             aborting=lambda xid=xid: coordinator.abort(xid),
                             holding=Procedure(lambda done, xid=xid: self._answer(coordinator.pause(xid), done)),
                             unholding=Procedure(lambda done, xid=xid: self._answer(coordinator.resume(xid), done)),
                             resuming=lambda xid=xid: coordinator.resume(xid),
                             parent=self.table)
            shuttle.add_skill("MoveToPosition", MoveToPosition.PARAMETERS, MoveToPosition.RESULTS,
                              MoveToPosition(self.coordinator, xid))
            self.shuttles[xid] = shuttle

        self.coordinator.on_tick.append(self.table.poll)
        self.coordinator.pinned_by = self._pinned
        self.coordinator.on_connection_lost = self.table.fail

    @property
    def modules(self):
        return [self.table, *self.shuttles.values()]

    def start(self):
        self.coordinator.start()

    def shutdown(self):
        self.coordinator.shutdown()

    # --- procedures ----------------------------------------------------------------------

    def _connect(self, done):
        def job():
            error = self.coordinator.connect(self.shuttle_ids)
            if error != ErrorID.NONE:
                done(False, error)
                return
            with self.lock:  # the shuttles are all stopped now: line them up at the loading station
                self.coordinator.gather(self.shuttle_ids, done)
        self.coordinator.run_job(job)

    def _stop(self, done):
        self.coordinator.cancel_gather()  # a Stop during Resetting stops the xbots it sent
        done(True, ErrorID.NONE)

    @staticmethod
    def _answer(error, done):
        done(error == ErrorID.NONE, error)

    def _check_xbot(self, xid, done):
        """Recover the xbot if the PMC switched it off after an accident, then check it; a
        recovered one is waited for until it has levitated. (The table's Resetting has recovered
        them all already, so its shuttles are checked at once and are Idle with the table.)"""
        coordinator = self.coordinator
        if not coordinator.connected or not coordinator.recover(xid):
            self._answer(coordinator.check_xbot(xid), done)
            return
        deadline = coordinator.clock() + RECOVER_TIMEOUT_S

        def check():
            error = coordinator.check_xbot(xid)
            if error == ErrorID.NONE or coordinator.clock() > deadline:
                coordinator.on_tick.remove(check)
                self._answer(error, done)
        coordinator.on_tick.append(check)

    # --- equipment commands (OPC UA, any thread) -----------------------------------------

    def set_payload(self, xid, session, kg, cg_height_mm):
        """The shuttle's occupant tells the PMC the xbot's payload (kg, centre of gravity mm above
        the xbot); not while its skill runs."""
        shuttle = self.shuttles[xid]
        with self.lock:
            if not shuttle.occupation.holds(session):
                return refuse(ErrorID.NOT_PERMITTED)
            if not (kg >= 0 and cg_height_mm >= 0):  # (NaN too)
                return refuse(ErrorID.OUT_OF_RANGE)
            if any(s.active for s in shuttle.skills.values()):
                return refuse(ErrorID.BUSY)
            error = self.coordinator.set_payload(xid, float(kg), cg_height_mm / 1000.0)
        return OK if error == ErrorID.NONE else refuse(error)

    def set_speed(self, xid, session, speed, accel):
        """How fast xbot xid may move at most (m/s, m/s^2; 0 or NaN: as fast as the table's), or
        every xbot (xid None: the table's; 0 or NaN: as it started): whoever may command its
        module (the occupant, or anyone while it is not occupied)."""
        module = self.table if xid is None else self.shuttles[xid]
        with self.lock:
            if not module.occupation.allows(session):
                return refuse(ErrorID.NOT_PERMITTED)
            error = self.coordinator.set_speed(xid, speed, accel)
        return OK if error == ErrorID.NONE else refuse(error)

    def zone_command(self, name, operation, session):
        """Deactivate or Activate a zone: whoever may command the table (its occupant, or anyone
        while it is not occupied)."""
        with self.lock:
            if not self.table.occupation.allows(session):
                return refuse(ErrorID.NOT_PERMITTED)
            if operation == "Deactivate":
                error = self.coordinator.deactivate_zone(name)
            else:
                error = self.coordinator.activate_zone(name)
        return OK if error == ErrorID.NONE else refuse(error)

    # --- asked by the coordinator, with the lock held ------------------------------------

    def _pinned(self, xid):
        """Whether the planner must leave this xbot where it is. Asked at the moment it plans, so a
        shuttle whose move has just ended is pinned before anything can push it."""
        shuttle = self.shuttles.get(xid)
        if shuttle is None:
            return True  # not offered as a shuttle: nobody here may move it
        moving = any(s.active for s in shuttle.skills.values())
        usable = shuttle.state in (ModuleState.IDLE, ModuleState.EXECUTE)  # (Held: it stays put too)
        return not usable or (shuttle.occupation.occupied and not moving and not self.displace_held)
