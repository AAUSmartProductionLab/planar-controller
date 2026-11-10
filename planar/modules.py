"""Expose the planar table and shuttles as occupied PackML modules with motion skills."""

import logging
import math

from .coordinator import Coordinator, Pose
from .statemachines import OK, ErrorID, Module, ModuleState, Parameter, Procedure, SkillBody, make_lock, refuse

log = logging.getLogger(__name__)

# How long a shuttle's Resetting waits for an xbot recovered after an accident to levitate.
RECOVER_TIMEOUT_S = 15.0


class MoveToPosition(SkillBody):
    """Adapt a shuttle's position skill from millimetres and degrees to coordinator SI units."""
    PARAMETERS = [
        Parameter("X", 0.0, 10000.0, 0.0, "mm"),
        Parameter("Y", 0.0, 10000.0, 0.0, "mm"),
        Parameter("Rz", -180.0, 180.0, 0.0, "deg"),
    ]
    RESULTS = ["X", "Y", "Rz"]

    def __init__(self, coordinator, xid):
        """Bind the skill to a coordinator and xbot ID."""
        self.coordinator = coordinator
        self.xid = xid

    @staticmethod
    def _pose(values):
        """Convert skill X/Y millimetres and Rz degrees into a Pose in metres and radians."""
        return Pose(values["X"] / 1000.0, values["Y"] / 1000.0, math.radians(values["Rz"]))

    def check(self, values):
        """Validate the requested pose through the coordinator."""
        return self.coordinator.check_target(self.xid, self._pose(values))

    def begin(self, values, finished):
        """Submit the pose request and report immediate rejection through finished."""
        error = self.coordinator.submit(self.xid, self._pose(values), finished)
        if error != ErrorID.NONE:
            finished(False, error)

    def halt(self):
        """Request orderly completion of the committed motion before interrupting the skill."""
        self.coordinator.halt(self.xid)

    def abort(self):
        """Stop the xbot and discard its pending task immediately."""
        self.coordinator.abort(self.xid)


class PlanarController:
    """Wire table and shuttle state machines to a selectable shared coordinator."""
    def __init__(self, driver, shuttle_ids, *, table_name="PlanarTable", shuttle_name="PlanarShuttle{}",
                 clock=None, displace_held=False, planner="classic", **coordinator_options):
        """Create modules, skills and lifecycle callbacks for the selected planner and driver."""
        self.lock = make_lock()
        kwargs = dict(coordinator_options)
        if clock is not None:
            kwargs["clock"] = clock
        if planner == 'classic':
            coordinator_type = Coordinator
        elif planner == 'blended':
            from .coordinator_blended import Coordinator as coordinator_type
        else:
            raise ValueError(f'unknown planner {planner!r}; choose classic or blended')
        self.planner = planner
        self.coordinator = coordinator_type(driver, self.lock, **kwargs)
        log.info('planner: %s', planner)
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
        """Return the table followed by its shuttle modules."""
        return [self.table, *self.shuttles.values()]

    def start(self):
        """Start the coordinator's background control loop."""
        self.coordinator.start()

    def shutdown(self):
        """Stop the coordinator and disconnect its driver."""
        self.coordinator.shutdown()

    # --- procedures ----------------------------------------------------------------------

    def _connect(self, done):
        """Queue table connection and loading-queue setup for the Reset procedure."""
        def job():
            """Connect outside the module lock, then gather shuttles and complete Reset."""
            error = self.coordinator.connect(self.shuttle_ids)
            if error != ErrorID.NONE:
                done(False, error)
                return
            with self.lock:  # the shuttles are all stopped now: line them up at the loading station
                self.coordinator.gather(self.shuttle_ids, done)
        self.coordinator.run_job(job)

    def _stop(self, done):
        """Cancel any loading-queue setup and complete the table's Stop procedure."""
        self.coordinator.cancel_gather()  # a Stop during Resetting stops the xbots it sent
        done(True, ErrorID.NONE)

    @staticmethod
    def _answer(error, done):
        """Translate an ErrorID into a procedure completion callback."""
        done(error == ErrorID.NONE, error)

    def _check_xbot(self, xid, done):
        """Recover an accident-disabled xbot and wait for levitation before completing Reset."""
        coordinator = self.coordinator
        if not coordinator.connected or not coordinator.recover(xid):
            self._answer(coordinator.check_xbot(xid), done)
            return
        deadline = coordinator.clock() + RECOVER_TIMEOUT_S

        def check():
            """Finish recovery once the xbot is ready or its recovery deadline expires."""
            error = coordinator.check_xbot(xid)
            if error == ErrorID.NONE or coordinator.clock() > deadline:
                coordinator.on_tick.remove(check)
                self._answer(error, done)
        coordinator.on_tick.append(check)

    # --- equipment commands (OPC UA, any thread) -----------------------------------------

    def set_payload(self, xid, session, kg, cg_height_mm):
        """The shuttle's occupant tells the PMC the xbot's payload (kg, centre of gravity mm above the
        xbot); not while its skill runs.
        """
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
        """Authorize and set xbot or table (xid=None) limits in m/s and m/s^2; zero or NaN restores
        defaults.
        """
        module = self.table if xid is None else self.shuttles[xid]
        with self.lock:
            if not module.occupation.allows(session):
                return refuse(ErrorID.NOT_PERMITTED)
            error = self.coordinator.set_speed(xid, speed, accel)
        return OK if error == ErrorID.NONE else refuse(error)

    def zone_command(self, name, operation, session):
        """Deactivate or Activate a zone: whoever may command the table (its occupant, or anyone while it
        is not occupied).
        """
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
        """Prevent displacement of unavailable or occupied idle shuttles unless displace_held permits it."""
        shuttle = self.shuttles.get(xid)
        if shuttle is None:
            return True  # not offered as a shuttle: nobody here may move it
        moving = any(s.active for s in shuttle.skills.values())
        usable = shuttle.state in (ModuleState.IDLE, ModuleState.EXECUTE)  # (Held: it stays put too)
        return not usable or (shuttle.occupation.occupied and not moving and not self.displace_held)
