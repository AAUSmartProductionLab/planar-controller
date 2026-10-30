"""
The planar table as modules, built like the generated IEC 61499 modules:

- PlanarTable: the table as a whole. Resetting connects to the PMC, takes mastership, levitates
  the xbots and, with a loading station, sends the shuttles into its queue with the PMC's auto
  routing; Abort stops every xbot at once. Its Stop and Abort reach every shuttle,
  its Reset, Start and Clear the shuttles in the matching state, so one Reset and one Start of
  the table bring everything up.
- PlanarShuttle<N>: one per xbot. An orchestrator occupies a shuttle, runs its MoveToPosition
  skill (X, Y in mm, Rz in degrees) as often as it likes, and releases it. Start answers at once;
  the skill's State turns Succeeded when the xbot has arrived (Failed with an ErrorID if it
  cannot). Resetting checks that the xbot is there and fit to move; Abort stops it at once.

Whether the planner may push an xbot out of another's way follows from its shuttle: an occupied
shuttle that is not moving stays where it is (a station may be working on it), and so does one
whose module is not in Idle or Execute. With displace_held, occupied shuttles that are not
moving may be pushed too (never one whose module is stopped or aborted).
"""

import logging
import math

from .coordinator import Coordinator, Pose
from .statemachines import ErrorID, Module, ModuleState, Parameter, Procedure, SkillBody, make_lock

log = logging.getLogger(__name__)


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

        self.table = Module(table_name, self.lock, clock,
                            resetting=Procedure(self._connect),
                            stopping=Procedure(self._stop),
                            aborting=self.coordinator.abort_all)
        self.shuttles = {}
        for xid in self.shuttle_ids:
            shuttle = Module(shuttle_name.format(xid), self.lock, clock,
                             resetting=Procedure(lambda done, xid=xid: self._check_xbot(xid, done)),
                             aborting=lambda xid=xid: self.coordinator.abort(xid),
                             parent=self.table)
            shuttle.add_skill("MoveToPosition", MoveToPosition.PARAMETERS, MoveToPosition.RESULTS,
                              MoveToPosition(self.coordinator, xid))
            self.shuttles[xid] = shuttle

        self.coordinator.on_tick.append(self.table.poll)
        self.coordinator.held_by = self._held
        self.coordinator.on_connection_lost = lambda: self.table.fail(ErrorID.PMC_NOT_CONNECTED)

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

    def _check_xbot(self, xid, done):
        error = self.coordinator.check_xbot(xid)
        done(error == ErrorID.NONE, error)

    # --- asked by the coordinator, with the lock held ------------------------------------

    def _held(self, xid):
        """Whether the planner must leave this xbot where it is. Asked at the moment it plans, so a
        shuttle whose move has just ended is held before anything can push it."""
        shuttle = self.shuttles.get(xid)
        if shuttle is None:
            return True  # not offered as a shuttle: nobody here may move it
        moving = any(s.active for s in shuttle.skills.values())
        usable = shuttle.state in (ModuleState.IDLE, ModuleState.EXECUTE)
        return not usable or (shuttle.occupation.occupied and not moving and not self.displace_held)
