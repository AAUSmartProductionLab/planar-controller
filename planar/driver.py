"""Define the serialized driver interface and feedback types; units are metres, radians and seconds."""

from dataclasses import dataclass

from .statemachines import ErrorID


@dataclass
class Reading:
    """One xbot as the driver reads it. state: idle, moving, fault or off; detail: the PMC's word for it."""
    x: float
    y: float
    rz: float
    state: str
    detail: str = ""


class DriverError(Exception):
    """A driver failure with the controller ErrorID used by tasks and modules."""
    def __init__(self, message, code=ErrorID.PMC_ERROR):
        """Store the failure message and normalize its controller error code."""
        super().__init__(message)
        self.code = ErrorID(code)


class Driver:
    """Interface for PMC connection, motion, feedback and optional equipment features. Implementations
    serialize calls across threads.
    """
    supports_blended_moves = False

    def move_blended(self, xid, path, speed, accel) -> float:
        """A RoundedPath, fixed orientation, ending at rest; estimated duration in seconds."""
        raise DriverError("this driver has no blended XY motion")

    def connect(self):
        """Connect, take mastership, recover and levitate xbots, and return the Layout. Raise
        MASTERSHIP_LOST if control is unavailable.
        """
        raise NotImplementedError

    def read(self) -> dict:
        """{xid: Reading}; a DriverError MASTERSHIP_LOST once another program has taken mastership."""
        raise NotImplementedError

    def move(self, xid, x, y, speed, accel) -> float:
        """A straight move to (x, y), at most speed m/s and accel m/s^2; the seconds it should take."""
        raise NotImplementedError

    def rotate(self, xid, rz) -> float:
        """Turn to rz the shortest way (by at most half a turn); the seconds it should take."""
        raise NotImplementedError

    def stop(self, xid=0):
        """Stop the xbot at once (0: every xbot)."""
        raise NotImplementedError

    def auto_drive(self, xids, targets, speed, accel):
        """Route these xbots with the PMC's own auto routing, each to one of the targets ((x, y), any to
        any), at most speed and accel; the xbots read as moving until they are there.
        """
        raise DriverError("this PMC has no auto routing")

    def close(self):
        """Disconnect from the PMC (the controller is stopping)."""

    def pause(self, xid):
        """Pause the xbot's motion where it is (it reads as idle meanwhile) until resume()."""
        raise DriverError("this PMC cannot pause a motion")

    def resume(self, xid):
        """Carry on with the motion paused."""
        raise DriverError("this PMC cannot pause a motion")

    def payload(self, xid):
        """(kg, centre of gravity height m, max acceleration m/s^2) as the PMC has them; None: unknown."""
        return None

    def set_payload(self, xid, kg, cg_height):
        """Set payload kg and centre-of-gravity height m; return max acceleration in m/s^2 or None. Raise
        OUT_OF_RANGE for unsupported payloads.
        """
        raise DriverError("this PMC does not take payloads")

    def recover(self, xids):
        """Recover these xbots if the PMC switched them off after an accident; the ones it did."""
        return []

    def define_zone(self, zid, box, speed):
        """Define (or redefine) zone zid, the box (x0, y0, x1, y1), its xbots' speed scaled by speed."""
        raise DriverError("this PMC has no zones", ErrorID.ZONES_INVALID)

    def zone_status(self, zid):
        """(state, xids inside): the PMC's ZONESTATE name, e.g. ACTIVATED_UNFENCED, LOADING_FENCED."""
        raise DriverError("this PMC has no zones")

    def zone_control(self, zid, operation):
        """deactivate (fence it, the xbots inside off), activate (levitate them) or unfence."""
        raise DriverError("this PMC has no zones")

    def flyways(self):
        """{(column, row): (power W, CPU, amplifier, motor temperature deg C)}."""
        return {}
