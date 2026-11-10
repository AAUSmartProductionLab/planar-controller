"""The table and its shuttles on the simulated driver, for the tests: time is theirs (run())."""

from planar.layout import Layout
from planar.modules import PlanarController
from planar.sim_driver import SimDriver
from planar.statemachines import ErrorID, ModuleState

STRIP = Layout(cols=4, rows=1)  # 8 x 2 slots
SIM = Layout(cols=4, rows=3, holes=frozenset({(2, 1)}))  # the simulation tool's table


def c(i, j):
    """Return a routing-cell centre in millimetres for skill parameters."""
    return ((i + 0.5) * 120.0, (j + 0.5) * 120.0)  # slot (i, j)'s centre in mm, as the skill takes them


class Cell:
    """PlanarController on SimDriver; xbots {xid: (x mm, y mm)}, turned 0."""

    def __init__(self, xbots, layout=STRIP, **options):
        """Create the simulated controller from millimetre poses with a manually advanced clock."""
        self.t = 0.0
        self.sim = SimDriver(layout, {xid: (x / 1000, y / 1000, 0.0) for xid, (x, y) in xbots.items()})
        self.ctl = PlanarController(self.sim, sorted(xbots), clock=lambda: self.t, plan_timeout=0.3, **options)
        self.coord = self.ctl.coordinator
        self.table = self.ctl.table
        self.s = self.ctl.shuttles
        self.watch = None  # called after every step

    def run(self, seconds, until=None):
        """Step time on for seconds; with until, only until it says so (True then)."""
        for _ in range(int(seconds / 0.02)):
            self.t += 0.02
            self.sim.advance(self.t)
            self.coord.tick(self.t)
            if self.watch:
                self.watch()
            if until is not None and until():
                return True
        return until is None

    def up(self):
        """Reset and Start the table, and with it every shuttle."""
        assert self.table.command("Reset", "") == (True, ErrorID.NONE)
        assert self.run(2, lambda: self.table.state == ModuleState.IDLE)
        assert all(s.state == ModuleState.IDLE for s in self.s.values())
        assert self.table.command("Start", "") == (True, ErrorID.NONE)
        assert all(s.state == ModuleState.EXECUTE for s in self.s.values())

    def skill(self, xid):
        """Return the selected shuttle's MoveToPosition skill."""
        return self.s[xid].skills["MoveToPosition"]

    def move(self, xid, session, x, y, rz=0.0):
        """Start a position skill using X/Y millimetres and Rz degrees."""
        return self.skill(xid).start(session, {"X": x, "Y": y, "Rz": rz})

    def x(self, xid):
        """Return the simulated xbot's X coordinate in metres."""
        return self.sim.xbots[xid].x
