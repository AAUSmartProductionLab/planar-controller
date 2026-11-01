"""PmcDriver against a stand-in for pmclib (the PMC is not needed)."""

import importlib.util
import math
import sys
import types
from pathlib import Path

import pytest


class _Member(str):
    @property
    def name(self):
        return str(self)


class _Names:
    """An enum of pmclib's: each member is its own name."""

    def __getattr__(self, name):
        return _Member(name)


@pytest.fixture
def pmc(monkeypatch):
    """The PMC as PmcDriver sees it: one xbot at state["rz"]; its commands in state["sent"]."""
    state = {"rz": 0.0, "sent": []}
    pm = types.ModuleType("pmclib.pmc_types")
    for enum in ("XBOTSTATE", "PMCSTATUS", "ROTATIONMODE", "POSITIONMODE", "LINEARPATHTYPE", "LEVITATEOPTIONS"):
        setattr(pm, enum, _Names())
    pm.ALLXBOTSFEEDBACKOPTION = lambda value: value
    xbot = types.ModuleType("pmclib.xbot_commands")
    xbot.get_all_xbot_info = lambda option: [types.SimpleNamespace(
        xbot_id=1, x_pos=0.12, y_pos=0.36, rz_pos=state["rz"], xbot_state=_Member("XBOT_IDLE"))]
    xbot.rotary_motion_p2p = lambda *args: state["sent"].append(args) or 0.5
    package = types.ModuleType("pmclib")
    package.pmc_types, package.xbot_commands = pm, xbot
    package.system_commands = types.ModuleType("pmclib.system_commands")
    for name, module in (("pmclib", package), ("pmclib.pmc_types", pm), ("pmclib.xbot_commands", xbot),
                         ("pmclib.system_commands", package.system_commands)):
        monkeypatch.setitem(sys.modules, name, module)
    path = Path(__file__).parent.parent / "planar" / "pmc_driver.py"
    spec = importlib.util.spec_from_file_location("planar._pmc_driver_under_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.PmcDriver(), state


@pytest.mark.parametrize("now, target, turn", [
    (-90, 180, -90),   # absolute, the PMC would turn +270
    (180, 90, -90),
    (0, -90, -90),
    (-179, 179, -2),
    (270, 0, 90),      # wherever the PMC's angle has got to
    (90, 180, 90),
])
def test_an_xbot_is_turned_the_shortest_way(pmc, now, target, turn):
    driver, state = pmc
    state["rz"] = math.radians(now)
    assert driver.rotate(1, math.radians(target)) == 0.5
    (label, xid, mode, rz, speed, accel, position_mode), = state["sent"]
    assert (xid, mode, position_mode) == (1, "NO_ANGLE_WRAP", "RELATIVE")
    assert math.degrees(rz) == pytest.approx(turn)


@pytest.mark.parametrize("pmc_deg, read_deg", [(-540, 180), (-450, -90), (-270, 90), (360, 0), (90, 90)])
def test_an_xbot_reads_its_angle_wrapped_though_the_pmc_does_not_wrap_it(pmc, pmc_deg, read_deg):
    driver, state = pmc
    state["rz"] = math.radians(pmc_deg)
    reading = driver.read()[1]
    assert (reading.x, reading.y, reading.state) == (0.12, 0.36, "idle")
    assert math.degrees(reading.rz) == pytest.approx(read_deg)
