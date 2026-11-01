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


def _load(monkeypatch, state, enums=("XBOTSTATE", "PMCSTATUS", "ROTATIONMODE", "POSITIONMODE", "LINEARPATHTYPE",
                                      "LEVITATEOPTIONS", "MOTIONINTERRUPTOPTIONS", "MOVERPROPERTY")):
    """planar/pmc_driver.py on a stand-in pmclib with these enums."""
    pm = types.ModuleType("pmclib.pmc_types")
    for enum in enums:
        setattr(pm, enum, _Names())
    pm.ALLXBOTSFEEDBACKOPTION = lambda value: value
    pm.XBotPayloadSettings = lambda *values: values
    xbot = types.ModuleType("pmclib.xbot_commands")
    xbot.get_all_xbot_info = lambda option: [types.SimpleNamespace(
        xbot_id=1, x_pos=0.12, y_pos=0.36, rz_pos=state["rz"], xbot_state=_Member(state.get("xbot", "XBOT_IDLE")))]
    xbot.get_xbot_ids = lambda: types.SimpleNamespace(xbot_count=2, xbot_ids_array=[1, 2])
    xbot.get_all_accident_xbots = lambda: types.SimpleNamespace(
        xbot_count=len(state.get("accidents", [])), xbot_ids_array=list(state.get("accidents", [])) + [0])
    xbot.rotary_motion_p2p = lambda *args: state["sent"].append(args) or 0.5
    xbot.linear_motion_si = lambda *args: state["sent"].append(("move", *args)) or 1.0
    xbot.motion_interrupt_command = lambda xid, option: state["sent"].append((option, xid))

    def write_payload(xid, settings):
        if settings[0] > 0.5:
            raise RuntimeError("write_xbot_payload_settings command failed with code INVALIDPARAMS(INVALIDPARAMS)")
        state["payload"] = settings
    xbot.write_xbot_payload_settings = write_payload
    xbot.read_xbot_payload_settings = lambda xid: types.SimpleNamespace(
        payloadkg=state.get("payload", (0.0,))[0], payload_cg_heightm=0.0, payload_dimension_xm=0.12,
        payload_dimension_ym=0.12)
    xbot.get_mover_properties = lambda xid, start: types.SimpleNamespace(
        valid_properties_count=1, mover_properties=[types.SimpleNamespace(
            property_id=_Member("MAXACCELERATION_7"), property_value=state.get("max_accel", 20.0))])
    package = types.ModuleType("pmclib")
    package.pmc_types, package.xbot_commands = pm, xbot
    package.system_commands = types.ModuleType("pmclib.system_commands")
    package.system_commands.is_master = lambda: state.get("master", True)
    for name, module in (("pmclib", package), ("pmclib.pmc_types", pm), ("pmclib.xbot_commands", xbot),
                         ("pmclib.system_commands", package.system_commands)):
        monkeypatch.setitem(sys.modules, name, module)
    path = Path(__file__).parent.parent / "planar" / "pmc_driver.py"
    spec = importlib.util.spec_from_file_location("planar._pmc_driver_under_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _module(driver):
    """The driver's module (loaded from its file: not in sys.modules)."""
    return types.SimpleNamespace(**type(driver).read.__globals__)


@pytest.fixture
def pmc(monkeypatch):
    """The PMC as PmcDriver sees it: one xbot at state["rz"]; its commands in state["sent"]."""
    state = {"rz": 0.0, "sent": []}
    return _load(monkeypatch, state).PmcDriver(), state


def test_a_pmclib_the_driver_is_not_written_for_says_which_to_install(monkeypatch):
    with pytest.raises(ImportError, match="pip install dependencies/pmclib-117.9.1"):
        _load(monkeypatch, {}, enums=("XbotState", "PMCSTATUS"))


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


def test_another_program_taking_mastership_is_noticed_within_a_second(pmc):
    driver, state = pmc
    module = _module(driver)
    driver.read()
    state["master"] = False
    driver._master_checked = 0.0  # a second on
    with pytest.raises(module.DriverError) as e:
        driver.read()
    assert e.value.code == module.ErrorID.MASTERSHIP_LOST


def test_an_xbot_the_pmc_switched_off_after_an_accident_reads_so(pmc):
    driver, state = pmc
    state["xbot"], state["accidents"] = "XBOT_DISABLED", [1]
    reading = driver.read()[1]
    assert (reading.state, reading.detail) == ("off", "ACCIDENT (XBOT_DISABLED)")


def test_a_hold_pauses_every_xbot_and_unhold_resumes_them(pmc):
    driver, state = pmc
    driver.pause(0)
    driver.resume(2)
    assert state["sent"] == [("PAUSE", 1), ("PAUSE", 2), ("RESUME", 2)]


def test_a_payload_the_xbot_does_not_take_is_out_of_range_and_one_it_does_slows_its_moves(pmc):
    driver, state = pmc
    module = _module(driver)
    with pytest.raises(module.DriverError) as e:
        driver.set_payload(1, 0.8, 0.02)
    assert e.value.code == module.ErrorID.OUT_OF_RANGE
    state["max_accel"] = 1.2
    assert driver.set_payload(1, 0.4, 0.02) == 1.2
    assert state["payload"] == (0.4, 0.02, 0.12, 0.12), "its size left as the PMC has it"
    driver.move(1, 0.36, 0.36)
    assert state["sent"][-1][9] == 1.2, "its moves keep to the acceleration the PMC allows with the payload"
    driver.move(2, 0.36, 0.36)
    assert state["sent"][-1][9] == driver.accel
