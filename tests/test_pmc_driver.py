"""PmcDriver against a stand-in for pmclib (the PMC is not needed)."""

import importlib.util
import math
import sys
import types
from enum import IntEnum
from pathlib import Path

import pytest


class _Member(str):
    """A string-valued stand-in for a vendor enum member."""
    @property
    def name(self):
        """Return the enum member's textual name."""
        return str(self)


class _Names:
    """An enum of pmclib's: each member is its own name."""

    def __getattr__(self, name):
        """Create a named enum stand-in on demand."""
        return _Member(name)


def _load(monkeypatch, state, enums=("XBOTSTATE", "PMCSTATUS", "ROTATIONMODE", "POSITIONMODE", "LINEARPATHTYPE",
                                      "LEVITATEOPTIONS", "MOTIONINTERRUPTOPTIONS", "MOTIONBUFFEROPTIONS",
                                      "ARCMODE", "ARCTYPE", "ARCDIRECTION")):
    """planar/pmc_driver.py on a stand-in pmclib with these enums."""
    pm = types.ModuleType("pmclib.pmc_types")
    for enum in enums:
        setattr(pm, enum, _Names())
    pm.MOVERPROPERTY = IntEnum("MOVERPROPERTY", {"MAXACCELERATION_7": 7})
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
    xbot.arc_motion_meters_radians = lambda *args: state['sent'].append(('arc', *args)) or 0.5
    xbot.motion_buffer_control = lambda xid, option: state['sent'].append(('buffer', xid, option)) or types.SimpleNamespace(
        is_buffer_blocked=True, buffered_motion_count=0)
    xbot.stop_motion = lambda xid: state['sent'].append(('stop', xid))
    xbot.motion_interrupt_command = lambda xid, option: state["sent"].append((option, xid))

    def write_payload(xid, settings):
        """Emulate payload acceptance and reject masses above the test limit."""
        if settings[0] > 0.5:
            raise RuntimeError("write_xbot_payload_settings command failed with code INVALIDPARAMS(INVALIDPARAMS)")
        state["payload"] = settings
    xbot.write_xbot_payload_settings = write_payload
    xbot.read_xbot_payload_settings = lambda xid: types.SimpleNamespace(
        payloadkg=state.get("payload", (0.0,))[0], payload_cg_heightm=0.0, payload_dimension_xm=0.12,
        payload_dimension_ym=0.12)
    def mover_properties(xid, start):
        """Emulate a property read and enforce pmclib's plain-integer property ID requirement."""
        if type(start) is not int:  # as pmclib asserts: an int, not one of its IntEnums
            raise TypeError(f"starting_property_id expected {int}, got {type(start)}")
        return types.SimpleNamespace(valid_properties_count=1, mover_properties=[types.SimpleNamespace(
            property_id=pm.MOVERPROPERTY(start), property_value=state.get("max_accel", 20.0))])
    xbot.get_mover_properties = mover_properties
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


@pytest.mark.parametrize('corner,direction', [((0.36, 0.36), 'COUNTERCLOCKWISE'), ((0.12, 0.60), 'CLOCKWISE')])
def test_blended_command_preloads_nonzero_transitions_and_finishes_at_rest(pmc, corner, direction):
    """Verify a full line-arc-line batch is loaded before release and ends at rest."""
    from planar.rounded_path import RoundedPath
    driver, state = pmc
    path = RoundedPath((0.12, 0.36), corner, (0.36, 0.60), 0.03)
    assert driver.move_blended(1, path, 0.1, 2.0) == 2.5
    sent = state['sent']
    assert [s[0] for s in sent] == ['buffer', 'move', 'arc', 'move', 'buffer']
    assert sent[0] == ('buffer', 1, 'BLOCKBUFFER')
    assert sent[-1] == ('buffer', 1, 'RELEASEBUFFER')
    assert sent[1][3:] == ('ABSOLUTE', 'DIRECT', *path.tangents[0], 0.1, 0.1, 2.0, 0.0)
    assert sent[2][3:] == ('TARGETRADIUS', 'MINORARC', direction, 'ABSOLUTE', *path.tangents[1], 0.1, 0.1, 2.0, 0.03, 0.0)
    assert sent[3][3:] == ('ABSOLUTE', 'DIRECT', *path.end, 0.0, 0.1, 2.0, 0.0)


@pytest.mark.parametrize('exception', [RuntimeError, KeyboardInterrupt])
def test_blended_partial_batch_is_stopped_and_cleared_before_release(pmc, monkeypatch, exception):
    """Verify partial batches are stopped and cleared before unblocking."""
    from planar.rounded_path import RoundedPath
    driver, state = pmc
    commands = _module(driver).xbot_cmd
    def fail(*args):
        """Interrupt arc loading with the configured exception type."""
        raise exception('arc rejected')
    monkeypatch.setattr(commands, 'arc_motion_meters_radians', fail)
    with pytest.raises((Exception, KeyboardInterrupt), match='arc rejected'):
        driver.move_blended(1, RoundedPath((0.12, 0.36), (0.36, 0.36), (0.36, 0.60), 0.03), 0.1, 2.0)
    assert state['sent'][-3:] == [('stop', 1), ('buffer', 1, 'CLEARBUFFER'), ('buffer', 1, 'RELEASEBUFFER')]


def test_blended_limits_corner_speed_without_slowing_entire_straights(pmc):
    """Verify corner-speed limits do not reduce the maximum speed of straight legs."""
    from planar.rounded_path import RoundedPath
    driver, state = pmc
    driver.move_blended(1, RoundedPath((0.12, 0.36), (0.36, 0.36), (0.36, 0.60), 0.03), 0.5, 2.0)
    line_in, arc, line_out = state['sent'][1:4]
    assert line_in[7] == pytest.approx(math.sqrt(2.0*0.03))
    assert line_in[8] == 0.5
    assert arc[9] == arc[10] == line_in[7]
    assert line_out[7:9] == (0.0, 0.5)


def test_blended_failed_cleanup_never_releases_partial_batch(pmc, monkeypatch):
    """Verify failed cleanup never releases an incomplete batch."""
    from planar.rounded_path import RoundedPath
    driver, state = pmc
    commands = _module(driver).xbot_cmd
    def fail(*args):
        """Emulate lost communication during loading or cleanup."""
        raise RuntimeError('communication lost')
    monkeypatch.setattr(commands, 'arc_motion_meters_radians', fail)
    monkeypatch.setattr(commands, 'stop_motion', fail)
    with pytest.raises(Exception, match='communication lost'):
        driver.move_blended(1, RoundedPath((0.12, 0.36), (0.36, 0.36), (0.36, 0.60), 0.03), 0.1, 2.0)
    assert not any(s == ('buffer', 1, 'RELEASEBUFFER') for s in state['sent'])


@pytest.fixture
def pmc(monkeypatch):
    """The PMC as PmcDriver sees it: one xbot at state["rz"]; its commands in state["sent"]."""
    state = {"rz": 0.0, "sent": []}
    return _load(monkeypatch, state).PmcDriver(), state


def test_a_pmclib_the_driver_is_not_written_for_says_which_to_install(monkeypatch):
    """Verify unsupported pmclib versions report the required installation."""
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
    """Verify rotation uses the shortest angular displacement."""
    driver, state = pmc
    state["rz"] = math.radians(now)
    assert driver.rotate(1, math.radians(target)) == 0.5
    (label, xid, mode, rz, speed, accel, position_mode), = state["sent"]
    assert (xid, mode, position_mode) == (1, "NO_ANGLE_WRAP", "RELATIVE")
    assert math.degrees(rz) == pytest.approx(turn)


@pytest.mark.parametrize("pmc_deg, read_deg", [(-540, 180), (-450, -90), (-270, 90), (360, 0), (90, 90)])
def test_an_xbot_reads_its_angle_wrapped_though_the_pmc_does_not_wrap_it(pmc, pmc_deg, read_deg):
    """Verify feedback angles are wrapped even when PMC angles are not."""
    driver, state = pmc
    state["rz"] = math.radians(pmc_deg)
    reading = driver.read()[1]
    assert (reading.x, reading.y, reading.state) == (0.12, 0.36, "idle")
    assert math.degrees(reading.rz) == pytest.approx(read_deg)


def test_another_program_taking_mastership_is_noticed_within_a_second(pmc):
    """Verify mastership loss is detected on the next scheduled check."""
    driver, state = pmc
    module = _module(driver)
    driver.read()
    state["master"] = False
    driver._master_checked = 0.0  # a second on
    with pytest.raises(module.DriverError) as e:
        driver.read()
    assert e.value.code == module.ErrorID.MASTERSHIP_LOST


def test_an_xbot_the_pmc_switched_off_after_an_accident_reads_so(pmc):
    """Verify accident-disabled movers carry accident information in feedback."""
    driver, state = pmc
    state["xbot"], state["accidents"] = "XBOT_DISABLED", [1]
    reading = driver.read()[1]
    assert (reading.state, reading.detail) == ("off", "ACCIDENT (XBOT_DISABLED)")


def test_an_xbots_motion_is_paused_and_resumed(pmc):
    """Verify pause and resume map to the vendor motion-interrupt commands."""
    driver, state = pmc
    driver.pause(2)
    driver.resume(2)
    assert state["sent"] == [("PAUSE", 2), ("RESUME", 2)]


def test_a_payload_the_xbot_does_not_take_is_out_of_range_and_one_it_does_gives_its_max_acceleration(pmc):
    """Verify rejected payloads map to OUT_OF_RANGE and valid payloads return acceleration limits."""
    driver, state = pmc
    module = _module(driver)
    with pytest.raises(module.DriverError) as e:
        driver.set_payload(1, 0.8, 0.02)
    assert e.value.code == module.ErrorID.OUT_OF_RANGE
    state["max_accel"] = 1.2
    assert driver.set_payload(1, 0.4, 0.02) == 1.2
    assert state["payload"] == (0.4, 0.02, 0.12, 0.12), "its size left as the PMC has it"


def test_a_move_is_sent_with_the_speed_and_acceleration_it_is_given(pmc):
    """Verify straight moves preserve the requested speed and acceleration."""
    driver, state = pmc
    assert driver.move(1, 0.36, 0.48, 0.3, 1.2) == 1.0
    (_, _, xid, _, _, x, y, final_speed, speed, accel, _), = state["sent"]
    assert (xid, x, y, final_speed, speed, accel) == (1, 0.36, 0.48, 0.0, 0.3, 1.2)
