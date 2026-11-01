"""
The coordinator's driver for a real PMC (or the Planar Motor simulation tool), through pmclib.

pmclib is a thin layer over one shared .NET session with no synchronisation of its own, so every
call goes through one lock.

The session is closed (close()) when the controller stops: the simulator, at least, otherwise
keeps reading the dead connection and writes an error to its trace log (PMTraceLog.txt) for
every read, gigabytes an hour, until it takes no new connections.
"""

import logging
import os
import tempfile
import threading
import time

from pmclib import pmc_types as pm
from pmclib import system_commands as sys_cmd
from pmclib import xbot_commands as xbot_cmd

if not hasattr(pm, "XBOTSTATE"):  # older pmclibs name their enums otherwise (XbotState)
    try:
        from importlib.metadata import version
        _have = f"pmclib {version('pmclib')}"
    except Exception:
        _have = "this pmclib"
    raise ImportError(f"{_have} is not the one the controller is written for: "
                      "pip install dependencies/pmclib-117.9.1-py3-none-any.whl")

from .coordinator import Driver, DriverError, Reading
from .layout import parse_pmc_config, pmc_flyway_ids, wrap_angle
from .statemachines import ErrorID

log = logging.getLogger(__name__)

MASTERSHIP_CHECK_S = 1.0  # how often read() asks the PMC whether this is still its master
ACCIDENT_CHECK_S = 1.0    # how often, while an xbot reads as off or in fault, whether it had an accident
_ZONE_OPERATIONS = {
    "deactivate": lambda zid: sys_cmd.zone_control(zid, pm.ZONEOPERATION.DEACTIVATE_ZONE),
    "activate": lambda zid: sys_cmd.zone_control(zid, pm.ZONEOPERATION.ACTIVATE_ZONE),
    "unfence": lambda zid: sys_cmd.zone_fence_control(zid, pm.FENCEOPERATION.REMOVE_FENCE),
}

_IDLE = {pm.XBOTSTATE.XBOT_IDLE}
# XBOT_RESERVED1: driven by the PMC's auto routing
_MOVING = {pm.XBOTSTATE.XBOT_MOTION, pm.XBOTSTATE.XBOT_WAIT, pm.XBOTSTATE.XBOT_STOPPING,
           pm.XBOTSTATE.XBOT_HOLDPOSITION, pm.XBOTSTATE.XBOT_RESERVED1}
_FAULT = {pm.XBOTSTATE.XBOT_ERROR, pm.XBOTSTATE.XBOT_OBSTACLE_DETECTED, pm.XBOTSTATE.XBOT_STOPPED}
_READY_PMC = {pm.PMCSTATUS.PMC_FULLCTRL, pm.PMCSTATUS.PMC_INTELLIGENTCTRL}


class PmcDriver(Driver):
    def __init__(self, ip=None, *, speed=0.5, accel=2.0, turn_speed=3.0, turn_accel=10.0, jerk=None,
                 activate_timeout_s=30.0, config_path=None):
        self.ip = ip
        self.speed, self.accel = speed, accel
        self.turn_speed, self.turn_accel = turn_speed, turn_accel
        self.jerk = jerk  # m/s^3 in X and Y, set on every xbot at connect; None: the PMC's
        self.activate_timeout_s = activate_timeout_s
        self.config_path = config_path or os.path.join(tempfile.gettempdir(), "pmc_config.xml")
        self._lock = threading.RLock()
        self._label = 0
        self._connected = False
        self._master_checked = 0.0
        self._accidents, self._accidents_checked = set(), 0.0
        self._max_accel = {}   # xid: m/s^2 with its payload
        self._flyway_ids = {}  # (column, row): the PMC's flyway id
        self._zone_ids = set()

    @staticmethod
    def _error(e, code=ErrorID.PMC_ERROR):
        if isinstance(e, DriverError):
            return e
        if "INVALIDPARAMS" in str(e) and code == ErrorID.PMC_ERROR:  # e.g. a payload over what the xbot takes
            code = ErrorID.OUT_OF_RANGE
        return DriverError(str(e), code)

    def _next_label(self):
        self._label = self._label % 65535 + 1
        return self._label

    def connect(self):
        with self._lock:
            try:
                self._ensure_connected()
                try:
                    sys_cmd.gain_mastership()
                except Exception as e:
                    raise DriverError(f"could not take mastership of the PMC ({e}); is PMT or another controller "
                                      f"its master?", ErrorID.MASTERSHIP_LOST) from e
                if not sys_cmd.is_master():
                    raise DriverError("could not take mastership of the PMC: is PMT or another controller its master?",
                                      ErrorID.MASTERSHIP_LOST)
                self._master_checked = time.monotonic()
                if sys_cmd.get_pmc_status() not in _READY_PMC:
                    xbot_cmd.activate_xbots()
                deadline = time.monotonic() + self.activate_timeout_s
                while sys_cmd.get_pmc_status() not in _READY_PMC:
                    if time.monotonic() > deadline:
                        raise DriverError(f"PMC not ready after {self.activate_timeout_s:.0f} s "
                                          f"({sys_cmd.get_pmc_status().name})")
                    time.sleep(0.2)
                recovered = self.recover(self._ids())
                if recovered:
                    log.info("xbots %s recovered after an accident", recovered)
                self._wait_levitated(deadline)
                for xid in self._ids():
                    try:  # paused by a Hold of a controller that stopped since: the motions sent would wait
                        xbot_cmd.motion_interrupt_command(int(xid), pm.MOTIONINTERRUPTOPTIONS.RESUME)
                    except Exception as e:
                        log.debug("xbot %d: resume: %s", xid, e)
                    if self.jerk:
                        xbot_cmd.set_jerk_limit(int(xid), pm.JERKLIMAXIS.X, float(self.jerk))
                sys_cmd.save_pmc_config_xml_file(self.config_path)
                self._flyway_ids = pmc_flyway_ids(self.config_path)
                return parse_pmc_config(self.config_path)
            except DriverError:
                raise
            except Exception as e:
                raise DriverError(str(e)) from e

    def close(self):
        with self._lock:
            if not self._connected:
                return
            self._connected = False
            try:
                sys_cmd.disconnect_from_pmc()
                log.info("disconnected from the PMC")
            except Exception as e:
                raise DriverError(str(e)) from e

    def _ensure_connected(self):
        """Keep a live connection: the PMC (the simulator at least) refuses one made again at once
        after a disconnect from the same process."""
        if self._connected and sys_cmd.check_tcp_connection():
            return
        ok = sys_cmd.connect_to_specific_pmc(self.ip) if self.ip else sys_cmd.auto_search_and_connect_to_pmc()
        if not ok:
            self._connected = False
            raise DriverError(f"no PMC at {self.ip or '(auto search)'}", ErrorID.PMC_NOT_CONNECTED)
        self._connected = True

    def _wait_levitated(self, deadline):
        ids = self._ids()
        for zid in self._zone_ids:  # not the ones in a zone switched off: they stay off
            status = sys_cmd.get_zone_status(zid)
            if status.zone_state != pm.ZONESTATE.ACTIVATED_UNFENCED:
                ids = [x for x in ids if x not in set(status.xbot_ids[:status.xbot_count])]
        while True:
            states = {i.xbot_id: i.xbot_state for i in self._all()}
            waiting = [x for x in ids if states.get(x) not in _IDLE | _MOVING | _FAULT]
            if not waiting:
                return
            if time.monotonic() > deadline:
                log.warning("xbots %s not levitated: %s", waiting, {x: states.get(x) for x in waiting})
                return
            xbot_cmd.levitation_command(0, pm.LEVITATEOPTIONS.LEVITATE)
            time.sleep(0.5)

    def _ids(self):
        rtn = xbot_cmd.get_xbot_ids()
        return list(rtn.xbot_ids_array)[:rtn.xbot_count]

    def _all(self):
        return xbot_cmd.get_all_xbot_info(pm.ALLXBOTSFEEDBACKOPTION(0))

    def detect(self):
        """The xbot ids the PMC knows, without activating anything."""
        with self._lock:
            try:
                self._ensure_connected()
                return self._ids()
            except DriverError:
                raise
            except Exception as e:
                raise DriverError(str(e)) from e

    def read(self):
        with self._lock:
            try:
                infos = self._all()
                now = time.monotonic()
                if now - self._master_checked >= MASTERSHIP_CHECK_S:
                    self._master_checked = now
                    if not sys_cmd.is_master():
                        raise DriverError("another program has taken mastership of the PMC (PMT, or another "
                                          "controller); Reset the table to take it back", ErrorID.MASTERSHIP_LOST)
                troubled = [i for i in infos if i.xbot_state not in _IDLE | _MOVING]
                if not troubled:
                    self._accidents = set()
                elif now - self._accidents_checked >= ACCIDENT_CHECK_S:
                    self._accidents_checked = now
                    rtn = xbot_cmd.get_all_accident_xbots()
                    self._accidents = set(rtn.xbot_ids_array[:rtn.xbot_count])
            except DriverError:
                raise
            except Exception as e:
                raise DriverError(str(e), ErrorID.PMC_NOT_CONNECTED) from e
        out = {}
        for i in infos:
            state = i.xbot_state
            if state in _IDLE:
                kind = "idle"
            elif state in _MOVING:
                kind = "moving"
            elif state in _FAULT:
                kind = "fault"
            else:
                kind = "off"
            detail = state.name if int(i.xbot_id) not in self._accidents else f"ACCIDENT ({state.name})"
            # The PMC's angle is not wrapped: turned round twice clockwise, it reads -720 deg.
            out[int(i.xbot_id)] = Reading(float(i.x_pos), float(i.y_pos), wrap_angle(float(i.rz_pos)), kind, detail)
        return out

    def _accel(self, xid):
        """The acceleration its moves keep to: the one set, or less with a payload."""
        limit = self._max_accel.get(int(xid))
        return min(self.accel, limit) if limit else self.accel

    def move(self, xid, x, y):
        with self._lock:
            try:
                return float(xbot_cmd.linear_motion_si(
                    self._next_label(), int(xid), pm.POSITIONMODE.ABSOLUTE, pm.LINEARPATHTYPE.DIRECT,
                    float(x), float(y), 0.0, float(self.speed), float(self._accel(xid)), 0.0))
            except Exception as e:
                raise DriverError(str(e)) from e

    def rotate(self, xid, rz):
        """Turn by the difference from where the xbot is, the shortest way. To an absolute angle
        the PMC turns straight from its own angle, unwrapped: from -90 deg to 180 deg it turns
        +270 deg, the long way."""
        with self._lock:
            try:
                now = next(float(i.rz_pos) for i in self._all() if int(i.xbot_id) == int(xid))
                return float(xbot_cmd.rotary_motion_p2p(
                    self._next_label(), int(xid), pm.ROTATIONMODE.NO_ANGLE_WRAP, wrap_angle(float(rz) - now),
                    float(self.turn_speed), float(self.turn_accel), pm.POSITIONMODE.RELATIVE))
            except Exception as e:
                raise DriverError(str(e)) from e

    def stop(self, xid=0):
        with self._lock:
            try:
                xbot_cmd.stop_motion(int(xid))
            except Exception as e:
                raise DriverError(str(e)) from e

    def auto_drive(self, xids, targets):
        """The PMC's auto routing, unlabelled: each xbot to one of the targets, whichever."""
        with self._lock:
            try:
                xbot_cmd.auto_driving_motion_si_with_speed(
                    xbot_count=len(xids), mode=pm.ASYNCOPTIONS.MOVEALL_UNLABELED,
                    max_speed=float(self.speed), max_accel=float(min(self._accel(x) for x in xids)),
                    xbot_ids=[int(x) for x in xids],
                    targets_x_meters=[float(t[0]) for t in targets],
                    targets_y_meters=[float(t[1]) for t in targets])
            except Exception as e:
                raise DriverError(str(e)) from e

    def _each(self, xid):
        return [int(xid)] if xid else self._ids()

    def pause(self, xid=0):
        with self._lock:
            try:
                for x in self._each(xid):
                    xbot_cmd.motion_interrupt_command(x, pm.MOTIONINTERRUPTOPTIONS.PAUSE)
            except Exception as e:
                raise self._error(e) from e

    def resume(self, xid=0):
        with self._lock:
            try:
                for x in self._each(xid):
                    xbot_cmd.motion_interrupt_command(x, pm.MOTIONINTERRUPTOPTIONS.RESUME)
            except Exception as e:
                raise self._error(e) from e

    def payload(self, xid):
        with self._lock:
            try:
                settings = xbot_cmd.read_xbot_payload_settings(int(xid))
                max_accel = self._read_max_accel(xid)
            except Exception as e:
                raise self._error(e) from e
        return float(settings.payloadkg), float(settings.payload_cg_heightm), max_accel

    def _read_max_accel(self, xid):
        rtn = xbot_cmd.get_mover_properties(int(xid), pm.MOVERPROPERTY.MAXACCELERATION_7)
        values = [p.property_value for p in rtn.mover_properties[:rtn.valid_properties_count]
                  if p.property_id == pm.MOVERPROPERTY.MAXACCELERATION_7]
        max_accel = float(values[0]) if values and values[0] > 0 else None
        if max_accel is None:
            self._max_accel.pop(int(xid), None)
        else:
            self._max_accel[int(xid)] = max_accel
        return max_accel

    def set_payload(self, xid, kg, cg_height):
        """The payload's size is left as the PMC has it (at least the xbot's own)."""
        with self._lock:
            try:
                now = xbot_cmd.read_xbot_payload_settings(int(xid))
                xbot_cmd.write_xbot_payload_settings(int(xid), pm.XBotPayloadSettings(
                    float(kg), float(cg_height), float(now.payload_dimension_xm), float(now.payload_dimension_ym)))
                return self._read_max_accel(xid)
            except Exception as e:
                raise self._error(e) from e

    def recover(self, xids):
        with self._lock:
            try:
                rtn = xbot_cmd.get_all_accident_xbots()
                accidents = set(rtn.xbot_ids_array[:rtn.xbot_count]) & {int(x) for x in xids}
                for xid in sorted(accidents):
                    xbot_cmd.recover_accident_xbot(xid, pm.RECOVERXBOTMODE.STOP, pm.RECOVERXBOTOPTIONS.DEFAULT)
                self._accidents -= accidents
                return sorted(accidents)
            except Exception as e:
                raise self._error(e) from e

    def define_zone(self, zid, box, speed):
        with self._lock:
            try:
                sys_cmd.define_zone(int(zid), *(float(v) for v in box))
                sys_cmd.override_zone(int(zid), float(speed), float(speed))
                self._zone_ids.add(int(zid))
            except Exception as e:
                raise self._error(e, ErrorID.ZONES_INVALID) from e

    def zone_status(self, zid):
        with self._lock:
            try:
                status = sys_cmd.get_zone_status(int(zid))
            except Exception as e:
                raise self._error(e) from e
        return status.zone_state.name, [int(x) for x in status.xbot_ids[:status.xbot_count]]

    def zone_control(self, zid, operation):
        with self._lock:
            try:
                _ZONE_OPERATIONS[operation](int(zid))
            except Exception as e:
                raise self._error(e) from e

    def flyways(self):
        out = {}
        for flyway, fid in sorted(self._flyway_ids.items()):
            with self._lock:  # one at a time, so the moves go on between them
                try:
                    s = sys_cmd.get_flyway_physical_status(fid)
                except Exception as e:
                    raise self._error(e) from e
            out[flyway] = (float(s.power_consumption_w), float(s.cpu_temp_c), float(s.amplifier_temp_c),
                           float(s.motor_temp_c))
        return out
