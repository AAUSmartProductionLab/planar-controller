"""
The coordinator's driver for a real PMC (or the Planar Motor simulation tool), through pmclib.

pmclib is a thin layer over one shared .NET session with no synchronisation of its own, so every
call goes through one lock.
"""

import logging
import os
import tempfile
import threading
import time

from pmclib import pmc_types as pm
from pmclib import system_commands as sys_cmd
from pmclib import xbot_commands as xbot_cmd

from .coordinator import Driver, DriverError, Reading
from .layout import parse_pmc_config
from .statemachines import ErrorID

log = logging.getLogger(__name__)

_IDLE = {pm.XBOTSTATE.XBOT_IDLE}
# XBOT_RESERVED1: driven by the PMC's auto routing
_MOVING = {pm.XBOTSTATE.XBOT_MOTION, pm.XBOTSTATE.XBOT_WAIT, pm.XBOTSTATE.XBOT_STOPPING,
           pm.XBOTSTATE.XBOT_HOLDPOSITION, pm.XBOTSTATE.XBOT_RESERVED1}
_FAULT = {pm.XBOTSTATE.XBOT_ERROR, pm.XBOTSTATE.XBOT_OBSTACLE_DETECTED, pm.XBOTSTATE.XBOT_STOPPED}
_READY_PMC = {pm.PMCSTATUS.PMC_FULLCTRL, pm.PMCSTATUS.PMC_INTELLIGENTCTRL}


class PmcDriver(Driver):
    def __init__(self, ip=None, *, speed=0.5, accel=2.0, turn_speed=3.0, turn_accel=10.0,
                 activate_timeout_s=30.0, config_path=None):
        self.ip = ip
        self.speed, self.accel = speed, accel
        self.turn_speed, self.turn_accel = turn_speed, turn_accel
        self.activate_timeout_s = activate_timeout_s
        self.config_path = config_path or os.path.join(tempfile.gettempdir(), "pmc_config.xml")
        self._lock = threading.RLock()
        self._label = 0
        self._connected = False

    def _next_label(self):
        self._label = self._label % 65535 + 1
        return self._label

    def connect(self):
        with self._lock:
            try:
                self._ensure_connected()
                sys_cmd.gain_mastership()
                if sys_cmd.get_pmc_status() not in _READY_PMC:
                    xbot_cmd.activate_xbots()
                deadline = time.monotonic() + self.activate_timeout_s
                while sys_cmd.get_pmc_status() not in _READY_PMC:
                    if time.monotonic() > deadline:
                        raise DriverError(f"PMC not ready after {self.activate_timeout_s:.0f} s "
                                          f"({sys_cmd.get_pmc_status().name})")
                    time.sleep(0.2)
                self._wait_levitated(deadline)
                sys_cmd.save_pmc_config_xml_file(self.config_path)
                return parse_pmc_config(self.config_path)
            except DriverError:
                raise
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
            out[int(i.xbot_id)] = Reading(float(i.x_pos), float(i.y_pos), float(i.rz_pos), kind, state.name)
        return out

    def move(self, xid, x, y):
        with self._lock:
            try:
                return float(xbot_cmd.linear_motion_si(
                    self._next_label(), int(xid), pm.POSITIONMODE.ABSOLUTE, pm.LINEARPATHTYPE.DIRECT,
                    float(x), float(y), 0.0, float(self.speed), float(self.accel), 0.0))
            except Exception as e:
                raise DriverError(str(e)) from e

    def rotate(self, xid, rz):
        with self._lock:
            try:
                return float(xbot_cmd.rotary_motion_p2p(
                    self._next_label(), int(xid), pm.ROTATIONMODE.NO_ANGLE_WRAP, float(rz),
                    float(self.turn_speed), float(self.turn_accel), pm.POSITIONMODE.ABSOLUTE))
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
                    max_speed=float(self.speed), max_accel=float(self.accel),
                    xbot_ids=[int(x) for x in xids],
                    targets_x_meters=[float(t[0]) for t in targets],
                    targets_y_meters=[float(t[1]) for t in targets])
            except Exception as e:
                raise DriverError(str(e)) from e
