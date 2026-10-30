"""
Run the planar table controller: python -m planar

Environment (each also an option, see --help):
  PMC_IP          the PMC's address (empty: auto search)
  OPCUA_ENDPOINT  where the OPC UA server listens (opc.tcp://0.0.0.0:4840/)
  SHUTTLES        the xbot ids to offer as shuttles, e.g. "1,2,3" (empty: the ids the PMC knows)
  AUTOSTART       1 to Reset and Start the table when the controller starts
  DISPLACE_HELD   1 to let the planner push occupied shuttles that are not moving
  BLOCKED_TIMEOUT_S  seconds a move may wait without getting closer before it fails with
                  Blocked (20); 0 (the default): it waits until the way is free or it is stopped
  LOADING_STATION "X,Y" in mm: the xbots nobody holds queue up there, and the table's Reset
                  sends them there (empty: an idle xbot stays where it was)
  IDLE_RZ         degrees, a multiple of 90: the xbots nobody holds are turned to it, and the
                  table's Reset turns every shuttle to it (empty: as they are); also
                  PlanarTable/Equipment/IdleRz over OPC UA
  XBOT_SPEED      m/s (0.5), XBOT_ACCEL m/s^2 (2.0)
  LOG_LEVEL       INFO
"""

import argparse
import asyncio
import logging
import math
import os

from .coordinator import DriverError
from .modules import PlanarController
from .opcua_server import OpcUaServer
from .pmc_driver import PmcDriver
from .statemachines import ModuleState

log = logging.getLogger("planar")


def _flag(name):
    return os.environ.get(name, "").strip().lower() in ("1", "true", "yes", "on")


def _point_mm(text):
    """ "X,Y" in mm -> (x, y) in m; empty -> None."""
    if not text or not text.strip():
        return None
    try:
        x, y = (float(v) for v in text.replace(" ", "").split(","))
    except ValueError:
        raise argparse.ArgumentTypeError(f"not X,Y in mm: {text!r}") from None
    return x / 1000.0, y / 1000.0


def _quarter_deg(text):
    """Degrees, a multiple of 90 -> radians; empty -> None."""
    if not text or not text.strip():
        return None
    try:
        deg = float(text)
    except ValueError:
        raise argparse.ArgumentTypeError(f"not degrees: {text!r}") from None
    if deg % 90:
        raise argparse.ArgumentTypeError(f"not a multiple of 90 degrees: {text!r}")
    return math.radians(deg)


def parse_args():
    p = argparse.ArgumentParser(prog="python -m planar", description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--pmc-ip", default=os.environ.get("PMC_IP") or None)
    p.add_argument("--endpoint", default=os.environ.get("OPCUA_ENDPOINT", "opc.tcp://0.0.0.0:4840/"))
    p.add_argument("--shuttles", default=os.environ.get("SHUTTLES", ""))
    p.add_argument("--autostart", action="store_true", default=_flag("AUTOSTART"))
    p.add_argument("--displace-held", action="store_true", default=_flag("DISPLACE_HELD"))
    p.add_argument("--blocked-timeout", type=float, default=float(os.environ.get("BLOCKED_TIMEOUT_S", 0) or 0))
    p.add_argument("--loading-station", type=_point_mm, metavar="X,Y", default=os.environ.get("LOADING_STATION", ""))
    p.add_argument("--idle-rz", type=_quarter_deg, metavar="DEG", default=os.environ.get("IDLE_RZ", ""))
    p.add_argument("--speed", type=float, default=float(os.environ.get("XBOT_SPEED", 0.5)))
    p.add_argument("--accel", type=float, default=float(os.environ.get("XBOT_ACCEL", 2.0)))
    p.add_argument("--log-level", default=os.environ.get("LOG_LEVEL", "INFO"))
    return p.parse_args()


async def detect(driver):
    """The xbot ids the PMC knows; waits for the PMC rather than giving up."""
    while True:
        try:
            shuttles = sorted(await asyncio.to_thread(driver.detect))
        except DriverError as e:
            log.warning("waiting for the PMC to list its xbots (or set SHUTTLES): %s", e)
            await asyncio.sleep(5)
            continue
        log.info("xbots on the PMC: %s", shuttles)
        return shuttles


async def autostart(ctl, retry_s=10.0):
    """Reset and Start the table; while the PMC is not there yet (it may still be starting up),
    try again every retry_s. It gives up only when someone else has commanded the table."""
    while True:
        with ctl.lock:
            if ctl.table.state != ModuleState.STOPPED:
                return
            ctl.table.command("Reset", "")
        while True:
            await asyncio.sleep(0.2)
            with ctl.lock:
                state, error = ctl.table.state, ctl.table.error
                if state == ModuleState.IDLE:
                    ctl.table.command("Start", "")
                    return
                if state != ModuleState.RESETTING:
                    break
        log.warning("autostart: the table did not reset (%s, %s); trying again in %.0f s",
                    state.name, error.name, retry_s)
        await asyncio.sleep(retry_s)


async def main():
    args = parse_args()
    logging.basicConfig(level=args.log_level, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    logging.getLogger("asyncua").setLevel(logging.WARNING)  # it logs every request at INFO
    driver = PmcDriver(args.pmc_ip, speed=args.speed, accel=args.accel)
    if args.shuttles.strip():
        shuttles = [int(x) for x in args.shuttles.replace(" ", "").split(",") if x]
    else:
        shuttles = await detect(driver)
    ctl = PlanarController(driver, shuttles, displace_held=args.displace_held,
                           patience_s=args.blocked_timeout or None, loading_station=args.loading_station,
                           idle_rz=args.idle_rz)
    server = OpcUaServer(ctl, args.endpoint)
    await server.init()
    ctl.start()
    try:
        tasks = [asyncio.create_task(server.run())]
        if args.autostart:
            tasks.append(asyncio.create_task(autostart(ctl)))
        await asyncio.gather(*tasks)
    finally:
        ctl.shutdown()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
