"""Run the planar controller and its OPC UA server; see mkdocs/docs/configuration.md for settings."""

import argparse
import asyncio
import errno
import logging
import math
import os
import signal

from . import lines, zones
from .driver import DriverError
from .modules import PlanarController
from .opcua_server import OpcUaServer
from .pmc_driver import PmcDriver
from .statemachines import ModuleState

log = logging.getLogger("planar")


def _env(name, default=""):
    """Read an environment setting, using the default when it is empty."""
    return os.environ.get(name, "").strip() or default


def _flag(name):
    """Parse an environment setting as a Boolean flag."""
    return _env(name).lower() in ("1", "true", "yes", "on")


def _number(text):
    """A number; empty -> None."""
    if not text.strip():
        return None
    try:
        return float(text)
    except ValueError:
        raise argparse.ArgumentTypeError(f"not a number: {text!r}") from None


def _positive(text):
    """A number above 0."""
    value = _number(text)
    if value is None or not 0 < value < math.inf:
        raise argparse.ArgumentTypeError(f"not a number above 0: {text!r}")
    return value


def _ids(text):
    """"1,2,3" -> [1, 2, 3]; empty -> None."""
    if not text.strip():
        return None
    try:
        return [int(x) for x in text.replace(" ", "").split(",") if x]
    except ValueError:
        raise argparse.ArgumentTypeError(f"not xbot ids like 1,2,3: {text!r}") from None


def _mode(text):
    """Validate and normalize the free or line routing mode."""
    if text.strip().lower() not in ("free", "line"):
        raise argparse.ArgumentTypeError(f"free or line, not {text!r}")
    return text.strip().lower()


def _level(text):
    """Validate and normalize a logging level."""
    if text.strip().upper() not in logging.getLevelNamesMapping():
        raise argparse.ArgumentTypeError(f"DEBUG, INFO, WARNING or ERROR, not {text!r}")
    return text.strip().upper()


def _point_mm(text):
    """"X,Y" in mm -> (x, y) in m; empty -> None."""
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
    """Parse command-line options with environment variables as defaults."""
    p = argparse.ArgumentParser(prog="python -m planar", description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    # Each defaults to its environment variable, which argparse reads as if it were given here.
    p.add_argument("--pmc-ip", default=_env("PMC_IP") or None)
    p.add_argument("--endpoint", default=_env("OPCUA_ENDPOINT", "opc.tcp://0.0.0.0:4840/"))
    p.add_argument("--shuttles", type=_ids, metavar="1,2,3", default=_env("SHUTTLES"))
    p.add_argument("--autostart", action="store_true", default=_flag("AUTOSTART"))
    p.add_argument("--displace-held", action="store_true", default=_flag("DISPLACE_HELD"))
    p.add_argument("--blocked-timeout", type=_number, metavar="S", default=_env("BLOCKED_TIMEOUT_S"))
    p.add_argument("--loading-station", type=_point_mm, metavar="X,Y", default=_env("LOADING_STATION"))
    p.add_argument("--idle-rz", type=_quarter_deg, metavar="DEG", default=_env("IDLE_RZ"))
    p.add_argument("--mode", type=_mode, metavar="free|line", default=_env("MODE", "free"))
    p.add_argument("--planner", choices=("classic", "blended"), default=_env("PLANNER", "classic"),
                   help="classic (default) or experimental blended XY corners; restart to switch")
    p.add_argument("--line-layout", metavar="FILE|URL", default=_env("LINE_LAYOUT"))
    p.add_argument("--zones", metavar="FILE|URL", default=_env("ZONES"))
    p.add_argument("--speed", type=_positive, metavar="M/S", default=_env("XBOT_SPEED", "0.5"))
    p.add_argument("--accel", type=_positive, metavar="M/S2", default=_env("XBOT_ACCEL", "2.0"))
    p.add_argument("--jerk", type=_number, metavar="M/S3", default=_env("XBOT_JERK"))
    p.add_argument("--log-level", type=_level, default=_env("LOG_LEVEL", "INFO"))
    return p.parse_args()


async def detect(driver):
    """Wait for a PMC connection and return its sorted xbot IDs."""
    while True:
        try:
            shuttles = sorted(await asyncio.to_thread(driver.detect))
        except DriverError as e:
            log.warning("waiting for the PMC to list its xbots (or set SHUTTLES): %s", e)
            await asyncio.sleep(5)
            continue
        log.info("xbots on the PMC: %s", shuttles)
        return shuttles


async def line_layout(source, retry_s=10.0):
    """Load a line layout, retrying unavailable URLs and rejecting invalid layouts."""
    while True:
        try:
            spec = await asyncio.to_thread(lines.load, source)
        except lines.LineError as e:
            raise SystemExit(f"LINE_LAYOUT {source}: {e}") from None
        except OSError as e:
            if not source.startswith(("http://", "https://")):
                raise SystemExit(f"LINE_LAYOUT {source}: {e}") from None
            log.warning("waiting for the line layout from %s: %s", source, e)
            await asyncio.sleep(retry_s)
            continue
        log.info("line mode: %d lanes from %s", len(spec.lanes), source)
        return spec


async def autostart(ctl, retry_s=10.0):
    """Retry Reset and Start until ready or another command changes the table state."""
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
    """Configure the driver and OPC UA server, then serve until interrupted."""
    args = parse_args()
    logging.basicConfig(level=args.log_level, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    logging.getLogger("asyncua").setLevel(logging.WARNING)  # it logs every request at INFO
    line_spec = None
    if args.mode == "line":
        if not args.line_layout:
            raise SystemExit("MODE=line needs LINE_LAYOUT (a JSON file or URL)")
        line_spec = await line_layout(args.line_layout)
    elif args.line_layout:
        log.warning("LINE_LAYOUT is set but MODE is free: the lanes are not used")
    zone_specs = []
    if args.zones:
        try:
            zone_specs = zones.load(args.zones)
        except (OSError, zones.ZoneError) as e:
            raise SystemExit(f"ZONES {args.zones}: {e}") from None
        log.info("zones: %s", ", ".join(z.name for z in zone_specs) or "none")
    driver = PmcDriver(args.pmc_ip, jerk=args.jerk)
    shuttles = args.shuttles or await detect(driver)
    ctl = PlanarController(driver, shuttles, planner=args.planner, displace_held=args.displace_held, speed=args.speed, accel=args.accel,
                           patience_s=args.blocked_timeout or None, loading_station=args.loading_station,
                           idle_rz=args.idle_rz, line_spec=line_spec,
                           line_source=(lambda: lines.load(args.line_layout)) if line_spec else None,
                           zones=zone_specs)
    server = OpcUaServer(ctl, args.endpoint)
    await server.init()
    ctl.start()
    try:
        tasks = [asyncio.create_task(server.run())]
        if args.autostart:
            tasks.append(asyncio.create_task(autostart(ctl)))
        await asyncio.gather(*tasks)
    except OSError as e:
        # Windows: WSAEADDRINUSE, or EACCES binding 0.0.0.0 on a port another program holds
        if e.errno not in (errno.EADDRINUSE, errno.EACCES, 10048, 10013):
            raise
        raise SystemExit(f"OPC UA endpoint {args.endpoint}: its port is in use by another program (another "
                         f"controller still running? on Windows the OPC UA Local Discovery Server holds 4840); "
                         f"set OPCUA_ENDPOINT to another port, e.g. opc.tcp://0.0.0.0:4841/") from None
    finally:
        ctl.shutdown()
        log.info("stopped")


def _interrupt(signum, frame):
    """Translate an operating-system signal into KeyboardInterrupt."""
    raise KeyboardInterrupt(signal.Signals(signum).name)


if __name__ == "__main__":
    # Without a handler these end the process at once, without disconnecting from the PMC.
    for name in ("SIGTERM", "SIGBREAK"):
        if hasattr(signal, name):
            signal.signal(getattr(signal, name), _interrupt)
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
