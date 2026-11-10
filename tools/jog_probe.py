"""Inspect the installed jogging API and optionally run a bounded XY jog."""

import argparse
import inspect
import math
import sys
import time
from importlib.metadata import version


def main():
    """Inspect by default; --run jogs one prepared idle xbot, logs feedback and requests stop/disconnect on
    exit. Process loss bypasses cleanup.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", action="store_true", help="connect, take mastership and jog")
    parser.add_argument("--ip", default="127.1.1.1", help="observed local simulator address")
    parser.add_argument("--xbot", type=int, default=1)
    parser.add_argument("--vx", type=float, default=0.02, help="table X velocity, m/s")
    parser.add_argument("--vy", type=float, default=0.0, help="table Y velocity, m/s")
    parser.add_argument("--duration", type=float, default=1.0, help="seconds, maximum 5")
    args = parser.parse_args()
    if not all(math.isfinite(v) for v in (args.vx, args.vy, args.duration)):
        parser.error("velocity and duration must be finite")
    if args.xbot <= 0:
        parser.error("select one positive xbot ID; broadcast is not supported")
    if not 0 < args.duration <= 5 or math.hypot(args.vx, args.vy) > 0.05:
        parser.error("this probe limits duration to 5 s and vector speed to 0.05 m/s")

    from pmclib import pmc_types as pm
    from pmclib import system_commands as system
    from pmclib import xbot_commands as xbot

    jog = getattr(xbot, "long_axis_jogging_control", None)
    print(f"pmclib {version('pmclib')}: {xbot.__file__}", file=sys.stderr)
    if jog is None:
        raise RuntimeError("installed pmclib has no long_axis_jogging_control")
    print(f"long_axis_jogging_control{inspect.signature(jog)}", file=sys.stderr)
    if not args.run:
        print("Inspection only. Use --run to connect and jog a prepared xbot.", file=sys.stderr)
        return 0

    connected = master = attempted = False
    failed = False

    def reading():
        """Return the selected xbot's feedback, or None when it is absent."""
        infos = xbot.get_all_xbot_info(pm.ALLXBOTSFEEDBACKOPTION(0))
        return next((i for i in infos if int(i.xbot_id) == args.xbot), None)

    try:
        connected = bool(system.connect_to_specific_pmc(args.ip))
        if not connected:
            raise RuntimeError(f"could not connect to {args.ip}")
        system.gain_mastership()
        master = bool(system.is_master())
        if not master:
            raise RuntimeError("mastership unavailable; release PMT control first")
        info = reading()
        if info is None or info.xbot_state != pm.XBOTSTATE.XBOT_IDLE:
            raise RuntimeError("selected xbot must already be levitated and idle")
        print(f"Jogging xbot {args.xbot}: ({args.vx}, {args.vy}) m/s for {args.duration}s",
              file=sys.stderr, flush=True)
        print("elapsed_s,unix_time_s,x_m,y_m,rz_rad,state", flush=True)
        start = time.monotonic()
        attempted = True  # also stop if the command executes but its reply fails
        jog(args.xbot, args.vx, args.vy, pm.JOGGINGOPERATION.JOGGING_ON)
        while time.monotonic() - start < args.duration:
            info = reading()
            if info is None:
                raise RuntimeError("xbot disappeared from feedback")
            print(f"{time.monotonic()-start:.6f},{time.time():.6f},"
                  f"{info.x_pos},{info.y_pos},{info.rz_pos},{info.xbot_state.name}", flush=True)
            if info.xbot_state not in (pm.XBOTSTATE.XBOT_IDLE, pm.XBOTSTATE.XBOT_MOTION):
                raise RuntimeError(f"unexpected state: {info.xbot_state.name}")
            if not system.is_master():
                raise RuntimeError("lost mastership")
            time.sleep(min(0.05, max(0, args.duration - (time.monotonic() - start))))
    except (Exception, KeyboardInterrupt) as exc:
        print(f"Probe interrupted/failed: {exc}", file=sys.stderr)
        failed = True
    finally:
        cleanup = []
        if attempted:
            cleanup.extend([
                ("jog off", lambda: jog(args.xbot, 0.0, 0.0, pm.JOGGINGOPERATION.JOGGING_OFF)),
                ("stop motion", lambda: xbot.stop_motion(args.xbot)),
            ])
        if master:
            cleanup.append(("release mastership", system.release_mastership))
        if connected:
            cleanup.append(("disconnect", system.disconnect_from_pmc))
        for label, operation in cleanup:
            try:
                operation()
                print(f"{label}: OK", file=sys.stderr)
            except Exception as exc:
                failed = True
                print(f"{label}: FAILED: {exc}", file=sys.stderr)
    return int(failed)


if __name__ == "__main__":
    sys.exit(main())
