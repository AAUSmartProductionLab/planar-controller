"""
Run the line layouts of config/layouts/ against a PMC, Planar Motor's simulator by default: for
each, a controller with the layout's controller.env, then the product agents of
tests/line_scenario.py with its stations, which watch the shuttles (lane ways, turns); a line per
layout, PASS or FAIL.

    python tools/sim_check.py                         # every layout; the simulator running
    python tools/sim_check.py clockwise-outwards --start-sim --agents 3 --products 4

Run it with the Python the controller runs with (pmclib from dependencies/). The logs go to a
temporary folder, named at the end. Exits 1 when a layout fails.
"""

import argparse
import os
import socket
import subprocess
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
LAYOUTS = ROOT / "config" / "layouts"


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def listening(port):
    with socket.socket() as s:
        s.settimeout(0.5)
        return s.connect_ex(("127.0.0.1", port)) == 0


def settings(path):
    """controller.env: {KEY: value}."""
    lines = (line.strip() for line in path.read_text(encoding="utf-8").splitlines())
    return dict(line.split("=", 1) for line in lines if line and not line.startswith("#"))


def tail(path, n=3):
    lines = [line for line in path.read_text(encoding="utf-8", errors="replace").splitlines() if line.strip()]
    return " | ".join(lines[-n:])


def check(layout, args, logs):
    """(passed, what the scenario said or why it did not run)."""
    port = free_port()
    endpoint = f"opc.tcp://127.0.0.1:{port}/"
    env = {**os.environ, **settings(layout / "controller.env"), "PMC_IP": args.pmc_ip,
           "OPCUA_ENDPOINT": endpoint, "AUTOSTART": "0", "PYTHONUNBUFFERED": "1"}
    controller_log = logs / f"{layout.name}.controller.log"
    with open(controller_log, "w", encoding="utf-8") as out:
        controller = subprocess.Popen([sys.executable, "-m", "planar"], cwd=ROOT, env=env,
                                      stdout=out, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 60
            while not listening(port):
                if controller.poll() is not None:
                    return False, f"the controller stopped: {tail(controller_log)}"
                if time.monotonic() > deadline:
                    return False, "the controller did not open its OPC UA port in 60 s"
                time.sleep(0.5)
            try:
                scenario = subprocess.run(
                    [sys.executable, str(ROOT / "tests" / "line_scenario.py"), "--endpoint", endpoint,
                     "--layout", str(layout), "--agents", str(args.agents), "--products", str(args.products),
                     "--seed", str(args.seed)],
                    cwd=ROOT, capture_output=True, text=True, timeout=args.timeout)
            except subprocess.TimeoutExpired:
                return False, f"the scenario did not end in {args.timeout:.0f} s"
            (logs / f"{layout.name}.scenario.log").write_text(scenario.stdout + scenario.stderr, encoding="utf-8")
            said = [line for line in scenario.stdout.splitlines() if line.strip()]
            problems = [line for line in said if line.startswith("PROBLEM")]
            last = said[-1] if said else (scenario.stderr.strip().splitlines() or ["no output"])[-1]
            return scenario.returncode == 0, last + "".join(f"\n      {p}" for p in problems[:3])
        finally:
            controller.terminate()
            try:
                controller.wait(10)
            except subprocess.TimeoutExpired:
                controller.kill()


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("layouts", nargs="*", help="folders of config/layouts/ (default: all)")
    p.add_argument("--pmc-ip", default="127.0.0.1")
    p.add_argument("--agents", type=int, default=3)
    p.add_argument("--products", type=int, default=3, help="per agent")
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--timeout", type=float, default=1200, help="seconds a layout's scenario may take")
    p.add_argument("--start-sim", action="store_true", help="start the simulator first (tools/pmc-sim.ps1, Windows)")
    args = p.parse_args()
    names = args.layouts or sorted(d.name for d in LAYOUTS.iterdir() if d.is_dir())
    missing = [n for n in names if not (LAYOUTS / n / "controller.env").is_file()]
    if missing:
        raise SystemExit(f"no layout {', '.join(missing)} in {LAYOUTS} (there: "
                         f"{', '.join(sorted(d.name for d in LAYOUTS.iterdir() if d.is_dir()))})")
    if args.start_sim:
        subprocess.run(["powershell", "-ExecutionPolicy", "Bypass", "-File", str(ROOT / "tools" / "pmc-sim.ps1"),
                        "start"], check=True)
    logs = Path(tempfile.mkdtemp(prefix="sim-check-"))
    failed = 0
    for name in names:
        t0 = time.monotonic()
        passed, said = check(LAYOUTS / name, args, logs)
        failed += not passed
        print(f"{'PASS' if passed else 'FAIL'} {name} ({time.monotonic() - t0:.0f} s): {said}", flush=True)
    print(f"{len(names) - failed} of {len(names)} passed; logs in {logs}")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
