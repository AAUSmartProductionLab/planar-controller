"""
Run the line layouts of config/layouts/ against a PMC, Planar Motor's simulator by default: for
each, a controller with the layout's controller.env, then the product agents of
tests/line_scenario.py with its stations, which watch the shuttles (lane ways, turns); a line per
layout, PASS or FAIL.

    python tools/sim_check.py                         # every layout; the simulator running
    python tools/sim_check.py clockwise-outwards --start-sim --agents 3 --products 4
    python tools/sim_check.py --hold-every 8 --mastership

With --hold-every S the scenario holds the table every S seconds (nothing may move while it is
held); with --mastership another pmclib session takes mastership after the scenario: the table
must abort with MastershipLost (32), and take it back at a Clear and Reset.

Run it with the Python the controller runs with (pmclib from dependencies/). The logs go to a
temporary folder, named at the end. Exits 1 when a layout fails. Each controller is stopped with
Ctrl+Break (SIGTERM off Windows), so it disconnects from the PMC: one that is killed leaves the
simulator writing errors to its trace log for ever.
"""

import argparse
import asyncio
import os
import signal
import socket
import subprocess
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
LAYOUTS = ROOT / "config" / "layouts"
NAMESPACE = "urn:aau:smartproductionlab:planar"
ABORTED, IDLE, EXECUTE, MASTERSHIP_LOST = 9, 4, 6, 32


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


async def _mastership(endpoint, pmc_ip):
    """(passed, what happened): another session takes mastership, then gives it back."""
    from asyncua import Client, ua
    from pmclib import system_commands as pmc

    session = ua.Variant("", ua.VariantType.String)
    async with Client(endpoint) as client:
        ns = await client.get_namespace_index(NAMESPACE)
        module = await client.nodes.objects.get_child([f"{ns}:PlanarTable", f"{ns}:Module"])
        state = await module.get_child(f"{ns}:State")
        error = await module.get_child(f"{ns}:ErrorID")

        async def wait(wanted, timeout):
            t0 = time.monotonic()
            while time.monotonic() - t0 < timeout:
                if await state.read_value() == wanted:
                    return True
                await asyncio.sleep(0.1)
            return False

        if not await asyncio.to_thread(pmc.connect_to_specific_pmc, pmc_ip):
            return False, f"mastership: no second session with the PMC at {pmc_ip}"
        try:
            t0 = time.monotonic()
            await asyncio.to_thread(pmc.gain_mastership)
            aborted = await wait(ABORTED, 5)
            took = time.monotonic() - t0
            code = await error.read_value()
            await asyncio.to_thread(pmc.release_mastership)
        finally:
            await asyncio.to_thread(pmc.disconnect_from_pmc)
        if not aborted or code != MASTERSHIP_LOST:
            return False, (f"mastership taken: the table is in {await state.read_value()} with ErrorID {code}, "
                           f"not Aborted (9) with MastershipLost (32)")
        for command, wanted in (("Clear", None), ("Reset", IDLE), ("Start", EXECUTE)):
            await module.call_method(f"{ns}:{command}", session)
            if wanted and not await wait(wanted, 60):
                return False, f"mastership given back: the table is in {await state.read_value()} after {command}"
        return True, f"mastership taken: Aborted with MastershipLost in {took:.1f} s; taken back at a Reset"


def stop(controller, log_path):
    """Stop the controller so it disconnects from the PMC; '' or what went wrong."""
    if os.name == "nt":
        controller.send_signal(signal.CTRL_BREAK_EVENT)
    else:
        controller.terminate()
    try:
        controller.wait(15)
    except subprocess.TimeoutExpired:
        controller.kill()
        controller.wait(5)
        return "the controller had to be killed: it did not disconnect from the PMC"
    if "disconnected from the PMC" not in log_path.read_text(encoding="utf-8", errors="replace"):
        return "the controller stopped without disconnecting from the PMC"
    return ""


def check(layout, args, logs):
    """(passed, what the scenario said or why it did not run)."""
    port = free_port()
    endpoint = f"opc.tcp://127.0.0.1:{port}/"
    env = {**os.environ, **settings(layout / "controller.env"), "PMC_IP": args.pmc_ip,
           "OPCUA_ENDPOINT": endpoint, "AUTOSTART": "0", "PYTHONUNBUFFERED": "1"}
    controller_log = logs / f"{layout.name}.controller.log"
    with open(controller_log, "w", encoding="utf-8") as out:
        controller = subprocess.Popen([sys.executable, "-m", "planar"], cwd=ROOT, env=env,
                                      stdout=out, stderr=subprocess.STDOUT,
                                      creationflags=subprocess.CREATE_NEW_PROCESS_GROUP if os.name == "nt" else 0)
        try:
            deadline = time.monotonic() + 60
            while not listening(port):
                if controller.poll() is not None:
                    return False, f"the controller stopped: {tail(controller_log)}"
                if time.monotonic() > deadline:
                    return False, "the controller did not open its OPC UA port in 60 s"
                time.sleep(0.5)
            hold = ["--hold-every", str(args.hold_every)] if args.hold_every else []
            try:
                scenario = subprocess.run(
                    [sys.executable, str(ROOT / "tests" / "line_scenario.py"), "--endpoint", endpoint,
                     "--layout", str(layout), "--agents", str(args.agents), "--products", str(args.products),
                     "--seed", str(args.seed), *hold],
                    cwd=ROOT, capture_output=True, text=True, timeout=args.timeout)
            except subprocess.TimeoutExpired:
                return False, f"the scenario did not end in {args.timeout:.0f} s"
            (logs / f"{layout.name}.scenario.log").write_text(scenario.stdout + scenario.stderr, encoding="utf-8")
            said = [line for line in scenario.stdout.splitlines() if line.strip()]
            problems = [line for line in said if line.startswith("PROBLEM")]
            last = said[-1] if said else (scenario.stderr.strip().splitlines() or ["no output"])[-1]
            passed, text = scenario.returncode == 0, last + "".join(f"\n      {p}" for p in problems[:3])
            if passed and args.mastership:
                passed, what = asyncio.run(_mastership(endpoint, args.pmc_ip))
                text += f"\n      {what}"
            return passed, text
        finally:
            trouble = stop(controller, controller_log)
            if trouble:
                print(f"     {layout.name}: {trouble}", flush=True)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("layouts", nargs="*", help="folders of config/layouts/ (default: all)")
    p.add_argument("--pmc-ip", default="127.0.0.1")
    p.add_argument("--agents", type=int, default=3)
    p.add_argument("--products", type=int, default=3, help="per agent")
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--timeout", type=float, default=1200, help="seconds a layout's scenario may take")
    p.add_argument("--hold-every", type=float, metavar="S", help="hold the table every S seconds during the scenario")
    p.add_argument("--mastership", action="store_true",
                   help="after the scenario, take mastership in another session: the table must abort")
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
