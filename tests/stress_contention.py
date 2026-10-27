"""
Contention stress on the simulated driver: many xbots ask for the same place; each that gets
there stays a while (held, as a station would hold it), then leaves for its own parking spot
and is released. The places are of every kind: slot centres, between slots, slot corners, flyway
centres, turned and crooked.

Checks: every request succeeds, arrivals follow request order (same-tick requests: by xbot id),
no collision, no hang. Prints one line per bad run and a summary.

    python tests/stress_contention.py [seeds]      # 12 runs per seed, about 0.7 s a run
"""
import logging
import math
import os
import random
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)
from test_coordinator import LAB, SIM, Harness  # noqa: E402

from planar.layout import overlap  # noqa: E402
from planar.statemachines import ErrorID  # noqa: E402


def poses(layout, rng):
    """A random pose of every kind: slot centre, between two slots, slot corner, flyway centre."""
    kind = rng.choice(["slot", "slot", "between", "corner", "flyway", "flyway_turned", "flyway_crooked"])
    i = rng.randrange(layout.cols * 2)
    j = rng.randrange(layout.rows * 2)
    p = 0.12
    if kind == "slot":
        x, y, rz = (i + 0.5) * p, (j + 0.5) * p, rng.choice([0, 0, 90, 180])
    elif kind == "between":
        if rng.random() < 0.5:
            x, y = (i + 1) * p, (j + 0.5) * p
        else:
            x, y = (i + 0.5) * p, (j + 1) * p
        x += rng.choice([0, 0, 0.013, -0.027])
        rz = rng.choice([0, 90])
    elif kind == "corner":
        x, y, rz = (i + 1) * p + rng.choice([0, 0.011]), (j + 1) * p, 0
    else:
        f = rng.choice(layout.flyways)
        x, y = layout.flyway_center(f)
        rz = {"flyway": 0, "flyway_turned": rng.choice([90, 180, -90]), "flyway_crooked": rng.choice([45, 30, -60])}[kind]
    return x, y, rz, kind


def valid(layout, pose):
    x, y, rz, _ = pose
    if not layout.on_table(x, y, math.radians(rz)):
        return False
    if rz % 90 and layout.flyway_at(x, y, 1e-6) is None:
        return False
    return True


def free_of(layout, pose, others):
    box = layout.square(pose[0], pose[1], math.radians(pose[2]))
    return all(not overlap(box, layout.square(o[0], o[1], math.radians(o[2]))) for o in others)


def trial(layout, n, seed, stagger, dwell):
    rng = random.Random(seed)
    # The contested spot, then distinct start poses (square-aligned) and parking spots.
    while True:
        spot = poses(layout, rng)
        if valid(layout, spot):
            break
    taken = [spot]
    starts = {}
    for xid in range(1, n + 1):
        for _ in range(500):
            s = poses(layout, rng)
            if s[2] % 90 == 0 and valid(layout, s) and free_of(layout, s, taken):
                starts[xid] = s
                taken.append(s)
                break
        else:
            return None
    parks = {}
    for xid in starts:
        for _ in range(500):
            s = poses(layout, rng)
            if valid(layout, s) and free_of(layout, s, taken):
                parks[xid] = s
                taken.append(s)
                break
        else:
            return None
    h = Harness(layout, {x: (s[0], s[1], math.radians(s[2])) for x, s in starts.items()})
    for x in starts:
        h.coord.hold(x, True)  # occupied by their orchestrators throughout
    order = list(starts)
    rng.shuffle(order)
    requests = []
    t_send = {}
    for k, x in enumerate(order):
        t_send[x] = h.t + (k * stagger if stagger else 0)
    pending = sorted(order, key=lambda x: (t_send[x], x))
    expected = list(pending)
    arrived = []
    leave_at = {}
    parked = set()
    deadline = 900.0
    while h.t < deadline:
        for x in list(pending):
            if h.t >= t_send[x]:
                e = h.send(x, spot[0], spot[1], spot[2])
                if e != ErrorID.NONE:
                    return f"send {x} refused {e!r}"
                requests.append(x)
                pending.remove(x)
        h.run(0.02)
        for x in starts:
            res = h.results.get(x, [])
            if len(res) >= 1 and x not in arrived:
                ok, err, r = res[0]
                if not ok:
                    return f"xbot {x} to spot failed {err!r}"
                arrived.append(x)
                leave_at[x] = h.t + dwell
            if x in leave_at and h.t >= leave_at[x]:
                del leave_at[x]
                e = h.send(x, parks[x][0], parks[x][1], parks[x][2])
                if e != ErrorID.NONE:
                    return f"park {x} refused {e!r}"
            if len(res) >= 2 and x not in parked:
                ok, err, r = res[1]
                if not ok:
                    return f"xbot {x} to park failed {err!r}"
                parked.add(x)
                h.coord.hold(x, False)  # the product is gone: the shuttle is released
        if len(parked) == n:
            break
    if h.sim.collisions:
        return f"collision {h.sim.collisions[0]}"
    if len(parked) < n:
        stuck = {x: (len(h.results.get(x, [])), h.coord.agents[x].cell, h.coord.agents[x].want) for x in starts}
        return f"hang at t={h.t:.0f}: arrived {arrived} parked {sorted(parked)} spot {spot} {stuck}"
    if arrived != expected:
        return f"order {arrived} != requests {expected} (spot {spot[3]})"
    return "ok"


def main():
    logging.disable(logging.CRITICAL)
    bad = runs = 0
    t0 = time.time()
    combos = [(SIM, 3), (SIM, 5), (SIM, 8), (LAB, 3), (LAB, 6), (LAB, 10)]
    seeds = range(int(sys.argv[1]) if len(sys.argv) > 1 else 25)
    for layout, n in combos:
        for seed in seeds:
            for stagger in (0.0, 0.7):
                out = trial(layout, n, seed * 7 + n, stagger, dwell=1.5)
                if out is None:
                    continue
                runs += 1
                if out != "ok":
                    bad += 1
                    print(f"BAD {'SIM' if layout is SIM else 'LAB'} n={n} seed={seed} stagger={stagger}: {out}", flush=True)
    print(f"{runs} runs, {bad} bad, {time.time() - t0:.0f}s")


if __name__ == "__main__":
    main()
