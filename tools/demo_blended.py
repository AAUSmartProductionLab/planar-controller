"""Demonstrate three-shuttle traffic using the blended coordinator."""

import argparse
import csv
import json
import logging
from pathlib import Path
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from planar.coordinator import Pose
from planar.coordinator_blended import Coordinator
from planar.layout import Layout
from planar.sim_driver import SimDriver
from planar.statemachines import ErrorID


GROUPS = [
    {1: (0.90, 0.66), 2: (0.42, 0.18), 3: (0.18, 0.36)},
    {1: (0.18, 0.66), 2: (0.90, 0.18), 3: (0.42, 0.18)},
    {1: (0.42, 0.18), 2: (0.18, 0.36), 3: (0.90, 0.66)},
]
PARK = {1: (0.42, 0.18), 2: (0.18, 0.36), 3: (0.18, 0.66)}


def main():
    """Run two traffic cycles and park all shuttles; --simulate uses a local model, --run the vendor
    simulator. Save feedback and results.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--run', action='store_true')
    mode.add_argument('--simulate', action='store_true', help='kinematic preflight; no PMC connection')
    parser.add_argument('--output', type=Path, default=Path('blended-demo-results'))
    args = parser.parse_args()
    if not args.run and not args.simulate:
        parser.print_help()
        return 0
    logging.basicConfig(level=logging.WARNING if args.simulate else logging.INFO, format='%(message)s')
    if args.simulate:
        driver = SimDriver(Layout(cols=4, rows=3, holes=frozenset({(2, 1)})),
                           {1: (0.42, 0.18), 2: (0.18, 0.360957), 3: (0.18, 0.610227)})
        clock = lambda: driver.now
    else:
        from planar.pmc_driver import PmcDriver
        driver = PmcDriver('127.1.1.1')
        clock = time.monotonic
    args.output.mkdir(parents=True, exist_ok=False)
    coordinator = Coordinator(driver, threading.RLock(), clock=clock, speed=0.15, accel=0.5)
    summary = dict(success=False, groups=[], peak_moving=0, blended_moves=0)
    previous = {}
    try:
        if coordinator.connect() != ErrorID.NONE:
            raise RuntimeError('cannot connect to simulator')
        if not {1, 2, 3} <= coordinator.agents.keys():
            raise RuntimeError('demo requires xbots 1, 2 and 3')
        if any(coordinator.agents[xid].state != 'idle' for xid in (1, 2, 3)):
            raise RuntimeError('demo requires all three xbots initially idle')
        for xid in coordinator.agents:
            coordinator.pin(xid, True)  # nobody is displaced without an explicit demo task
        start = clock()
        with (args.output/'feedback.csv').open('w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['elapsed_s', 'group', 'xbot', 'x_m', 'y_m', 'state', 'motion'])
            for group, targets in enumerate(GROUPS*2+[PARK], start=1):
                print(f'Group {group}/7: {targets}', flush=True)
                done = {}
                for xid, target in targets.items():
                    code = coordinator.submit(xid, Pose(*target),
                        lambda ok, error, results, xid=xid: done.update({xid: (ok, int(error))}))
                    if code != ErrorID.NONE:
                        raise RuntimeError(f'xbot {xid}: submit failed: {code}')
                group_start = clock()
                while len(done) < 3:
                    if args.simulate:
                        driver.advance(driver.now+0.02)
                    coordinator.tick(clock())
                    moving = 0
                    for xid in (1, 2, 3):
                        agent = coordinator.agents[xid]
                        motion = agent.motion
                        if motion is not None and motion is not previous.get(xid):
                            summary['blended_moves'] += int(motion.kind == 'blend')
                            previous[xid] = motion
                        moving += int(agent.state == 'moving')
                        writer.writerow([clock()-start, group, xid, agent.pose.x, agent.pose.y,
                                         agent.state, motion.kind if motion else ''])
                        if agent.fault:
                            raise RuntimeError(f'xbot {xid}: {agent.fault}')
                    summary['peak_moving'] = max(summary['peak_moving'], moving)
                    f.flush()
                    if any(not ok for ok, error in done.values()):
                        raise RuntimeError(f'move failed: {done}')
                    if not coordinator.connected or clock()-group_start > 60:
                        raise RuntimeError('group disconnected or timed out')
                    if args.simulate and driver.collisions:
                        raise RuntimeError(f'simulated collision: {driver.collisions[0]}')
                    if not args.simulate:
                        time.sleep(0.02)
                summary['groups'].append(dict(group=group, elapsed_s=clock()-group_start, results=done))
                print(f'Group {group} complete in {clock()-group_start:.1f}s', flush=True)
        summary['elapsed_s'] = clock()-start
        summary['success'] = True
    except (Exception, KeyboardInterrupt) as exc:
        summary['error'] = f'{type(exc).__name__}: {exc}'
        logging.exception('demo interrupted')
    finally:
        for xid in (1, 2, 3):
            try:
                driver.stop(xid)
            except Exception as exc:
                summary['success'] = False
                summary.setdefault('cleanup_errors', []).append(str(exc))
        try:
            driver.close()
        finally:
            (args.output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary, indent=2), flush=True)
    return 0 if summary['success'] else 1


if __name__ == '__main__':
    sys.exit(main())
