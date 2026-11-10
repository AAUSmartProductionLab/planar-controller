"""Compare classic and blended execution on the standard local vendor simulator."""

import argparse
import csv
import json
import logging
from pathlib import Path
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def main():
    """With --run, compare xbot 1 from (420,180) to (900,660) mm, pin others and return; write CSV and JSON
    results.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', action='store_true')
    parser.add_argument('--output', type=Path, default=Path('planner-comparison'))
    args = parser.parse_args()
    if not args.run:
        print('Use --run to move xbot 1 on the local simulator at 127.1.1.1.')
        return 0
    from planar.coordinator import Coordinator as Classic, Pose
    from planar.coordinator_blended import Coordinator as Blended
    from planar.pmc_driver import PmcDriver
    from planar.statemachines import ErrorID

    logging.basicConfig(level=logging.INFO, format='%(message)s')
    args.output.mkdir(parents=True, exist_ok=False)
    driver = PmcDriver('127.1.1.1')
    results = []
    failed = False
    with (args.output/'feedback.csv').open('w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['phase', 'elapsed_s', 'x_m', 'y_m', 'state', 'motion_kind'])

        def run(kind, target, label):
            """Execute and time one move with the selected coordinator, logging feedback until completion."""
            coordinator = kind(driver, threading.RLock(), speed=0.1, accel=0.5)
            if coordinator.connect() != ErrorID.NONE:
                raise RuntimeError('coordinator connection failed')
            for xid in coordinator.agents:
                coordinator.pin(xid, True)
            done, motions = [], []
            code = coordinator.submit(1, Pose(*target, rz=0.0), lambda *v: done.append(v))
            if code != ErrorID.NONE:
                raise RuntimeError(f'{label}: submit failed: {code}')
            start = time.monotonic()
            last_motion = None
            while not done:
                coordinator.tick(time.monotonic())
                agent = coordinator.agents[1]
                motion = agent.motion
                if motion is not None and motion is not last_motion:
                    motions.append(dict(kind=motion.kind, target=motion.target, expected_s=motion.expected))
                    last_motion = motion
                writer.writerow([label, time.monotonic()-start, agent.pose.x, agent.pose.y,
                                 agent.state, motion.kind if motion else ''])
                f.flush()
                if not coordinator.connected or time.monotonic()-start > 60:
                    raise RuntimeError(f'{label}: disconnected or timed out')
                time.sleep(0.02)
            result = dict(phase=label, elapsed_s=time.monotonic()-start,
                          success=done[0][0], error=int(done[0][1]), motions=motions,
                          final_xy_m=[agent.pose.x, agent.pose.y])
            results.append(result)
            print(json.dumps(result), flush=True)
            if not result['success']:
                raise RuntimeError(f'{label} failed')

        try:
            run(Classic, (0.42, 0.18), 'stage')
            run(Classic, (0.90, 0.66), 'classic')
            run(Classic, (0.42, 0.18), 'return')
            run(Blended, (0.90, 0.66), 'blended')
            run(Classic, (0.42, 0.18), 'final_return')
        except (Exception, KeyboardInterrupt):
            logging.exception('comparison interrupted')
            failed = True
        finally:
            try:
                if driver._connected:
                    driver.stop(1)
            finally:
                driver.close()
                (args.output/'summary.json').write_text(json.dumps(results, indent=2)+'\n')
    return int(failed)


if __name__ == '__main__':
    sys.exit(main())
