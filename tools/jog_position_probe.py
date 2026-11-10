"""Probe XY position control using repeated jogging commands on the local simulator."""

import argparse
import csv
import json
import math
from pathlib import Path
import sys
import time


def main():
    """With --run, move 20 mm in X and 15 mm in Y and return using PI feedback; --restart stops between
    updates. Log results and clean up.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', action='store_true')
    parser.add_argument('--restart', action='store_true', help='disable jog and wait idle before each update')
    parser.add_argument('--xbot', type=int, default=1)
    parser.add_argument('--kp', type=float, default=2.0, help='1/s')
    parser.add_argument('--ki', type=float, default=0.1, help='1/s^2; zero selects P only')
    parser.add_argument('--output', type=Path, default=Path('jog-position-results'))
    args = parser.parse_args()
    if args.xbot <= 0 or not math.isfinite(args.kp) or not math.isfinite(args.ki):
        parser.error('positive xbot ID and finite gains required')
    if not 0 < args.kp <= 5 or not 0 <= args.ki <= 1:
        parser.error('probe gain limits: 0 < kp <= 5, 0 <= ki <= 1')
    if not args.run:
        print('Use --run to test the local simulator at 127.1.1.1.')
        return 0

    from pmclib import system_commands as system, xbot_commands as xbot, pmc_types as pm

    args.output.mkdir(parents=True, exist_ok=False)
    summary = dict(kp=args.kp, ki=args.ki, restart=args.restart, speed_limit_m_s=0.02,
                   tolerance_m=0.0005, phases=[], success=False)
    connected = master = attempted = False
    intervals = []
    moving_updates = 0
    failure = None

    def read():
        """Read all xbots and require finite feedback with an idle or moving state for the selected xbot."""
        infos = xbot.get_all_xbot_info(pm.ALLXBOTSFEEDBACKOPTION(0))
        info = next((i for i in infos if int(i.xbot_id) == args.xbot), None)
        if info is None:
            raise RuntimeError('selected xbot missing from feedback')
        if info.xbot_state not in (pm.XBOTSTATE.XBOT_IDLE, pm.XBOTSTATE.XBOT_MOTION):
            raise RuntimeError(f'unexpected state: {info.xbot_state.name}')
        if not all(math.isfinite(float(v)) for v in (info.x_pos, info.y_pos, info.rz_pos)):
            raise RuntimeError('nonfinite feedback')
        return info, infos

    try:
        connected = bool(system.connect_to_specific_pmc('127.1.1.1'))
        if not connected:
            raise RuntimeError('cannot connect to local simulator')
        system.gain_mastership()
        master = bool(system.is_master())
        if not master:
            raise RuntimeError('mastership unavailable')
        initial, _ = read()
        if initial.xbot_state != pm.XBOTSTATE.XBOT_IDLE:
            raise RuntimeError('xbot must start idle and levitated')
        origin = (float(initial.x_pos), float(initial.y_pos))
        summary['origin_m'] = origin
        targets = [(origin[0] + 0.02, origin[1] + 0.015), origin]
        start = time.monotonic()
        with (args.output / 'feedback.csv').open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['elapsed_s', 'phase', 'dt_s', 'x_m', 'y_m', 'rz_rad',
                             'target_x_m', 'target_y_m', 'error_m', 'vx_m_s', 'vy_m_s',
                             'state', 'command_latency_s'])
            for phase, target in enumerate(targets):
                print(f'Phase {phase}: target {target}', flush=True)
                phase_start = previous = time.monotonic()
                integral = [0.0, 0.0]
                settled_since = None
                while True:
                    tick = time.monotonic()
                    if tick - phase_start > 15:
                        raise RuntimeError(f'phase {phase} failed to converge within 15 s')
                    if args.restart and attempted:
                        xbot.long_axis_jogging_control(args.xbot, 0.0, 0.0,
                                                      pm.JOGGINGOPERATION.JOGGING_OFF)
                        deadline = time.monotonic() + 0.3
                        while True:
                            stopped, _ = read()
                            if stopped.xbot_state == pm.XBOTSTATE.XBOT_IDLE:
                                break
                            if time.monotonic() > deadline:
                                raise RuntimeError('jog off did not reach idle within 0.3 s')
                            time.sleep(0.005)
                    info, others = read()
                    now = time.monotonic()
                    dt = now - previous
                    previous = now
                    intervals.append(dt)
                    if dt > 0.5:
                        raise RuntimeError('feedback/control interval exceeded 0.5 s')
                    if math.hypot(info.x_pos-origin[0], info.y_pos-origin[1]) > 0.05:
                        raise RuntimeError('left 50 mm experiment radius')
                    for other in others:
                        if int(other.xbot_id) != args.xbot and math.hypot(
                                other.x_pos-info.x_pos, other.y_pos-info.y_pos) < 0.15:
                            raise RuntimeError('another xbot within 150 mm')
                    if not system.is_master():
                        raise RuntimeError('lost mastership')
                    error = [target[0]-info.x_pos, target[1]-info.y_pos]
                    distance = math.hypot(*error)
                    if distance <= 0.0005:
                        velocity = [0.0, 0.0]
                        integral = [0.0, 0.0]
                        if settled_since is None:
                            settled_since = now
                    else:
                        settled_since = None
                        candidate = [max(-0.02, min(0.02, integral[i]+error[i]*dt))
                                     for i in range(2)]
                        velocity = [args.kp*error[i]+args.ki*candidate[i] for i in range(2)]
                        magnitude = math.hypot(*velocity)
                        if magnitude <= 0.02:
                            integral = candidate  # freeze integration during saturation
                        else:
                            velocity = [v*0.02/magnitude for v in velocity]
                    attempted = True
                    sent = time.monotonic()
                    xbot.long_axis_jogging_control(args.xbot, float(velocity[0]),
                                                  float(velocity[1]), pm.JOGGINGOPERATION.JOGGING_ON)
                    latency = time.monotonic()-sent
                    moving_updates += int(info.xbot_state == pm.XBOTSTATE.XBOT_MOTION)
                    writer.writerow([now-start, phase, dt, info.x_pos, info.y_pos, info.rz_pos,
                                     *target, distance, *velocity, info.xbot_state.name, latency])
                    stream.flush()
                    if settled_since is not None and now-settled_since >= 0.5:
                        result = dict(phase=phase, target_m=target, error_m=distance,
                                      elapsed_s=now-phase_start)
                        summary['phases'].append(result)
                        print(result, flush=True)
                        break
                    time.sleep(max(0, 0.05-(time.monotonic()-tick)))
    except (Exception, KeyboardInterrupt) as exc:
        failure = f'{type(exc).__name__}: {exc}'
        print(failure, file=sys.stderr)
    finally:
        cleanup = []
        if attempted:
            cleanup = [('jog off', lambda: xbot.long_axis_jogging_control(
                args.xbot, 0.0, 0.0, pm.JOGGINGOPERATION.JOGGING_OFF)),
                       ('stop', lambda: xbot.stop_motion(args.xbot))]
        for label, action in cleanup:
            try:
                action()
                print(f'{label}: OK', flush=True)
            except Exception as exc:
                failure = f'{failure or ""}; {label} failed: {exc}'
        if attempted:
            try:
                time.sleep(0.5)
                final, _ = read()
                summary['final_position_m'] = [float(final.x_pos), float(final.y_pos)]
                summary['final_state'] = final.xbot_state.name
                summary['final_error_m'] = math.hypot(final.x_pos-origin[0], final.y_pos-origin[1])
                if final.xbot_state != pm.XBOTSTATE.XBOT_IDLE or summary['final_error_m'] > 0.0005:
                    failure = f'{failure or ""}; final idle/position check failed'
            except Exception as exc:
                failure = f'{failure or ""}; final feedback failed: {exc}'
        for label, action, enabled in [('release', system.release_mastership, master),
                                       ('disconnect', system.disconnect_from_pmc, connected)]:
            if enabled:
                try:
                    action()
                except Exception as exc:
                    failure = f'{failure or ""}; {label} failed: {exc}'
        summary.update(success=failure is None and len(summary['phases']) == 2,
                       failure=failure, updates_while_moving=moving_updates,
                       mean_interval_s=sum(intervals)/len(intervals) if intervals else None,
                       max_interval_s=max(intervals) if intervals else None)
        (args.output / 'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
        print(json.dumps(summary, indent=2))
    return 0 if summary['success'] else 1


if __name__ == '__main__':
    sys.exit(main())
