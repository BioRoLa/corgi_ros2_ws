#!/usr/bin/env python3
"""Record the topics needed to inspect one MPC controller configuration."""

import argparse
import subprocess
from datetime import datetime
from pathlib import Path


CONTROL_TOPICS = (
    '/trigger', '/motor/state', '/motor/command', '/walk/swing_phase',
)
LEGACY_TOPICS = (
    '/imu', '/odometry/legacy/position', '/odometry/legacy/velocity',
    '/odometry/legacy/contact', '/odometry/legacy/z_position_hip',
)


def topics_for(controller, state_source=None, gait=None, contact_source='gait', raw_lidar=False):
    if controller == 'closed':
        if state_source not in ('odom_legacy', 'sim_driver', 'esekf'):
            raise ValueError('closed controller needs a valid state_source')
        if contact_source not in ('gait', 'gmo'):
            raise ValueError('contact_source must be gait or gmo')
        if contact_source == 'gmo' and state_source != 'esekf':
            raise ValueError('gmo contact requires esekf state_source')
        topics = list(CONTROL_TOPICS) + ['/force/state', '/impedance/command']
        if state_source == 'odom_legacy':
            topics.extend(LEGACY_TOPICS)
        elif state_source == 'sim_driver':
            topics.extend(LEGACY_TOPICS)  # controller's fallback state
            topics.extend(('/tf', '/sim/body/velocity'))
        else:
            topics.extend(('/imu_raw', '/ekf', '/gmo/contact_state',
                           '/lidar_odom', '/odom_mapping', '/fusion/bv'))
            if raw_lidar:
                topics.extend(('/livox/lidar', '/livox/imu', '/Odometry'))
        return topics

    if controller == 'open':
        if gait == 'h20_v10':
            return list(CONTROL_TOPICS) + ['/force/state', *LEGACY_TOPICS]
        if gait == 'wlw':
            return list(CONTROL_TOPICS)
        raise ValueError('open controller needs gait h20_v10 or wlw')
    raise ValueError('controller must be closed or open')


def default_output(controller, state_source, gait):
    script = Path(__file__).resolve()
    package_dir = script.parent.parent
    workspace = script.parents[5] if len(script.parents) > 5 else None
    source_dir = workspace / 'src' / 'corgi_mpc' if workspace else None
    if source_dir and source_dir.is_dir():
        package_dir = source_dir
    mode = state_source if controller == 'closed' else gait
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    return package_dir / 'bag' / f'mpc_{mode}_{stamp}'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--controller', choices=('closed', 'open'), required=True)
    parser.add_argument('--state-source', choices=('odom_legacy', 'sim_driver', 'esekf'))
    parser.add_argument('--gait', choices=('h20_v10', 'wlw'))
    parser.add_argument('--contact-source', choices=('gait', 'gmo'), default='gait')
    parser.add_argument('--raw-lidar', action='store_true')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--print-topics', action='store_true')
    args = parser.parse_args()
    try:
        topics = topics_for(args.controller, args.state_source, args.gait,
                            args.contact_source, args.raw_lidar)
    except ValueError as exc:
        parser.error(str(exc))
    if args.print_topics:
        print('\n'.join(topics))
        return
    output = args.output or default_output(args.controller, args.state_source, args.gait)
    output.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(['ros2', 'bag', 'record', *topics, '-o', str(output)], check=True)


if __name__ == '__main__':
    main()
