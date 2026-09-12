# Copyright 2024 Clearpath Robotics / trajectory_collector contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Trajectory sweep runner for batch data collection."""

import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from typing import Any, Dict, List, Optional, Tuple

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError


def validate_output_directory(output_dir: str, overwrite: bool = False) -> None:
    """Ensure output directory is either nonexistent or empty, unless overwrite is True."""
    out_path = Path(output_dir)
    if out_path.exists():
        if not out_path.is_dir():
            raise ValueError(f"Output path '{output_dir}' exists and is not a directory.")
        try:
            has_contents = any(out_path.iterdir())
        except PermissionError:
            has_contents = False
        if has_contents and not overwrite:
            raise FileExistsError(
                f"Output directory '{output_dir}' already exists and is not empty. "
                'Specify a different directory or pass --overwrite to proceed.'
            )
    out_path.mkdir(parents=True, exist_ok=True)


def parse_yaw(traj: Dict[str, Any], prefix: str) -> float:
    """Extract yaw orientation in radians from a trajectory record."""
    rad_key = f'{prefix}_yaw_rad'
    deg_key = f'{prefix}_yaw_deg'
    generic_key = f'{prefix}_yaw'

    if rad_key in traj and traj[rad_key] not in (None, ''):
        return float(traj[rad_key])
    if deg_key in traj and traj[deg_key] not in (None, ''):
        return math.radians(float(traj[deg_key]))
    if generic_key in traj and traj[generic_key] not in (None, ''):
        return float(traj[generic_key])
    return 0.0


def load_waypoints(csv_path: str) -> List[Dict[str, Any]]:
    """Load and validate waypoints from a CSV file."""
    if not os.path.isfile(csv_path):
        raise FileNotFoundError(f"Waypoints file not found: '{csv_path}'")

    waypoints = []
    required = {'start_x', 'start_y', 'goal_x', 'goal_y'}
    with open(csv_path, 'r', newline='', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames:
            raise ValueError(f"Waypoints file '{csv_path}' has no header.")
        missing = required - set(reader.fieldnames)
        if missing:
            raise ValueError(f"Waypoints file '{csv_path}' missing required columns: {missing}")

        for idx, row in enumerate(reader):
            if 'id' not in row or row['id'] in (None, ''):
                row['id'] = str(idx)
            waypoints.append(row)

    if not waypoints:
        raise ValueError(f"Waypoints file '{csv_path}' contains no data rows.")
    return waypoints


def filter_waypoints(
    waypoints: List[Dict[str, Any]],
    start_id: int = 0,
    count: Optional[int] = None
) -> List[Dict[str, Any]]:
    """Filter waypoints by starting ID and count limit."""
    filtered = [w for w in waypoints if int(w.get('id', 0)) >= start_id]
    if count is not None and count > 0:
        filtered = filtered[:count]
    return filtered


def format_status(exit_code: int) -> str:
    """Map exit code to human-readable status string."""
    if exit_code == 0:
        return 'SUCCESS'
    elif exit_code == 1:
        return 'FAILED'
    elif exit_code == 2:
        return 'TIMEOUT'
    elif exit_code == 130:
        return 'ABORTED'
    else:
        return f'ERROR_{exit_code}'


def append_summary_row(summary_path: str, row: Dict[str, Any]) -> None:
    """Append a single trajectory result row to the summary CSV and flush immediately."""
    fieldnames = [
        'id', 'status', 'exit_code', 'duration_s', 'distance',
        'start_x', 'start_y', 'goal_x', 'goal_y', 'bag_path',
    ]
    file_exists = os.path.exists(summary_path) and os.path.getsize(summary_path) > 0
    with open(summary_path, 'a', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        if not file_exists:
            writer.writeheader()
        writer.writerow(row)
        f.flush()


def get_launch_file_path() -> str:
    """Find the collect_trajectory.launch.py path in share or workspace."""
    try:
        pkg_share = get_package_share_directory('trajectory_collector')
        share_launch = os.path.join(pkg_share, 'launch', 'collect_trajectory.launch.py')
        if os.path.isfile(share_launch):
            return share_launch
    except PackageNotFoundError:
        pass

    # Fallback to relative workspace path
    workspace_launch = os.path.abspath(
        os.path.join(os.path.dirname(__file__), '..', 'launch', 'collect_trajectory.launch.py')
    )
    if os.path.isfile(workspace_launch):
        return workspace_launch

    raise FileNotFoundError('Could not locate collect_trajectory.launch.py')


def run_single_trajectory(
    launch_file: str,
    traj: Dict[str, Any],
    output_dir: str,
    timeout: float = 300.0
) -> Tuple[str, int, float, str]:
    """Execute a single trajectory in an isolated process session."""
    traj_id = int(traj['id'])
    bag_name = f'traj_{traj_id:03d}'
    bag_path = os.path.join(output_dir, bag_name)

    start_yaw = parse_yaw(traj, 'start')
    goal_yaw = parse_yaw(traj, 'goal')

    cmd = [
        'ros2', 'launch', launch_file,
        f'start_x:={traj["start_x"]}',
        f'start_y:={traj["start_y"]}',
        f'start_yaw:={start_yaw:.4f}',
        f'goal_x:={traj["goal_x"]}',
        f'goal_y:={traj["goal_y"]}',
        f'goal_yaw:={goal_yaw:.4f}',
        f'timeout:={timeout}',
        f'bag_directory:={os.path.abspath(output_dir)}',
        f'bag_name:={bag_name}',
        'auto_shutdown:=true',
    ]

    start_time = time.monotonic()
    raw_exit_code = 0
    proc = None
    try:
        proc = subprocess.Popen(cmd, start_new_session=True)
        raw_exit_code = proc.wait(timeout=timeout + 60.0)
    except KeyboardInterrupt:
        if proc and proc.poll() is None:
            try:
                os.killpg(proc.pid, signal.SIGINT)
                proc.wait(timeout=15.0)
            except Exception:
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except Exception:
                    pass
        duration = time.monotonic() - start_time
        return 'ABORTED', 130, duration, bag_path
    except subprocess.TimeoutExpired:
        if proc and proc.poll() is None:
            try:
                os.killpg(proc.pid, signal.SIGINT)
                proc.wait(timeout=15.0)
            except Exception:
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except Exception:
                    pass
        duration = time.monotonic() - start_time
        return 'TIMEOUT', 2, duration, bag_path

    duration = time.monotonic() - start_time

    # Inspect status.json written directly by navigate_to_goal node
    status_file = os.path.join(bag_path, 'status.json')
    if os.path.isfile(status_file):
        try:
            with open(status_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
                status = str(data.get('status', 'SUCCESS'))
                exit_code = int(data.get('exit_code', 0))
                duration = float(data.get('duration_s', duration))
                return status, exit_code, duration, bag_path
        except Exception:
            pass

    # If status.json does not exist, navigation never completed
    if raw_exit_code != 0:
        return f'ERROR_{raw_exit_code}', raw_exit_code, duration, bag_path
    return 'INIT_FAILED', 1, duration, bag_path


def parse_args(args: Optional[List[str]] = None) -> argparse.Namespace:
    """Parse command-line arguments for the sweep runner."""
    parser = argparse.ArgumentParser(
        description='Batch runner for Clearpath Husky trajectory collection sweeps.'
    )
    parser.add_argument(
        '-w', '--waypoints',
        default='data/waypoints.csv',
        help='Path to waypoints CSV file (default: data/waypoints.csv)'
    )
    parser.add_argument(
        '-o', '--output-dir',
        default='data/trajectories',
        help='Output directory for rosbags and summary CSV (default: data/trajectories)'
    )
    parser.add_argument(
        '--overwrite',
        action='store_true',
        help='Allow writing into an existing non-empty directory (default: False)'
    )
    parser.add_argument(
        '-n', '--count',
        type=int,
        default=None,
        help='Maximum number of trajectories to run (default: all)'
    )
    parser.add_argument(
        '--start-id',
        type=int,
        default=0,
        help='Trajectory ID to start from, useful for resuming (default: 0)'
    )
    parser.add_argument(
        '--timeout',
        type=float,
        default=300.0,
        help='Maximum duration in seconds per trajectory (default: 300.0 / 5 min)'
    )
    parser.add_argument(
        '--cooldown',
        type=float,
        default=3.0,
        help='Cooldown pause in seconds between runs (default: 3.0)'
    )
    return parser.parse_args(args)


def main(argv: Optional[List[str]] = None) -> int:
    """Run trajectory sweep across waypoints."""
    args = parse_args(argv)

    # 1. Validate output directory
    try:
        validate_output_directory(args.output_dir, overwrite=args.overwrite)
    except (FileExistsError, ValueError) as err:
        print(f'Error: {err}', file=sys.stderr)
        return 1

    # 2. Load waypoints
    try:
        waypoints = load_waypoints(args.waypoints)
    except (FileNotFoundError, ValueError) as err:
        print(f'Error: {err}', file=sys.stderr)
        return 1

    # 3. Filter trajectories
    selected = filter_waypoints(waypoints, start_id=args.start_id, count=args.count)
    if not selected:
        print(
            f'No waypoints selected (start_id={args.start_id}, count={args.count}).',
            file=sys.stderr
        )
        return 0

    # 4. Locate launch file
    try:
        launch_file = get_launch_file_path()
    except FileNotFoundError as err:
        print(f'Error: {err}', file=sys.stderr)
        return 1

    summary_path = os.path.join(args.output_dir, 'sweep_summary.csv')
    total = len(selected)
    print(f'Starting trajectory sweep: {total} run(s) scheduled.')
    print(f'Destination: {os.path.abspath(args.output_dir)}')
    print(f'Timeout per trajectory: {args.timeout:.1f}s')

    for idx, traj in enumerate(selected, start=1):
        traj_id = traj.get('id', idx - 1)
        print(f'\n=== [{idx}/{total}] Trajectory ID {traj_id} ===')
        print(f'  Start: ({traj["start_x"]}, {traj["start_y"]})')
        print(f'  Goal:  ({traj["goal_x"]}, {traj["goal_y"]})')

        status = 'FAILED'
        exit_code = 1
        duration = 0.0
        bag_path = os.path.join(args.output_dir, f'traj_{int(traj_id):03d}')

        try:
            status, exit_code, duration, bag_path = run_single_trajectory(
                launch_file=launch_file,
                traj=traj,
                output_dir=args.output_dir,
                timeout=args.timeout
            )
        except KeyboardInterrupt:
            print('\n[Sweep] Interrupted by user (Ctrl-C).')
            status = 'ABORTED'
            exit_code = 130

        print(f'  Result: {status} (code {exit_code}) in {duration:.1f}s')

        # Append row immediately
        summary_row = {
            'id': traj_id,
            'status': status,
            'exit_code': exit_code,
            'duration_s': f'{duration:.2f}',
            'distance': traj.get('distance', ''),
            'start_x': traj.get('start_x', ''),
            'start_y': traj.get('start_y', ''),
            'goal_x': traj.get('goal_x', ''),
            'goal_y': traj.get('goal_y', ''),
            'bag_path': bag_path,
        }
        append_summary_row(summary_path, summary_row)

        if status == 'ABORTED':
            print('Sweep aborted due to interrupt.')
            return 130

        # Cooldown between runs
        if idx < total and args.cooldown > 0:
            print(f'  Cooling down {args.cooldown:.1f}s before next run...')
            time.sleep(args.cooldown)

    print(f'\nTrajectory sweep finished. Summary saved to: {os.path.abspath(summary_path)}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
