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

"""Unit tests for the trajectory sweep runner (run_sweep.py)."""

import csv
import math
from pathlib import Path

import pytest

from trajectory_collector.run_sweep import (
    append_summary_row,
    filter_waypoints,
    format_status,
    load_waypoints,
    parse_args,
    parse_yaw,
    validate_output_directory,
)


def test_validate_output_directory_nonexistent(tmp_path: Path):
    """Test that a non-existent output directory is created successfully."""
    target = tmp_path / 'new_dest'
    assert not target.exists()
    validate_output_directory(str(target), overwrite=False)
    assert target.is_dir()


def test_validate_output_directory_existing_empty(tmp_path: Path):
    """Test that an existing empty directory is accepted."""
    target = tmp_path / 'empty_dest'
    target.mkdir()
    validate_output_directory(str(target), overwrite=False)
    assert target.is_dir()


def test_validate_output_directory_non_empty_without_overwrite(tmp_path: Path):
    """Test that an existing non-empty directory raises FileExistsError without overwrite."""
    target = tmp_path / 'busy_dest'
    target.mkdir()
    (target / 'old_file.txt').write_text('content')

    with pytest.raises(FileExistsError) as exc_info:
        validate_output_directory(str(target), overwrite=False)
    assert 'already exists and is not empty' in str(exc_info.value)
    assert '--overwrite' in str(exc_info.value)


def test_validate_output_directory_non_empty_with_overwrite(tmp_path: Path):
    """Test that an existing non-empty directory is accepted when overwrite is True."""
    target = tmp_path / 'busy_dest'
    target.mkdir()
    (target / 'old_file.txt').write_text('content')

    # Should not raise
    validate_output_directory(str(target), overwrite=True)
    assert target.is_dir()


def test_validate_output_directory_path_is_file(tmp_path: Path):
    """Test that passing a path to a regular file raises ValueError."""
    target = tmp_path / 'some_file.txt'
    target.write_text('hello')

    with pytest.raises(ValueError) as exc_info:
        validate_output_directory(str(target), overwrite=True)
    assert 'is not a directory' in str(exc_info.value)


def test_load_waypoints(tmp_path: Path):
    """Test loading waypoints from valid CSV."""
    csv_file = tmp_path / 'waypoints.csv'
    csv_file.write_text(
        'id,start_x,start_y,start_yaw_rad,goal_x,goal_y,goal_yaw_deg,distance\n'
        '0,1.0,2.0,0.5,3.0,4.0,90.0,2.83\n'
        '1,-1.0,-2.0,1.57,-3.0,-4.0,180.0,2.83\n'
    )
    waypoints = load_waypoints(str(csv_file))
    assert len(waypoints) == 2
    assert waypoints[0]['id'] == '0'
    assert float(waypoints[0]['start_x']) == 1.0
    assert float(waypoints[1]['goal_y']) == -4.0


def test_load_waypoints_missing_file():
    """Test loading waypoints from nonexistent file raises FileNotFoundError."""
    with pytest.raises(FileNotFoundError):
        load_waypoints('/path/does/not/exist/waypoints.csv')


def test_load_waypoints_missing_columns(tmp_path: Path):
    """Test that missing required columns raises ValueError."""
    csv_file = tmp_path / 'bad_waypoints.csv'
    csv_file.write_text('id,start_x,goal_x\n0,1.0,2.0\n')
    with pytest.raises(ValueError) as exc_info:
        load_waypoints(str(csv_file))
    assert 'missing required columns' in str(exc_info.value)


def test_filter_waypoints():
    """Test start_id and count filtering logic."""
    waypoints = [
        {'id': '0', 'name': 'zero'},
        {'id': '1', 'name': 'one'},
        {'id': '2', 'name': 'two'},
        {'id': '3', 'name': 'three'},
        {'id': '4', 'name': 'four'},
    ]

    # All
    assert len(filter_waypoints(waypoints)) == 5

    # Start ID
    res_start = filter_waypoints(waypoints, start_id=2)
    assert [w['id'] for w in res_start] == ['2', '3', '4']

    # Count
    res_count = filter_waypoints(waypoints, count=2)
    assert [w['id'] for w in res_count] == ['0', '1']

    # Start ID + Count
    res_both = filter_waypoints(waypoints, start_id=2, count=2)
    assert [w['id'] for w in res_both] == ['2', '3']

    # Out of range start ID
    assert filter_waypoints(waypoints, start_id=10) == []


def test_parse_yaw():
    """Test yaw extraction from various possible key conventions."""
    # From radians key
    traj_rad = {'start_yaw_rad': '1.5708'}
    assert math.isclose(parse_yaw(traj_rad, 'start'), 1.5708, rel_tol=1e-3)

    # From degrees key
    traj_deg = {'goal_yaw_deg': '90.0'}
    assert math.isclose(parse_yaw(traj_deg, 'goal'), math.pi / 2, rel_tol=1e-3)

    # Generic key
    traj_gen = {'start_yaw': '0.785'}
    assert math.isclose(parse_yaw(traj_gen, 'start'), 0.785, rel_tol=1e-3)

    # Missing/empty
    assert parse_yaw({}, 'start') == 0.0
    assert parse_yaw({'start_yaw_rad': ''}, 'start') == 0.0


def test_format_status():
    """Test exit code to status mapping."""
    assert format_status(0) == 'SUCCESS'
    assert format_status(1) == 'FAILED'
    assert format_status(2) == 'TIMEOUT'
    assert format_status(130) == 'ABORTED'
    assert format_status(99) == 'ERROR_99'


def test_append_summary_row(tmp_path: Path):
    """Test summary CSV appending and headers."""
    summary_csv = tmp_path / 'sweep_summary.csv'
    assert not summary_csv.exists()

    row1 = {
        'id': '0',
        'status': 'SUCCESS',
        'exit_code': 0,
        'duration_s': '32.10',
        'distance': '10.5',
        'start_x': '1.0',
        'start_y': '2.0',
        'goal_x': '3.0',
        'goal_y': '4.0',
        'bag_path': '/path/to/bag_000',
    }
    append_summary_row(str(summary_csv), row1)
    assert summary_csv.exists()

    # Verify header and row written
    with open(summary_csv, 'r', newline='', encoding='utf-8') as f:
        reader = list(csv.reader(f))
        assert len(reader) == 2
        assert reader[0][0] == 'id'
        assert reader[1][0] == '0'
        assert reader[1][1] == 'SUCCESS'

    # Append second row
    row2 = dict(row1)
    row2['id'] = '1'
    row2['status'] = 'TIMEOUT'
    row2['exit_code'] = 2
    append_summary_row(str(summary_csv), row2)

    with open(summary_csv, 'r', newline='', encoding='utf-8') as f:
        reader = list(csv.reader(f))
        assert len(reader) == 3  # Header not repeated!
        assert reader[2][0] == '1'
        assert reader[2][1] == 'TIMEOUT'


def test_parse_args():
    """Test CLI argument parsing including --headless flag."""
    # Default behavior
    args_default = parse_args([])
    assert args_default.headless is False
    assert args_default.overwrite is False
    assert args_default.count is None

    # Explicit --headless
    args_headless = parse_args(['--headless', '-n', '5', '--overwrite'])
    assert args_headless.headless is True
    assert args_headless.count == 5
    assert args_headless.overwrite is True
