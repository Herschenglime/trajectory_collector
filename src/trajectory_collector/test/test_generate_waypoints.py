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

"""Unit tests for the generate_waypoints module."""

import csv
import math
import os
import shutil
import tempfile

import pytest
from trajectory_collector.generate_waypoints import (
    generate_waypoints,
    grid_to_world,
    is_pair_dissimilar,
    resolve_map_path,
    save_waypoints_to_csv,
)


def test_resolve_map_path():
    """Verify preset and custom map path resolution."""
    # Preset 'warehouse' should resolve to an existing yaml
    resolved = resolve_map_path('warehouse')
    assert os.path.isfile(resolved)
    assert resolved.endswith('warehouse.yaml')

    # Non-existent map should raise FileNotFoundError
    with pytest.raises(FileNotFoundError):
        resolve_map_path('completely_nonexistent_map_12345')


def test_grid_to_world():
    """Verify pixel grid coordinates convert correctly to world (x, y)."""
    # Origin at (0, 0, 0), resolution = 0.05, height = 100
    # Center of bottom-left cell: col=0, row=99 -> x = 0.025, y = 0.025
    x, y = grid_to_world(col=0, row=99, height=100, resolution=0.05, origin=(0.0, 0.0, 0.0))
    assert pytest.approx(x, abs=1e-5) == 0.025
    assert pytest.approx(y, abs=1e-5) == 0.025

    # Center of top-right cell: col=99, row=0 -> x = 4.975, y = 4.975
    x, y = grid_to_world(col=99, row=0, height=100, resolution=0.05, origin=(0.0, 0.0, 0.0))
    assert pytest.approx(x, abs=1e-5) == 4.975
    assert pytest.approx(y, abs=1e-5) == 4.975


def test_is_pair_dissimilar():
    """Verify Euclidean separation and reverse pair filtering."""
    existing = [
        (0.0, 0.0, 10.0, 10.0),
    ]

    # Identical pair -> should be rejected
    assert not is_pair_dissimilar((0.1, 0.1, 9.9, 9.9), existing, thresh=1.0)

    # Reverse pair -> should be rejected
    assert not is_pair_dissimilar((10.0, 10.0, 0.0, 0.0), existing, thresh=1.0)

    # Far away pair -> should be accepted
    assert is_pair_dissimilar((20.0, 0.0, 30.0, 10.0), existing, thresh=1.0)


def test_generate_waypoints_and_csv():
    """Verify waypoint generation and CSV export with configurable output."""
    test_dir = tempfile.mkdtemp(dir='/home/pgrau/husky_ws')
    try:
        csv_out = os.path.join(test_dir, 'subfolder', 'test_waypoints.csv')
        png_out = os.path.join(test_dir, 'subfolder', 'test_preview.png')

        waypoints = generate_waypoints(
            map_input='warehouse',
            num_samples=4,
            min_distance=3.0,
            max_distance=15.0,
            clearance_m=0.65,
            seed=42,
            preview_path=png_out
        )

        assert len(waypoints) == 4
        for wp in waypoints:
            dist = wp['distance']
            assert 3.0 <= dist <= 15.0
            # Heading should match direction from start to goal
            dx = wp['goal_x'] - wp['start_x']
            dy = wp['goal_y'] - wp['start_y']
            expected_yaw = math.atan2(dy, dx)
            assert pytest.approx(wp['start_yaw_rad'], abs=1e-3) == expected_yaw

        # Save and verify CSV output
        save_waypoints_to_csv(waypoints, csv_out)
        assert os.path.isfile(csv_out)
        assert os.path.isfile(png_out)

        with open(csv_out, 'r', encoding='utf-8') as f:
            reader = list(csv.DictReader(f))
            assert len(reader) == 4
            assert 'start_x' in reader[0]
            assert 'goal_x' in reader[0]
            assert reader[0]['map'] == 'warehouse'
    finally:
        shutil.rmtree(test_dir, ignore_errors=True)
