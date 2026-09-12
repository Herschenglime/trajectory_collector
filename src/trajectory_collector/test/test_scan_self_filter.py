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

"""Unit tests for the ScanSelfFilter node."""

import numpy as np
import pytest
import rclpy
from sensor_msgs.msg import LaserScan
from trajectory_collector.scan_self_filter import ScanSelfFilter


@pytest.fixture(scope='module', autouse=True)
def init_rclpy():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_point_in_polygon():
    """Verify vectorized even-odd ray casting point-in-polygon logic."""
    poly = np.array([
        [-1.0, -1.0],
        [1.0, -1.0],
        [1.0, 1.0],
        [-1.0, 1.0],
    ])

    pts = np.array([
        [0.0, 0.0],     # inside
        [0.5, -0.5],   # inside
        [2.0, 0.0],    # outside right
        [-2.0, 0.0],   # outside left
        [0.0, 2.0],    # outside above
        [0.0, -2.0],   # outside below
    ])

    inside_mask = ScanSelfFilter.inside(poly, pts)
    expected = np.array([True, True, False, False, False, False])
    np.testing.assert_array_equal(inside_mask, expected)


def test_scan_self_filter_filtering():
    """Verify returns inside footprint are masked, while environmental returns are kept."""
    node = ScanSelfFilter()
    try:
        # Simulate cached LiDAR geometry at x=0.328, y=0.0, yaw=0.0
        # 4 beams: forward (0 rad), left (pi/2 rad), backward (pi rad), right (-pi/2 rad)
        bearings = np.array([0.0, np.pi / 2, np.pi, -np.pi / 2])
        cos_b = np.cos(bearings)
        sin_b = np.sin(bearings)
        node.geometry = (0.328, 0.0, cos_b, sin_b, 4)

        msg = LaserScan()
        msg.angle_min = -np.pi / 2
        msg.angle_max = np.pi
        msg.angle_increment = np.pi / 2
        msg.range_min = 0.05
        msg.range_max = 25.0
        # Beam 0: forward 2.0m (lands at x=2.328, y=0.0 -> outside)
        # Beam 1: left 0.2m (lands at x=0.328, y=0.2 -> inside footprint)
        # Beam 2: backward 0.2m (lands at x=0.128, y=0.0 -> inside footprint)
        # Beam 3: right 2.0m (lands at x=0.328, y=-2.0 -> outside)
        msg.ranges = [2.0, 0.2, 0.2, 2.0]

        mask = node.find_self_returns(msg)
        assert mask[0] is False or mask[0] == 0
        assert mask[1] is True or mask[1] == 1
        assert mask[2] is True or mask[2] == 1
        assert mask[3] is False or mask[3] == 0
    finally:
        node.destroy_node()
