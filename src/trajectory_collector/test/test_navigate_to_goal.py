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

"""Unit tests for NavigateToGoal node and math helpers."""

import math

import pytest
import rclpy
from trajectory_collector.navigate_to_goal import NavigateToGoal, yaw_to_quaternion


@pytest.fixture(scope='module', autouse=True)
def init_rclpy():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_yaw_to_quaternion():
    """Verify planar yaw to quaternion conversion."""
    # Yaw = 0 -> (0, 0, 0, 1)
    qx, qy, qz, qw = yaw_to_quaternion(0.0)
    assert qx == 0.0
    assert qy == 0.0
    assert qz == 0.0
    assert pytest.approx(qw, abs=1e-6) == 1.0

    # Yaw = pi/2 -> (0, 0, sin(pi/4), cos(pi/4))
    qx, qy, qz, qw = yaw_to_quaternion(math.pi / 2.0)
    assert qx == 0.0
    assert qy == 0.0
    assert pytest.approx(qz, abs=1e-6) == math.sin(math.pi / 4.0)
    assert pytest.approx(qw, abs=1e-6) == math.cos(math.pi / 4.0)

    # Yaw = pi -> (0, 0, 1, 0)
    qx, qy, qz, qw = yaw_to_quaternion(math.pi)
    assert qx == 0.0
    assert qy == 0.0
    assert pytest.approx(qz, abs=1e-6) == 1.0
    assert pytest.approx(qw, abs=1e-6) == 0.0


def test_navigate_to_goal_init():
    """Verify node initializes with expected parameters and interfaces."""
    node = NavigateToGoal()
    try:
        assert node.goal_x == 0.0
        assert node.goal_y == 0.0
        assert node.goal_yaw == 0.0
        assert node.timeout == 120.0
        assert not node.is_done
        assert node.exit_code == 0
    finally:
        node.destroy_node()
