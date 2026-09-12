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

import math

import pytest
import rclpy
from trajectory_collector.set_initial_pose import InitialPosePublisher


@pytest.fixture(scope='module', autouse=True)
def init_rclpy():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_initial_pose_node_creation():
    node = InitialPosePublisher()
    assert node.get_name() == 'set_initial_pose'
    assert node.pose_msg.header.frame_id == 'map'
    assert node.pose_msg.pose.pose.position.x == 0.0
    assert node.pose_msg.pose.pose.position.y == 0.0
    assert math.isclose(node.pose_msg.pose.pose.orientation.w, 1.0)
    assert math.isclose(node.pose_msg.pose.pose.orientation.z, 0.0)
    assert node.pose_msg.pose.covariance[0] == 0.25
    node.destroy_node()
