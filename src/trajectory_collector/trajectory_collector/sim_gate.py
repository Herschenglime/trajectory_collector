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

import sys

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message


def main(args=None):
    rclpy.init(args=args)
    node = Node('sim_gate')
    node.declare_parameter('topic', '/a200_0000/platform/odom')
    node.declare_parameter('timeout', 30.0)

    topic = node.get_parameter('topic').value
    timeout = float(node.get_parameter('timeout').value)

    node.get_logger().info(f'Sim gate: waiting for {topic} (timeout: {timeout}s)...')
    success = False
    try:
        success, _ = wait_for_message(Odometry, node, topic, time_to_wait=timeout)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if success:
        sys.exit(0)
    sys.exit(1)


if __name__ == '__main__':
    main()
