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

"""Node to publish initial pose estimate to AMCL and confirm initialization."""

import math
import sys

from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class InitialPosePublisher(Node):
    """Publishes initial pose to AMCL and shuts down after confirmation."""

    def __init__(self):
        super().__init__('set_initial_pose')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('timeout', 30.0)

        x = float(self.get_parameter('x').value)
        y = float(self.get_parameter('y').value)
        yaw = float(self.get_parameter('yaw').value)
        frame_id = self.get_parameter('frame_id').value
        timeout = float(self.get_parameter('timeout').value)

        self.pose_msg = PoseWithCovarianceStamped()
        self.pose_msg.header.frame_id = frame_id
        self.pose_msg.pose.pose.position.x = x
        self.pose_msg.pose.pose.position.y = y
        self.pose_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        self.pose_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # Standard initial AMCL covariance
        self.pose_msg.pose.covariance[0] = 0.25
        self.pose_msg.pose.covariance[7] = 0.25
        self.pose_msg.pose.covariance[35] = 0.068

        self.pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self.on_amcl_pose, 10
        )

        self.timer = self.create_timer(1.0, self.publish_pose)
        self.timeout_timer = self.create_timer(timeout, self.on_timeout)
        self.get_logger().info(
            f'Publishing initial pose (x={x}, y={y}, yaw={yaw}) to initialpose...'
        )

    def publish_pose(self):
        self.pose_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.pose_msg)

    def on_amcl_pose(self, msg):
        self.get_logger().info('AMCL pose confirmed. Initial pose established successfully.')
        sys.exit(0)

    def on_timeout(self):
        self.get_logger().error('Timed out waiting for AMCL pose confirmation.')
        sys.exit(1)


def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
