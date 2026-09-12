# Copyright 2024 TODO
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

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class TrajectoryCollectorNode(Node):

    def __init__(self, **kwargs):
        super().__init__('trajectory_collector', **kwargs)
        try:
            self.declare_parameter('publish_rate', 50.0)
            rate = self.get_parameter('publish_rate').value
            if type(rate) not in (int, float) or not math.isfinite(rate) or rate <= 0:
                raise ValueError('publish_rate must be finite and positive')
            period = 1.0 / rate
            if not math.isfinite(period) or not 1.0 <= period * 1e9 < 2.0 ** 63:
                raise ValueError('publish_rate is outside the timer duration range')
            self.timer = self.create_timer(period, self.timer_callback)
        except Exception:
            # A failed constructor never reaches main()'s node assignment.
            self.destroy_node()
            raise
        self.get_logger().info('Node started')

    def timer_callback(self):
        pass  # Implement your logic here

    # QoS event callbacks — attach to publishers/subscribers via
    # event_callbacks=QoSEventHandler(incompatible_qos_callback=self.on_qos_event)
    def on_qos_event(self, event):
        self.get_logger().warn(
            f'QoS incompatibility detected: {event.last_policy_kind}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TrajectoryCollectorNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node is not None:
                node.destroy_node()
        finally:
            # A signal handler may already have shut down the context.
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
