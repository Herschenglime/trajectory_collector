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

"""Sends a navigation goal to Nav2 after verifying AMCL localization and bt_navigator lifecycle."""

from datetime import datetime
import json
import math
import os
import sys
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PolygonStamped, PoseWithCovarianceStamped
from lifecycle_msgs.msg import State, TransitionEvent
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import rosbag2_py


def yaw_to_quaternion(yaw: float):
    """Convert planar yaw (radians) to quaternion (x, y, z, w)."""
    half_yaw = yaw * 0.5
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


class NavigateToGoal(Node):
    """Waits for AMCL localization and bt_navigator ACTIVE state, then dispatches goal."""

    def __init__(self):
        super().__init__('navigate_to_goal')

        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_yaw', 0.0)
        self.declare_parameter('timeout', 120.0)
        self.declare_parameter('startup_timeout', 45.0)
        self.declare_parameter('record_bag', True)
        self.declare_parameter('bag_directory', '')
        self.declare_parameter('bag_name', '')
        self.declare_parameter('record_topics', [''])

        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.goal_yaw = float(self.get_parameter('goal_yaw').value)
        self.timeout = float(self.get_parameter('timeout').value)
        self.startup_timeout = float(self.get_parameter('startup_timeout').value)
        self.record_bag = bool(self.get_parameter('record_bag').value)
        self.bag_directory = str(self.get_parameter('bag_directory').value)
        self.bag_name = str(self.get_parameter('bag_name').value)
        self.record_topics = [
            t for t in self.get_parameter('record_topics').value if t
        ]

        self.exit_code = 0
        self.is_done = False
        self._recorder = None
        self._bag_uri = None
        self._start_time = None
        # Since collect_trajectory launches this node only after set_initial_pose
        # exits cleanly with code 0, AMCL localization is already established.
        self._amcl_ready = True
        self._bt_navigator_ready = False
        self._costmap_ready = False
        self._goal_sent = False
        self._goal_handle = None
        self._timeout_timer = None
        self._startup_timer = self.create_timer(
            self.startup_timeout, self._on_startup_timeout
        )
        self._last_feedback_time = 0.0

        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 1. Listen for AMCL poses opportunistically for logging
        self._amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self._on_amcl_pose,
            10
        )

        # 2. Hook into bt_navigator lifecycle transitions
        self.get_logger().info('Waiting for bt_navigator lifecycle to reach ACTIVE state...')
        self._transition_sub = self.create_subscription(
            TransitionEvent,
            'bt_navigator/transition_event',
            self._on_bt_transition,
            10
        )

        # 3. Hook into local costmap footprint to confirm sensor pipeline has updated
        self.get_logger().info(
            'Waiting for local costmap sensor update on local_costmap/published_footprint...'
        )
        self._costmap_sub = self.create_subscription(
            PolygonStamped,
            'local_costmap/published_footprint',
            self._on_costmap_footprint,
            10
        )

        # Initial check in case bt_navigator was already active before subscription
        if self._action_client.server_is_ready():
            self.get_logger().info('bt_navigator action server already active at startup.')
            self._bt_navigator_ready = True
            self._check_ready_and_dispatch()

    def _on_bt_transition(self, msg: TransitionEvent):
        """Handle lifecycle transition events from bt_navigator."""
        if msg.goal_state.id == State.PRIMARY_STATE_ACTIVE or msg.goal_state.label == 'active':
            self.get_logger().info('bt_navigator reached ACTIVE state via transition event.')
            self._bt_navigator_ready = True
            if self._transition_sub is not None:
                self.destroy_subscription(self._transition_sub)
                self._transition_sub = None
            self._check_ready_and_dispatch()

    def _on_costmap_footprint(self, msg: PolygonStamped):
        """Handle first costmap update event indicating sensors are incorporated."""
        if not self._costmap_ready:
            self.get_logger().info('Local costmap sensor processing confirmed.')
            self._costmap_ready = True
            if self._costmap_sub is not None:
                self.destroy_subscription(self._costmap_sub)
                self._costmap_sub = None
            self._check_ready_and_dispatch()

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        """Log AMCL pose updates."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.get_logger().info(f'AMCL localized pose: ({x:.3f}, {y:.3f}).')
        if self._amcl_sub is not None:
            self.destroy_subscription(self._amcl_sub)
            self._amcl_sub = None

    def _check_ready_and_dispatch(self):
        """Dispatch goal once both bt_navigator and costmap are ready."""
        if (
            self._bt_navigator_ready and
            self._costmap_ready and
            not self._goal_sent
        ):
            if self._startup_timer is not None:
                self._startup_timer.cancel()
                self._startup_timer = None
            self._goal_sent = True
            self._start_time = time.monotonic()
            self.get_logger().info(
                f'All prerequisites verified (bt_navigator, costmap). '
                f'Dispatching goal: ({self.goal_x:.3f}, {self.goal_y:.3f}, '
                f'yaw={self.goal_yaw:.3f} rad)'
            )
            self._dispatch_goal()

    def _on_startup_timeout(self):
        """Handle timeout waiting for navigation stack readiness."""
        if self._startup_timer is not None:
            self._startup_timer.cancel()
            self._startup_timer = None
        if self._goal_sent:
            return

        missing = []
        if not self._bt_navigator_ready:
            missing.append('bt_navigator')
        if not self._costmap_ready:
            missing.append('local_costmap')
        self.get_logger().error(
            f'Timed out after {self.startup_timeout:.1f}s waiting for prerequisites: '
            f'{", ".join(missing)}. Navigation aborted.'
        )
        self.exit_code = 2
        self.is_done = True
        self._write_status()

    def _start_recording(self):
        """Initialize and start rosbag recording with MCAP storage backend."""
        try:
            if not self.bag_directory:
                bag_dir = os.path.expanduser('~/husky_ws/data/trajectories')
            else:
                bag_dir = os.path.expanduser(self.bag_directory)
            os.makedirs(bag_dir, exist_ok=True)

            if not self.bag_name:
                bag_name = datetime.now().strftime('traj_%Y%m%d_%H%M%S')
            else:
                bag_name = self.bag_name

            bag_uri = os.path.abspath(os.path.join(bag_dir, bag_name))
            self._bag_uri = bag_uri

            if self.record_topics:
                topics = list(self.record_topics)
            else:
                ns = self.get_namespace().rstrip('/')
                prefix = f'{ns}/' if ns else '/'
                topics = [
                    '/tf',
                    '/tf_static',
                    f'{prefix}platform/odom',
                    f'{prefix}amcl_pose',
                    f'{prefix}cmd_vel',
                    f'{prefix}sensors/lidar2d_0/scan',
                    f'{prefix}sensors/lidar2d_0/scan_filtered',
                    f'{prefix}sensors/camera_0/color/image',
                    f'{prefix}sensors/camera_0/color/camera_info',
                ]
                if ns:
                    topics.extend([f'{ns}/tf', f'{ns}/tf_static'])

            storage_options = rosbag2_py.StorageOptions(
                uri=bag_uri,
                storage_id='mcap'
            )
            record_options = rosbag2_py.RecordOptions()
            record_options.topics = topics
            record_options.disable_keyboard_controls = True
            use_sim_time = True
            if self.has_parameter('use_sim_time'):
                use_sim_time = bool(self.get_parameter('use_sim_time').value)
            record_options.use_sim_time = use_sim_time

            self._recorder = rosbag2_py.Recorder(
                storage_options,
                record_options,
                'info',
                'trajectory_recorder'
            )
            self._recorder.start_spin()
            self._recorder.record()
            self.get_logger().info(
                f'Started recording to {bag_uri} (mcap) for {len(topics)} topics.'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to start rosbag recorder: {e}')
            self._recorder = None

    def _stop_recording(self):
        """Stop rosbag recording and finalize MCAP bag."""
        if self._recorder is not None:
            self.get_logger().info('Stopping rosbag recording and finalizing MCAP bag...')
            try:
                self._recorder.stop()
                self._recorder.stop_spin()
            except Exception as e:
                self.get_logger().warn(f'Error while stopping recorder: {e}')
            finally:
                self._recorder = None

    def _dispatch_goal(self):
        """Send target pose to Nav2 action server."""
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 navigate_to_pose action server not available!')
            self.exit_code = 1
            self.is_done = True
            return

        if self.record_bag:
            self._start_recording()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = self.goal_x
        goal_msg.pose.pose.position.y = self.goal_y
        goal_msg.pose.pose.position.z = 0.0

        qx, qy, qz, qw = yaw_to_quaternion(self.goal_yaw)
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.get_logger().info('Sending goal to navigate_to_pose action server...')
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._on_feedback
        )
        send_goal_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        """Handle goal acceptance or rejection from Nav2."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected by Nav2 action server!')
            self._stop_recording()
            self.exit_code = 1
            self.is_done = True
            return

        self.get_logger().info('Goal accepted by Nav2. Moving to target...')
        self._goal_handle = goal_handle
        # Arm one-shot timeout timer
        self._timeout_timer = self.create_timer(self.timeout, self._on_timeout)
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_feedback(self, feedback_msg):
        """Log navigation progress periodically."""
        now = time.monotonic()
        if now - self._last_feedback_time >= 3.0:
            self._last_feedback_time = now
            feedback = feedback_msg.feedback
            dist = feedback.distance_remaining
            self.get_logger().info(f'Navigating... Distance remaining: {dist:.2f} m')

    def _write_status(self):
        """Write execution status JSON file into the trajectory directory."""
        bag_uri = self._bag_uri
        if not bag_uri:
            bag_dir = os.path.expanduser(self.bag_directory or '~/husky_ws/data/trajectories')
            bag_name = self.bag_name or 'latest'
            bag_uri = os.path.abspath(os.path.join(bag_dir, bag_name))

        os.makedirs(bag_uri, exist_ok=True)
        status_file = os.path.join(bag_uri, 'status.json')

        status_map = {0: 'SUCCESS', 1: 'FAILED', 2: 'TIMEOUT', 130: 'ABORTED'}
        status_str = status_map.get(self.exit_code, f'ERROR_{self.exit_code}')
        duration = (time.monotonic() - self._start_time) if self._start_time else 0.0

        try:
            with open(status_file, 'w', encoding='utf-8') as f:
                json.dump({
                    'status': status_str,
                    'exit_code': self.exit_code,
                    'duration_s': round(duration, 2),
                }, f, indent=2)
        except Exception as e:
            self.get_logger().warn(f'Failed to write status file: {e}')

    def _on_result(self, future):
        """Handle final navigation result."""
        self._stop_recording()
        if self._timeout_timer is not None:
            self._timeout_timer.cancel()
            self._timeout_timer = None

        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal reached successfully!')
            self.exit_code = 0
        else:
            self.get_logger().error(f'Navigation failed with status code: {status}')
            self.exit_code = 1
        self._write_status()
        self.is_done = True

    def _on_timeout(self):
        """Handle navigation timeout event."""
        self.get_logger().error(f'Navigation timed out after {self.timeout:.1f} seconds!')
        self._stop_recording()
        if self._timeout_timer is not None:
            self._timeout_timer.cancel()
            self._timeout_timer = None
        if self._goal_handle is not None:
            self.get_logger().info('Cancelling active goal...')
            self._goal_handle.cancel_goal_async()
        self.exit_code = 2
        self._write_status()
        self.is_done = True

    def destroy_node(self):
        """Clean up timers, subscriptions, and recorder."""
        self._stop_recording()
        if self._startup_timer is not None:
            self._startup_timer.cancel()
            self._startup_timer = None
        if self._timeout_timer is not None:
            self._timeout_timer.cancel()
            self._timeout_timer = None
        self._write_status()
        super().destroy_node()


def main(args=None):
    """Run NavigateToGoal node until goal reached, timeout, or shutdown."""
    rclpy.init(args=args)
    node = NavigateToGoal()

    try:
        while rclpy.ok() and not node.is_done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.exit_code = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    sys.exit(node.exit_code)


if __name__ == '__main__':
    main()
