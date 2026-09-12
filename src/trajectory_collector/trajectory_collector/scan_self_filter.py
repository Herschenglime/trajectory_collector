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

"""Filters out LiDAR scan returns that fall within the robot's own footprint."""

import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import tf2_ros


class ScanSelfFilter(Node):
    """Filters laser returns landing within the robot footprint to prevent false obstacles."""

    def __init__(self):
        super().__init__('scan_self_filter')

        self.declare_parameter('scan_in', 'sensors/lidar2d_0/scan')
        self.declare_parameter('scan_out', 'sensors/lidar2d_0/scan_filtered')
        self.declare_parameter('base_frame', 'base_link')
        # Default A200 footprint matching costmap definition (length 1.1m, width 0.9m)
        self.declare_parameter(
            'footprint',
            [0.55, 0.45, 0.55, -0.45, -0.55, -0.45, -0.55, 0.45]
        )
        self.declare_parameter('margin', 0.02)

        scan_in = self.get_parameter('scan_in').value
        scan_out = self.get_parameter('scan_out').value
        self.base_frame = self.get_parameter('base_frame').value
        self.margin = float(self.get_parameter('margin').value)

        flat = list(self.get_parameter('footprint').value)
        if len(flat) < 6 or len(flat) % 2 != 0:
            raise ValueError('footprint must be an even number of at least 3 x,y pairs')
        self.poly = np.array(flat, dtype=float).reshape(-1, 2)
        self.poly_inflated = self.inflate_polygon()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.geometry = None
        self.warned = False
        self.reported = None

        self.pub = self.create_publisher(LaserScan, scan_out, qos_profile_sensor_data)
        self.sub = self.create_subscription(
            LaserScan, scan_in, self.on_scan, qos_profile_sensor_data
        )
        self.get_logger().info(
            f'Filtering {scan_in} -> {scan_out} against {self.poly.shape[0]}-point footprint'
        )

    def inflate_polygon(self):
        """Grow the polygon outwards by margin about its centroid."""
        c = self.poly.mean(axis=0)
        d = self.poly - c
        norms = np.linalg.norm(d, axis=1, keepdims=True)
        norms[norms == 0] = 1.0
        return c + d + (d / norms) * self.margin

    @staticmethod
    def inside(poly, pts):
        """Vectorized point-in-polygon test using the even-odd ray casting rule."""
        x, y = pts[:, 0], pts[:, 1]
        result = np.zeros(len(pts), dtype=bool)
        n = len(poly)
        for i in range(n):
            x0, y0 = poly[i]
            x1, y1 = poly[(i + 1) % n]
            straddles = (y0 > y) != (y1 > y)
            with np.errstate(divide='ignore', invalid='ignore'):
                xint = (x1 - x0) * (y - y0) / (y1 - y0) + x0
            result ^= straddles & (x < xint)
        return result

    def resolve_target_frame(self, source_frame):
        """Determine base_frame name, accounting for configured base_frame."""
        if self.base_frame:
            return self.base_frame
        return 'base_link'

    def solve_geometry(self, msg):
        """Compute and cache the rigid LiDAR-to-base_link transform and beam angle vectors."""
        target_frame = self.resolve_target_frame(msg.header.frame_id)
        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame, msg.header.frame_id, rclpy.time.Time()
            )
        except tf2_ros.TransformException as e:
            if not self.warned:
                self.get_logger().warn(
                    f'Waiting for transform {target_frame} -> {msg.header.frame_id}: {e}'
                )
                self.warned = True
            return False

        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y ** 2 + q.z ** 2)
        )

        n = len(msg.ranges)
        bearings = msg.angle_min + np.arange(n) * msg.angle_increment + yaw
        self.geometry = (t.x, t.y, np.cos(bearings), np.sin(bearings), n)
        self.get_logger().info(
            f'LiDAR localized at x={t.x:.3f}, y={t.y:.3f}, yaw={math.degrees(yaw):.1f}° '
            f'in {target_frame}; {n} beams cached'
        )
        return True

    def find_self_returns(self, msg):
        """Generate boolean mask of beams whose returns land inside the robot footprint."""
        lx, ly, cos_b, sin_b, _ = self.geometry
        r = np.asarray(msg.ranges, dtype=float)
        finite = np.isfinite(r) & (r >= msg.range_min) & (r <= msg.range_max)
        r_safe = np.where(finite, r, 0.0)
        pts = np.column_stack((lx + r_safe * cos_b, ly + r_safe * sin_b))
        pts[~finite] = 1e6  # Place invalid points well outside footprint
        return self.inside(self.poly_inflated, pts) & finite

    def on_scan(self, msg):
        """Filter incoming scan and publish processed LaserScan message."""
        if self.geometry is None or self.geometry[4] != len(msg.ranges):
            if not self.solve_geometry(msg):
                # Pass through raw scan until transform is available to avoid starving AMCL/Nav2
                self.pub.publish(msg)
                return

        mask = self.find_self_returns(msg)
        count = int(mask.sum())
        if self.reported is None or abs(count - self.reported) > 4:
            self.reported = count
            self.get_logger().info(
                f'Filtered {count} of {len(msg.ranges)} returns landing inside footprint'
            )

        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max

        ranges = np.array(msg.ranges, dtype=np.float32)
        ranges[mask] = np.inf
        out.ranges = ranges.tolist()

        if msg.intensities:
            intensities = np.array(msg.intensities, dtype=np.float32)
            intensities[mask] = 0.0
            out.intensities = intensities.tolist()

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ScanSelfFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
