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

from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message


@pytest.fixture(scope='module', autouse=True)
def init_rclpy():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_wait_for_message_timeout():
    node = Node('test_gate_timeout')
    success, msg = wait_for_message(Odometry, node, '/non_existent_topic', time_to_wait=0.1)
    assert not success
    assert msg is None
    node.destroy_node()


def test_wait_for_message_success():
    import threading
    import time

    receiver = Node('test_gate_receiver')
    sender = Node('test_gate_sender')
    pub = sender.create_publisher(Odometry, '/test_odom_gate', 10)

    stop_event = threading.Event()

    def publish_loop():
        msg = Odometry()
        msg.header.frame_id = 'odom'
        while not stop_event.is_set():
            pub.publish(msg)
            time.sleep(0.05)

    thread = threading.Thread(target=publish_loop)
    thread.start()

    try:
        success, received_msg = wait_for_message(
            Odometry, receiver, '/test_odom_gate', time_to_wait=5.0)
        assert success
        assert received_msg.header.frame_id == 'odom'
    finally:
        stop_event.set()
        thread.join()
        receiver.destroy_node()
        sender.destroy_node()
