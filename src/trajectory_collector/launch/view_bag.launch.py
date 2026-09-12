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

"""Launch file for trajectory RViz visualization and optional rosbag playback."""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def evaluate_bag_playback(context, *args, **kwargs):
    """Execute ros2 bag play if a bag path was provided."""
    bag_val = context.perform_substitution(LaunchConfiguration('bag')).strip()
    if not bag_val:
        return [
            LogInfo(msg='No bag path provided. RViz launched in playback mode (awaiting data).')
        ]

    bag_path = os.path.expanduser(bag_val)
    rate = context.perform_substitution(LaunchConfiguration('rate')).strip()
    loop = context.perform_substitution(LaunchConfiguration('loop')).strip().lower()

    cmd = ['ros2', 'bag', 'play', bag_path, '--clock']
    if rate and rate != '1.0':
        cmd.extend(['-r', rate])
    if loop in ('true', '1'):
        cmd.append('--loop')

    return [
        LogInfo(msg=f'Starting rosbag playback: {" ".join(cmd)}'),
        ExecuteProcess(
            cmd=cmd,
            output='screen'
        )
    ]


def generate_launch_description():
    pkg_trajectory_collector = FindPackageShare('trajectory_collector')
    pkg_clearpath_viz = FindPackageShare('clearpath_viz')

    bag_arg = DeclareLaunchArgument(
        'bag',
        default_value='',
        description='Path to recorded rosbag folder/file (plays automatically if provided)'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='a200_0000',
        description='Robot namespace for RViz and topics'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        choices=['true', 'false'],
        description='Use playback/simulation clock'
    )

    rate_arg = DeclareLaunchArgument(
        'rate',
        default_value='1.0',
        description='Playback rate multiplier (e.g. 2.0 for 2x speed)'
    )

    loop_arg = DeclareLaunchArgument(
        'loop',
        default_value='false',
        choices=['true', 'false'],
        description='Loop playback continuously if true'
    )

    rviz_config_path = PathJoinSubstitution([
        pkg_trajectory_collector,
        'config',
        'rviz',
        'bag_playback.rviz'
    ])

    viz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_clearpath_viz, 'launch', 'view_navigation.launch.py'])
        ),
        launch_arguments=[
            ('namespace', LaunchConfiguration('namespace')),
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('config', rviz_config_path),
        ]
    )

    bag_playback_action = OpaqueFunction(function=evaluate_bag_playback)

    return LaunchDescription([
        bag_arg,
        namespace_arg,
        use_sim_time_arg,
        rate_arg,
        loop_arg,
        viz_launch,
        bag_playback_action,
    ])
