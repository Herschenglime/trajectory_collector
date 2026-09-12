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

import os
import signal

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_trajectory_collector = FindPackageShare('trajectory_collector')
    pkg_clearpath_nav2_demos = FindPackageShare('clearpath_nav2_demos')

    # Simulation & environment arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        choices=['true', 'false'],
        description='Use simulation (Gazebo) clock if true'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='/a200_0000',
        description='Robot namespace'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='warehouse',
        description='Gazebo world name'
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([
            pkg_clearpath_nav2_demos,
            'maps',
            'warehouse.yaml'
        ]),
        description='Full path to map yaml file to load'
    )

    setup_path_arg = DeclareLaunchArgument(
        'setup_path',
        default_value=PathJoinSubstitution([EnvironmentVariable('HOME'), 'clearpath']),
        description='Clearpath setup path containing robot.yaml'
    )

    # Start pose arguments (forwarded to bringup.launch.py)
    start_x_arg = DeclareLaunchArgument(
        'start_x',
        default_value='0.0',
        description='Robot spawn and initial pose X coordinate'
    )

    start_y_arg = DeclareLaunchArgument(
        'start_y',
        default_value='0.0',
        description='Robot spawn and initial pose Y coordinate'
    )

    start_yaw_arg = DeclareLaunchArgument(
        'start_yaw',
        default_value='0.0',
        description='Robot spawn and initial pose Yaw orientation (radians)'
    )

    # Goal pose arguments
    goal_x_arg = DeclareLaunchArgument(
        'goal_x',
        default_value='2.0',
        description='Navigation target goal X coordinate'
    )

    goal_y_arg = DeclareLaunchArgument(
        'goal_y',
        default_value='2.0',
        description='Navigation target goal Y coordinate'
    )

    goal_yaw_arg = DeclareLaunchArgument(
        'goal_yaw',
        default_value='0.0',
        description='Navigation target goal Yaw orientation (radians)'
    )

    timeout_arg = DeclareLaunchArgument(
        'timeout',
        default_value='120.0',
        description='Maximum allowed seconds for goal navigation'
    )

    record_bag_arg = DeclareLaunchArgument(
        'record_bag',
        default_value='true',
        choices=['true', 'false'],
        description='Record navigation trajectory and sensors to MCAP bag if true'
    )

    bag_directory_arg = DeclareLaunchArgument(
        'bag_directory',
        default_value=PathJoinSubstitution([
            EnvironmentVariable('HOME'),
            'husky_ws',
            'data',
            'trajectories'
        ]),
        description='Directory path to store recorded rosbags'
    )

    bag_name_arg = DeclareLaunchArgument(
        'bag_name',
        default_value='',
        description='Custom folder name for recorded bag (empty for timestamped auto-name)'
    )

    auto_shutdown_arg = DeclareLaunchArgument(
        'auto_shutdown',
        default_value='false',
        choices=['true', 'false'],
        description='Automatically terminate entire stack when navigation finishes'
    )

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        choices=['true', 'false'],
        description='Run simulation and navigation without GUI (Gazebo server-only, no RViz)'
    )

    # 1. Full system bringup (Gazebo, sim_gate, Nav2, localization, RViz, scan_self_filter)
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_trajectory_collector, 'launch', 'bringup.launch.py'])
        ),
        launch_arguments=[
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('namespace', LaunchConfiguration('namespace')),
            ('world', LaunchConfiguration('world')),
            ('map', LaunchConfiguration('map')),
            ('setup_path', LaunchConfiguration('setup_path')),
            ('x', LaunchConfiguration('start_x')),
            ('y', LaunchConfiguration('start_y')),
            ('yaw', LaunchConfiguration('start_yaw')),
            ('headless', LaunchConfiguration('headless')),
        ]
    )

    # 2. Automated goal navigation node (spawned once AMCL initial pose confirms)
    navigate_to_goal_node = Node(
        package='trajectory_collector',
        executable='navigate_to_goal',
        namespace=LaunchConfiguration('namespace'),
        parameters=[{
            'goal_x': LaunchConfiguration('goal_x'),
            'goal_y': LaunchConfiguration('goal_y'),
            'goal_yaw': LaunchConfiguration('goal_yaw'),
            'timeout': LaunchConfiguration('timeout'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'record_bag': LaunchConfiguration('record_bag'),
            'bag_directory': LaunchConfiguration('bag_directory'),
            'bag_name': LaunchConfiguration('bag_name'),
        }],
        output='screen'
    )

    def on_initial_pose_exit(event, context):
        cmd_str = ' '.join(event.cmd) if event.cmd else ''
        if (
            ('set_initial_pose' in cmd_str or 'set_initial_pose' in event.process_name) and
            event.returncode == 0
        ):
            return [
                LogInfo(msg='AMCL initial pose confirmed. Launching navigate_to_goal node...'),
                navigate_to_goal_node,
            ]
        return None

    launch_goal_handler = RegisterEventHandler(
        OnProcessExit(
            on_exit=on_initial_pose_exit
        )
    )

    def on_navigation_exit(event, context):
        def emulate_ctrl_c(ctx):
            try:
                os.killpg(os.getpgrp(), signal.SIGINT)
            except Exception:
                pass
            return []

        return [
            LogInfo(
                msg=f'Trajectory navigation completed (exit code {event.returncode}). '
                    'Emulating Ctrl-C (SIGINT to process group)...'
            ),
            OpaqueFunction(function=emulate_ctrl_c),
            EmitEvent(event=Shutdown(reason=f'navigation_exit_{event.returncode}')),
        ]

    # Optional automatic shutdown handler when goal node finishes
    auto_shutdown_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=navigate_to_goal_node,
            on_exit=on_navigation_exit
        ),
        condition=IfCondition(LaunchConfiguration('auto_shutdown'))
    )

    return LaunchDescription([
        use_sim_time_arg,
        namespace_arg,
        world_arg,
        map_arg,
        setup_path_arg,
        start_x_arg,
        start_y_arg,
        start_yaw_arg,
        goal_x_arg,
        goal_y_arg,
        goal_yaw_arg,
        timeout_arg,
        record_bag_arg,
        bag_directory_arg,
        bag_name_arg,
        auto_shutdown_arg,
        headless_arg,
        bringup_launch,
        launch_goal_handler,
        auto_shutdown_handler,
    ])
