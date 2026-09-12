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

"""Fork of clearpath_gz simulation.launch.py supporting headless mode."""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description with headless simulation support."""
    pkg_clearpath_gz = FindPackageShare('clearpath_gz')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')

    arguments = [
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            choices=['true', 'false'],
            description='Start RViz'
        ),
        DeclareLaunchArgument(
            'world',
            default_value='warehouse',
            description='Gazebo World'
        ),
        DeclareLaunchArgument(
            'setup_path',
            default_value=[EnvironmentVariable('HOME'), '/clearpath/'],
            description='Clearpath setup path'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'x',
            default_value='0.0',
            description='x component of the robot pose'
        ),
        DeclareLaunchArgument(
            'y',
            default_value='0.0',
            description='y component of the robot pose'
        ),
        DeclareLaunchArgument(
            'z',
            default_value='0.3',
            description='z component of the robot pose'
        ),
        DeclareLaunchArgument(
            'yaw',
            default_value='0.0',
            description='yaw component of the robot pose'
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            choices=['true', 'false'],
            description='Run Gazebo in server-only headless mode (-s)'
        ),
    ]

    # Determine Gazebo resource path from sourced ROS packages
    packages_paths = [
        os.path.join(p, 'share')
        for p in os.getenv('AMENT_PREFIX_PATH', '').split(':')
        if p
    ]
    gz_sim_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            PathJoinSubstitution([pkg_clearpath_gz, 'worlds']),
            ':',
            PathJoinSubstitution([pkg_clearpath_gz, 'meshes']),
            ':',
            ':'.join(packages_paths),
        ]
    )

    # 1a. GUI Gazebo simulator
    gui_gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments=[
            ('gz_args', [
                LaunchConfiguration('world'),
                '.sdf -r -v 4 --gui-config ',
                PathJoinSubstitution([pkg_clearpath_gz, 'config', 'gui.config']),
            ])
        ],
        condition=UnlessCondition(LaunchConfiguration('headless'))
    )

    # 1b. Headless Gazebo simulator (server-only)
    headless_gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments=[
            ('gz_args', [
                LaunchConfiguration('world'),
                '.sdf -r -s -v 4',
            ])
        ],
        condition=IfCondition(LaunchConfiguration('headless'))
    )

    # 2. Clock bridge
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    # 3. Clearpath robot spawn
    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_clearpath_gz, 'launch', 'robot_spawn.launch.py'])
        ]),
        launch_arguments=[
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('setup_path', LaunchConfiguration('setup_path')),
            ('world', LaunchConfiguration('world')),
            ('rviz', LaunchConfiguration('rviz')),
            ('x', LaunchConfiguration('x')),
            ('y', LaunchConfiguration('y')),
            ('z', LaunchConfiguration('z')),
            ('yaw', LaunchConfiguration('yaw')),
        ]
    )

    return LaunchDescription(
        arguments + [
            gz_sim_resource_path,
            clock_bridge,
            gui_gz_sim,
            headless_gz_sim,
            robot_spawn,
        ]
    )
