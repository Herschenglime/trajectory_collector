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

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package share lookups
    pkg_clearpath_gz = FindPackageShare('clearpath_gz')
    pkg_clearpath_nav2_demos = FindPackageShare('clearpath_nav2_demos')
    pkg_clearpath_viz = FindPackageShare('clearpath_viz')

    # Launch arguments
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

    x_arg = DeclareLaunchArgument(
        'x',
        default_value='0.0',
        description='Robot spawn and initial pose X coordinate'
    )

    y_arg = DeclareLaunchArgument(
        'y',
        default_value='0.0',
        description='Robot spawn and initial pose Y coordinate'
    )

    yaw_arg = DeclareLaunchArgument(
        'yaw',
        default_value='0.0',
        description='Robot spawn and initial pose Yaw orientation (radians)'
    )

    # 1. Clearpath Gazebo simulation
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_clearpath_gz, 'launch', 'simulation.launch.py'])
        ),
        launch_arguments=[
            ('world', LaunchConfiguration('world')),
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('setup_path', LaunchConfiguration('setup_path')),
            ('x', LaunchConfiguration('x')),
            ('y', LaunchConfiguration('y')),
            ('yaw', LaunchConfiguration('yaw')),
        ]
    )

    # 2. Gate Node (waits for robot to spawn and odometry to publish)
    sim_gate = Node(
        package='trajectory_collector',
        executable='sim_gate',
        parameters=[{
            'topic': PathJoinSubstitution([LaunchConfiguration('namespace'), 'platform/odom']),
            'timeout': 30.0,
        }],
        output='screen'
    )

    # 3. Clearpath Nav2 navigation stack (conditioned on sim_gate)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_clearpath_nav2_demos, 'launch', 'nav2.launch.py'])
        ),
        launch_arguments=[
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('setup_path', LaunchConfiguration('setup_path')),
        ]
    )

    # 4. Clearpath Nav2 localization (conditioned on sim_gate)
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_clearpath_nav2_demos, 'launch', 'localization.launch.py'])
        ),
        launch_arguments=[
            ('map', LaunchConfiguration('map')),
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('setup_path', LaunchConfiguration('setup_path')),
        ]
    )

    # 5. Clearpath RViz visualization (conditioned on sim_gate)
    viz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_clearpath_viz, 'launch', 'view_navigation.launch.py'])
        ),
        launch_arguments=[
            ('namespace', LaunchConfiguration('namespace')),
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
        ]
    )

    # 6. Automatic initial pose publisher (runs with Nav2 after sim_gate opens)
    initial_pose_node = Node(
        package='trajectory_collector',
        executable='set_initial_pose',
        namespace=LaunchConfiguration('namespace'),
        parameters=[{
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'yaw': LaunchConfiguration('yaw'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        output='screen'
    )

    def on_sim_ready(event, context):
        if event.returncode == 0:
            return [nav2_launch, localization_launch, viz_launch, initial_pose_node]
        msg = f'Sim gate failed (exit code {event.returncode}); navigation stack aborted.'
        return [LogInfo(msg=msg)]

    start_nav_event = RegisterEventHandler(
        OnProcessExit(
            target_action=sim_gate,
            on_exit=on_sim_ready
        )
    )

    return LaunchDescription([
        use_sim_time_arg,
        namespace_arg,
        world_arg,
        map_arg,
        setup_path_arg,
        x_arg,
        y_arg,
        yaw_arg,
        simulation_launch,
        sim_gate,
        start_nav_event,
    ])
