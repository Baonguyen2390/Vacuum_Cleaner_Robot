# Copyright 2020 ros2_control Development Team
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
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    my_robot_dir = FindPackageShare('my_robot')

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("my_robot"),
            "config",
            "diffbot_controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("/diffbot_base_controller/odom", "/odom"),
        ],
    )

    rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('my_robot'), 'launch', 'rplidar_a1_launch.py'
            ])
        ]),
    )

    # this is the common parts between real and simulated robot
    common_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([my_robot_dir, 'launch', 'common_launch.py'])),
        launch_arguments={
            'use_sim_time' : 'false',
            'xacro_file_dir' : PathJoinSubstitution([my_robot_dir, 'description', 'robot.xacro'])
        }.items(),
    )

    return LaunchDescription([
        control_node,
        rplidar,
        common_launch,
    ])
