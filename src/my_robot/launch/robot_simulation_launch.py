# run the following command in the root directory (vacuum...) to launch the robot:
# colcon build --packages-select my_robot && source install/setup.bash && ros2 launch my_robot robot_simulation_launch.py
# open a new terminal and run this command to control the robot:
# ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diffbot_base_controller/cmd_vel_unstamped

import os
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    my_robot_dir = os.path.join(get_package_share_directory('my_robot'))

    world_path = os.path.join(my_robot_dir,'world','hallway.world')
    gazebo_params_path = os.path.join(my_robot_dir,'config','gazebo_params.yaml')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
            launch_arguments={
                'world': world_path,
                'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_path              
            }.items()
        )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'my_bot',
                                '-x', '5.0',
                                '-y', '-0.5'],
                    output='screen')

    # Run the nodes
    return LaunchDescription([
        gazebo,
        spawn_entity,
    ])