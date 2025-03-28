# run the following command in the root directory (vacuum...) to launch the robot:
# colcon build --packages-select my_robot && source install/setup.bash && ros2 launch my_robot robot_simulation_launch.py
# open a new terminal and run this command to control the robot:
# ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped

import os
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_path = os.path.join(get_package_share_directory('my_robot'))
    file_path = os.path.join(pkg_path,'description','robot_simulation.xacro')


    # Use xacro to process the file
    robot_description_raw = xacro.process_file(file_path).toxml()


    # Configure the nodes
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': True,
            }] # add other parameters here if required
    )

    world_path = os.path.join(pkg_path,'world','obstacles.world')
    gazebo_params_path = os.path.join(pkg_path,'config','gazebo_params.yaml')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
            launch_arguments={
                'world': world_path,
                'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_path              
            }.items()
        )
    
    ros2_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros2_mapping'), 'launch'), '/online_async_launch.py']),
            # launch_arguments={}.items()
        )
    
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'my_bot'],
                    output='screen')
    
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )    
    
    twist_mux_params = os.path.join(pkg_path,'config','twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {'use_sim_time' : True}],
        remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')],
    )

    # Run the nodes
    return LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        twist_mux,
        spawn_entity,
        ros2_mapping,
        diff_drive_spawner,
        joint_broad_spawner,
    ])