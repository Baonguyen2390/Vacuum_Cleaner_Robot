# run the following command in the root directory (vacuum...) to launch the robot:
# colcon build --packages-select my_robot && source install/setup.bash && ros2 launch my_robot robot_launch.py
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
    file_path = os.path.join(pkg_path,'description','robot.urdf.xacro')


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

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=os.path.join(pkg_path,'config','urdf.rviz'),
        description='Absolute path to rviz config file')

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        )
    
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'my_bot'],
                    output='screen')
    
    # Run the nodes
    return LaunchDescription([
        node_robot_state_publisher,
        joint_state_publisher_gui,
        rviz_arg,
        rviz2,
        gazebo,
        spawn_entity
    ])