import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    my_robot_dir = os.path.join(get_package_share_directory('my_robot'))

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
    'use_sim_time',
    default_value='False',
    description='Use simulation (Gazebo) clock if true, also used to determine simulated or real robot is launched')

    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=os.path.join(my_robot_dir,'config','view.rviz'),
        description='Absolute path to rviz config file')

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time' : use_sim_time}],
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )  

    # Run the nodes
    return LaunchDescription([
        declare_use_sim_time_cmd,
        rviz_arg,
        rviz2,
    ])