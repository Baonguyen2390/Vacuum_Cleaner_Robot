import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_path = os.path.join(get_package_share_directory('my_robot'))

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

    # Run the nodes
    return LaunchDescription([
        rviz_arg,
        rviz2,
    ])