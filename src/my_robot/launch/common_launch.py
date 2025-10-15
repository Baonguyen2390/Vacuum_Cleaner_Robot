from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    my_robot_dir = get_package_share_directory('my_robot')

    use_sim_time = LaunchConfiguration('use_sim_time')
    xacro_file_dir = LaunchConfiguration('xacro_file_dir')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
    'use_sim_time',
    default_value='False',
    description='Use simulation (Gazebo) clock if true, also used to determine simulated or real robot is launched')

    declare_xacro_file_dir_cmd = DeclareLaunchArgument(
        'xacro_file_dir',
        default_value=PathJoinSubstitution([my_robot_dir, 'description', 'robot.xacro']),
        description="Path to robot's description",
    )
    
    # Use xacro to process the description file
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        xacro_file_dir,
    ])

    # Configure the nodes
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time,
        }]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
    )

    twist_mux_params_dir = os.path.join(my_robot_dir, "config", "twist_mux.yaml")
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params_dir],
        remappings=[('/cmd_vel_out','/diffbot_base_controller/cmd_vel_unstamped')],
    )

    slam_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            my_robot_dir,
            'launch',
            'online_async_launch.py'
        ]),
        launch_arguments={
            'use_sim_time' : use_sim_time,
        }.items()
    )

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([my_robot_dir, 'launch', 'robot_launch.py'])
        ]),
        condition=IfCondition(PythonExpression(['not ', use_sim_time])),
    )

    robot_simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([my_robot_dir, 'launch', 'robot_simulation_launch.py'])
        ]),
        condition=IfCondition(use_sim_time),
    )

    # Delay start of joint_state_broadcaster after `robot_controller`
    # TODO(anyone): This is a workaround for flaky tests. Remove when fixed.
    delayed_actions = [
        joint_state_broadcaster_spawner,
        slam_launch,
    ]
    delayed_actions_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=delayed_actions,
        )
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_xacro_file_dir_cmd)

    # only one of these two actually gets launched, based on use_sim_time
    ld.add_action(robot_launch)
    ld.add_action(robot_simulation_launch)

    ld.add_action(node_robot_state_publisher)
    ld.add_action(twist_mux)
    ld.add_action(robot_controller_spawner)
    ld.add_action(delayed_actions_handler)

    return ld