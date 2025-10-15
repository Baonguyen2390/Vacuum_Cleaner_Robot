#!/bin/bash

echo "Script executed from: ${PWD}"

gnome-terminal --tab --title="rviz" --command="bash -c 'cd ${PWD} && source install/setup.bash && ros2 launch my_robot robot_view_launch.py use_sim_time:=true'"
gnome-terminal --tab --title="joy" --command="bash -c 'ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_joy -p use_sim_time:=true'"