#!/bin/bash

echo "Script executed from: ${PWD}"

gnome-terminal --tab --title="robot" --command="bash -c 'cd ${PWD} && source install/setup.bash && ros2 launch my_robot robot_launch.py'"
gnome-terminal --tab --title="nav2" --command="bash -c 'cd ${PWD} && source install/setup.bash && ros2 launch my_robot navigation_launch.py use_sim_time:=false'"