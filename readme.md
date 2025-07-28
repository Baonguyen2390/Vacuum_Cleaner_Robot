# Introduction
A robot that can automatically navigate its enviroment and perform complete coverage (clean the desired area).

# Demonstration
[Youtube link](https://youtu.be/2wPtgp3S4mg)

# System design

![The robot](/images/robot.png)

The robot uses a Lidar sensor as its main source of awareness of the enviroment around it. Thanks to [SLAM tool-box](https://github.com/SteveMacenski/slam_toolbox), mapping and localization can be easily implemented.
![mapped images of my dorm-hall](/images/dorm_hall_viewed_in_rviz.png)

Then, a complete coverage path is generated using [Fields2Cover](https://github.com/Fields2Cover/Fields2Cover).
![Swaths](/images/swaths.png)
![Path](/images/path.png)

Lastly, [Navigation2](https://github.com/ros-navigation/navigation2) helps navigating the robot throught its enviroment and follow the complete coverage path.

# Other parts of this project
This project consists of multiple modules:
- [grid map to polygon conveter](https://github.com/Baonguyen2390/convert_gridmap_to_polygon_using_opencv): used to find the edges of the walls surrounding the robot and the obstacle it needs to avoid.
- [Motor controller](https://github.com/ngocrc3108/Project_02_JGB37): handles the communication between STM32 (used to control the wheels) and the main CPU.
- [Opennav coverage - modified](https://github.com/ngocrc3108/opennav_coverage): A fork of opennav_coverage packages, used to generate the complete coverage path.
