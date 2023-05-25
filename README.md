# Package for control stabilizing robot in Gazebo.

## Required packages
* [ROS noetic](http://wiki.ros.org/noetic/Installation) + [Gazebo](https://classic.gazebosim.org/tutorials?tut=install_ubuntu)
* [teeterbot](https://github.com/robustify/teeterbot.git)
* [gui_teleop](https://github.com/MohitShridhar/gui_teleop.git)
-------------------
## How to install
### 1. Clone repository to your ros_ws and build it
```
cd <PATH_TO_ROS_WS>/src
```
```
git clone git@github.com:lakomchik/advctrl_balancing_robot.git
```
```
cd ..
```
```
catkin build
```
```
source devel/setup.bash
```
### 2. Launch the simulation and enjoy!
```
roslaunch advctrl_balancing_robot launch_lqr_control.launch
```

------------------
