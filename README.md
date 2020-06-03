# SRC2-driving stack

## Installing the packages

Create a catkin workspace. For instructions on how to create the workspace go here. Download the move_package and move to this new src/ folder. 

```
cd ~/catkin_ws/
mkdir src
git clone https://github.com/wvu-navLab/SRC2-driving.git
```

Compile and build the packages.

```
cd ..
catkin_make
```

## Running and testing the nodes

1. Use the launch file. Round 1 and 3, `{rover_type}` is `scout`. For round 2 `{rover_type}` is either `excavator` or `hauler`.

```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch driving_tools {rover_type}_driving_tools
roslaunch motion_control {rover_type}_motion_control
roslaunch driving_control {rover_type}_driving_control
roslaunch teleop_modes {rover_type}_driving_control
```
