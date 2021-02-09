# SRC2-driving stack


## Launch the sim


```
~/srcp2-final-public/docker/scripts/run_srcp2_final -C ~/srcp2-final-public/config/default_config.yaml
```

## Installing the packages

Open another terminal and download the SRC2_driving stack. 

```
cd ~/srcp2-final-public/cmp_workspace/src
git clone https://github.com/wvu-navLab/SRC2-driving.git
```

Login to Docker enviroment.
```
~/srcp2-final-public/docker/scripts/run_comp_final -d -i
source ~/ros_workspace/install/setup.bash
```

Build and source the packages.

```
cd ~/cmp_workspace/src
catkin build
source devel/setup.bash
```

## Running and testing the nodes

Use the launch files substituting the parameter `{rover_type}`:`scout`, `excavator`, `hauler`.

To run the teleop:
```
export ROS_MASTER_URI=http://172.18.0.2:11311
roslaunch teleop_modes {rover_type}_teleop_modes
```

To run the 4WS controllers:
```
export ROS_MASTER_URI=http://172.18.0.2:11311
roslaunch driving_control {rover_type}_driving_control
```
