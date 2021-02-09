# SRC2-driving stack


## Launch the sim


```
~/srcp2-final-public/docker/scripts/run_srcp2_final -C ~/srcp2-final-public/config/default_config.yaml
```

## Installing the packages

Open another terminal and login to Docker enviroment.
```
~/srcp2-final-public/docker/scripts/run_comp_final -d -i
source ~/ros_workspace/install/setup.bash
```
Download the SRC2_driving stack. 

```
cd ~/srcp2-final-public/cmp_workspace/src
git clone https://github.com/wvu-navLab/SRC2-driving.git
```

Build and source the packages.

```
cd ..
catkin build
source devel/setup.bash
```

## Running and testing the nodes

Use the launch files substituting the parameter `{rover_type}`:`scout`, `excavator`, `hauler`.

To run the teleop:
```
roslaunch teleop_modes {rover_type}_teleop_modes
```

To run the 4WS controllers:
```
roslaunch driving_control {rover_type}_driving_control
```
