# TELEOP_MODES PACKAGE

## Running and testing the nodes

0. Make sure you have launched the motion_control nodes (https://github.com/wvu-navLab/SRC2-driving/tree/master/motion_control).

1. Use the launch file. Round 1 and 3, `{rover_type}` is `scout`. For round 2 `{rover_type}` is either `excavator` or `hauler`.

```
cd ~/srcp2-competitors/ros_workspace/
source devel/setup.bash
roslaunch teleop_modes {rover_type}_teleop_modes.launch
```

2. Press "Arrow Up" after seeing Awaiting command...

3. Follow the teleop guide on terminal