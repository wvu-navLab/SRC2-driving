# TELEOP_MODES PACKAGE

## Running and testing the nodes

1. Use the launch file. Round 1 and 3, `{rover_type}` is `scout`. For round 2 `{rover_type}` is either `excavator` or `hauler`.

```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch teleop_modes {rover_type}_motion_teleop.launch
```

2. Press "Arrow Up" after seeing Awaiting command...

3. Follow the teleop guide on terminal
