# DRIVING_CONTROL PACKAGE

## Running and testing the nodes

0. Make sure you have launched the motion_control nodes (https://github.com/wvu-navLab/SRC2-driving/tree/master/motion_control).

1. Use the launch file. Round 1 and 3, `{rover_type}` is `scout`. For round 2 `{rover_type}` is either `excavator` or `hauler`.

```
cd ~/srcp2-competitors/ros_workspace/
source devel/setup.bash
roslaunch driving_control {rover_type}_driving_control
```

2. Send a /cmd_vel message with linear.x and angular.z values.

```
rostopic pub /excavator_1/cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" 
```