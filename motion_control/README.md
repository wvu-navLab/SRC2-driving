# MOTION_CONTROL PACKAGE


## Running and testing the nodes

1. Use the launch file. Round 1 and 3, `{rover_type}` is `scout`. For round 2 `{rover_type}` is either `excavator` or `hauler`.

```
cd ~/srcp2-competitors/ros_workspace/
source devel/setup.bash
roslaunch motion_control {rover_type}_motion_control
```

3. To check if there's volatile in the bucket. Open a new terminal and:

```
cd ~/srcp2-competitors/ros_workspace/
source ~/srcp2-competitors/ros_workspace/install/setup.bash
rostopic echo /excavator_1/bucket_info
```

4. To check if there's volatile in the hauler. Open a new terminal and:

```
cd ~/srcp2-competitors/ros_workspace/
source ~/srcp2-competitors/ros_workspace/install/setup.bash
rostopic echo /hauler_1/bin_info
```
