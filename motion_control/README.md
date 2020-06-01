# MOTION_CONTROL PACKAGE

## Installing the package

Create a catkin workspace. For instructions on how to create the workspace go here. Download the move_package and move to this new src/ folder. 

```
cd ~/catkin_ws/
mkdir src
git clone https://github.com/wvu-navLab/SRC2.git
```


Go to motion_control/CMakeLists.txt and edit it to include the right path to the SRC2 messages and services (change `~/catkin_ws` to where you cloned the SRC2 repo):

```
include_directories(
include
~/<some_directory>/srcp2-competitors/ros_workspace/install/include
${catkin_INCLUDE_DIRS}
)
```

Then, build the catkin workspace. (If it shows an error with four_wheel_steering msgs, you might need to catkin_make again or yell at Çagri).

```
cd ..
catkin_make
```

## Running and testing the nodes

1. To run and test the node you must have ran the simulator:

```
~/<some_directory>/srcp2-competitors/docker/scripts/launch/roslaunch_docker --run-round 2
```

2. Launch the launch file. Round 1 and 3, `{rover_type}` is `scout`. For round 2 `{rover_type}` is either `excavator` or `hauler`.

```
cd ~/catkin_ws/
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
