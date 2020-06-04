# DRIVING_TOOLS PACKAGE

Provides services for driving a robot blindly



1. Use the launch file. Round 1 and 3, `{rover_type}` is `scout`. For round 2 `{rover_type}` is either `excavator` or `hauler`.

```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch driving_tools {rover_type}_driving_tools
```

2. Check the services:
```
rosservice list
```

3. Call a service. Currently supported:

1. Stop
```
rosservice call /{rover_type}/stop "enableStop: false"
```

2. Rotate in place
```
rosservice call /{rover_type}/rotate_in_place "throttle: 0.0"
```

3. Move Forward
```
rosservice call /{rover_type}/move_forward "throttle: 0.0"
```

4. Circulate Base Station
```
rosservice call /{rover_type}/circ_base_station "throttle: 0.0
radius: 0.0"
```
