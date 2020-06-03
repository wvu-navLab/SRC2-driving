# DRIVING_TOOLS PACKAGE

Provides services for driving a robot blindly
Currently supported:
1. Stop
2. Rotate in place
3. Move Forward
4. Circulate Base Station


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

3. Call a service:

```
rosservice call {service_type} {srv_request}
```