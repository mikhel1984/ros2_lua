# Examples of ROS2 Lua nodes

The package contains examples how the ROS2 nodes can be created, testd and launched in Lua. It includes the following directories:
- *rcllua_examples* - make nodes with **rcllua**
- *launch* - launch files for some examples
- *tests* - unit test example
- *raw* - make nodes with pure **rclbind** calls

## Prerequisits

The package uses **std_msgs** messages. You could build it standalone in the local workspace or
build all the common interfaces as described in the main [readme](../README.md).

## Build

Call
```
. install/setup.bash
colcon build --symlink-install --packages-select rcllua_examples
```

## Testing (optional)

Call
```
. install/setup.bash
colcon test --packages-select rcllua_examples --event-handlers console_direct+
```

## Execution

### Publisher and subscription

In the first terminal call
```
. install/setup.bash
ros2 run rcllua_examples simple_publisher
```
In the second terminal call
```
. install/setup.bash
ros2 run rcllua_examples simple_subscription
```

It can also be called using launch file
```
. install/setup.bash
ros2 launch rcllua_examples pub_sub.launch.py
```

### Client and service

In the first terminal call
```
. install/setup.bash
ros2 run rcllua_examples simple_service
```
In the second terminal call
```
. install/setup.bash
ros2 run rcllua_examples simple_client
```

