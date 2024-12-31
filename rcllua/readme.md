# ROS2 library for the Lua language

## Design concepts

Most of the functionality is implemented in shared library **rclbind** with Lua C API. The pure Lua code
is used to define Executor and other objects not presented in **rcl** library, hide some data structures
and make the code closer to the Python/C++ style.

To avoid errors when work with table members, functions are always called with colon, other objects with dot.
```lua
-- i.e.
local t = {a = 1, b = function (self) return 'foo' end}
print(t.a)  -- get object, use dot
print(t:b())  -- call function, use colon
```

Pure Lua does not support multithreading, that's why some related ROS concepts like CallbackGroup are missing.

## Building

To build **rcllua** with all required packages call
```
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

## Dependencies
- Lua 5.3+
- ROS2 Huble
