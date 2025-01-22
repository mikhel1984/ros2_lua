# ros2_lua

Collection of packages that allows writing ROS2 nodes in Lua.

## Features

- support for publishers and subscriptions
- support for clients and services
- parameter server
- message generation

## Installation

Assuming ROS2 humble installed to the standard location in Linux, run the following commands:
```sh
source /opt/ros/humble/setup.pash
mkdir -p ~/ws_lua/src && cd ws_lua/src
git clone -b humble https://github.com/mikhel1984/ros2_lua.git
cd ..
colcon build --symlink-install
```

## ROS interfaces

Lua messages for default ROS interfaces can be generated using **rcllua_std_msgs** node. The 
list of messages is defined it its CMakeLists file and could be edited acording the repository
requirements.

## Making ROS2 package in Lua

**ros2_lua** uses **CMake** functionality for all the operations with Lua nodes.

### Create package
```sh
cd src
ros2 pkg create --build-type ament_cmake project_name
```

Lua scripts could be placed in any location inside the project directory, but it is recommended
to make _project_name_ subdirectory and keep files inside it. Later this subdirectory can be added
to LUA_PATH for sharing code with other scripts and writing unit tests.

### Write code

See _rcllua_examples_ for details.

### Update CMakeLists

Add dependencies.
```
find_package(rcllua REQUIRED)
find_package(rcllua_cmake REQUIRED
```

Add files for execution via _ros2 run_ and launch files in form.
```
rcllua_cmake_executable(project_name/script.lua exe_name)
```

The library can be addedd to LUA_PATH with command
```
rcllua_cmake_install_lib(project_name)
```

If there is C/C++ Lua component inside the package, build it as a usual shared library and add to
CPATH_LUA with command
```
rcllua_cmake_install_clib(project_name lib_name)
```

### Unit tests

See _rcllua_unit_ for details.

### Build

Build package as usual, _simlink_ mode is recommended.
```
. install/setup.bash
colcon build --symlink-install --packages-select project_name
```

## Dependencies
- Lua 5.3+
- ROS2 Huble

