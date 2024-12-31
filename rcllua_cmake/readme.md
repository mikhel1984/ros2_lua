# CMake functions and macro

The packages contains CMake components that can be used to simplify work with Lua packages in ROS2.

## Elements

Add Lua library.
```
rcllua_cmake_install_lib(dir_name)
```
Copy files from *dir_name* to *install/package/lib/lua/dir_name*, make bash hook to update LUA_PATH. It is assumed that the library is called in form
```lua
require('dir_name.script')
```

Add C library.
```
rcllua_cmake_install_clib(dir_name lib_name)
```
Install shared library *lib_name* to *install/package/lib/lua/dir_name*, make bash hook to update LUA_CPATH. It is assumed that the library is called in form
```lua
require('dir_name.lib_name')
```

Make executable.
```
rcllua_cmake_executable(path_to_file exec_name)
```
Create new Lua file and update its premission to be called as
```
ros2 run package exec_name
```
The operation is obligatory since scripts can be executed without 'ros2 run'. On the other hand,
it makes the source file callable without updating the LUA_PATH.

