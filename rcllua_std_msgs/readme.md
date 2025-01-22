# Generate Lua messages for standard ROS interfaces

## Message generation

List of generated message types can be changed in the _CMakeLists.txt_ file. In orther to add
new message group open it and write command
```
rcllua_generate_interfaces(message dependency1 dependency2)
```
List of dependencies is the same as in CMakeLists of the message itself. The node assumes that
the message package have already been generated and all required files can be found in _share_ 
and _install_ directories.
