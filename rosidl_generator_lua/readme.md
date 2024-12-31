# Lua message generator

## Design concepts

ROS messages are implemented in the form of wrappers over the corresponding C structures.
The benefit of this approach is efficient memory usage in comparison with Lua data types.
But the drawback is access to the message elements through metamethods.

If a message field is a variable length list, its size should be defined explicitly before usage.
Resize operation tries to keep the existing elements and does not initialize the new one. Out-of-range getting returns nil, setting is ignored.

If the message field is also a message or a sequence of elements, then Lua “userdata” has to be created to store the required metamethods. For example, in the case of expression “x = msg.a.b.c” userdata will be generated for the fields “a”, “b” and “c”, if the last one is not a primitive type. Such “userdata” has constant small size and works as a smart pointer, but it is recommended to keep required pointers in variables to increase efficiency.
```lua
-- msg.a[5] - array or list
-- 'slow' version
for i = 1, #msg.a do msg.a[i] = 42 end
-- 'fast' version
local a = msg.a
for i = 1, #a do a[i] = 42 end
```

All interface types (messages, services, actions) are compiled into separate dynamic libraries: package_name.msg, package_name.srv, package_name.action.
Each library knows its dependencies and tries to load it during the ‘require’ procedure.

Access to constants available only through the message 'class'.

## Operations with messages

- comparison (a == b, a ~= b)
- getting size (#a, return nil for non-lists)
- short string description (tostring(a))
- deep copy of nested elements (a.x = b.y, x and y should be of the same type)
- call as function (a(...) -> bool)

The last operation (call) provides several actions depending on the argument type:
- other message - deep copy
(i.e. a(b) means copy b into a)
- key-value table - initialize corresponding message elements
(a{x=1, y=2} is equal to a.x = 1; a.y = 2)
- list of elements - initialize array, resize list if need
(a.z{2,3,4} is equal to a.z[1] = 2, a.z[2] = 3, a.z[3] = 4)
- positive number - resize list
(a.z(4) after previous operation should contain {2, 3, 4, trash})

## Building messages

The package **rosidl_generator_lua** must be build before any ROS2 messages because it sets hook
for automatic Lua code generation.

In order to work with default interfaces (std_msgs, nav_msgs etc.) clone it and build in your local workspace.

