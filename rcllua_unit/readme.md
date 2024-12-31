# Unit tests

## CMake part 

The package uses CTest system for unit test organization. The tests should be organized
as Lua files, each file can have one or several test units and exit with non-zero code in the 
case of errors.

Add the following lines to TEST part of CMakeLists file
```
find_package(rcllua_unit REQUIRED)

rcllua_unit_tests(
  dir_name/test1.lua
  dir_name/test2.lua
  etc
)
``` 

It is assumed that the Lua interpreter can be called as 'lua'. If the name is different (for example, 'lua53')
update the RCLLUA_CALL variable first, e.g.
```
set(RCLLUA_CALL lua53)
rcllua_unit(...)
```

## Lua part

The **rcllua_unit** package includes a simple Lua library for writing unit tests. It is based on
assertion calls, all test units must be defined as member functions of the library. These 
functions can have any names except **run**, **eql**, **eqlf**, **catch**. Call **run**() in 
the end of the file. 
```lua
local rut = require 'rcllua_unit.testing'

function rut:test_1()
  -- first block of tests
  assert(condition1)
  ...
end

function rut:test_2()
  -- second block of tests
  assert(condition2, "message")
  ...
end

-- execute all the tests
rut:run()
```

There are additional methods for convenience:
- **eql(v1, v2)** - when v1 != v2 return message with actual values of the variables
- **eqlf(v1, v2, tol)** - compare floating point numbers with predefined toleratnce (0.001 by default)
- **catch(fn, ...)** - call fn(...), raise error when the function is finished successfully

The **rcllua_unit.testing** library is optional, you may use any other unit test library instead.
