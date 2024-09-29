
#ifndef ROSIDL_LUACOMMON_DEFINITION_H
#define ROSIDL_LUACOMMON_DEFINITION_H

#include <lua.h>
#include <lauxlib.h>

typedef enum {
  IDL_LUA_OBJECT = 0,  // must be deleted with fini
  // don't call fini method
  IDL_LUA_PTR = -1,     // pointer to another object
  IDL_LUA_SEQ = -2      // dynamic length
  // positive values for array size
} IDL_LUA_TYPE;

typedef struct {
  void *obj;
  int value;
} idl_lua_msg_t;

#endif  // ROSIDL_LUACOMMON_DEFINITION_H

