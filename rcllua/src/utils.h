#ifndef RCL_LUA_UTILS_H
#define RCL_LUA_UTILS_H

#include <lua.h>

typedef struct rcl_lua_enum_v
{
  const char* name;
  int value;
} rcl_lua_enum;

void rcl_lua_utils_add_mt (lua_State* L, const char* name, const luaL_Reg* fn);

void rcl_lua_utils_add_enum (lua_State* L, const char* name, const rcl_lua_enum* ps);

#endif  // RCL_LUA_UTILS_H
