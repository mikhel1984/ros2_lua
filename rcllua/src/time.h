#ifndef RCL_LUA_TIME_H
#define RCL_LUA_TIME_H

#include <lua.h>

extern const char* MT_TIME;

void rcl_lua_add_time_methods (lua_State* L);

#endif  // RCL_LUA_TIME_H
