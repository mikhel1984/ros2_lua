#ifndef RCL_LUA_TIME_H
#define RCL_LUA_TIME_H

#include <lua.h>

#define NSEC_IN_SEC 1000000000

extern const char* MT_TIME;
extern const char* MT_DURATION;

void rcl_lua_add_time_methods (lua_State* L);

#endif  // RCL_LUA_TIME_H
