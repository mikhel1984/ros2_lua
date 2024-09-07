#ifndef RCL_LUA_TIMER_H
#define RCL_LUA_TIMER_H

#include <lua.h>

extern const char* MT_TIMER;

void rcl_lua_add_timer_methods (lua_State* L);

#endif  // RCL_LUA_TIMER_H
