#ifndef RCL_LUA_PUBLISHER_H
#define RCL_LUA_PUBLISHER_H

#include <lua.h>

void rcl_lua_add_publisher_methods (lua_State* L);

void rcl_lua_publisher_new (lua_State* L);

#endif  // RCL_LUA_PUBLISHER_H
