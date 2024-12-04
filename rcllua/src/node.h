#ifndef RCL_LUA_NODE_H
#define RCL_LUA_NODE_H

#include <lua.h>

extern const char* MT_NODE;

void rcl_lua_add_node_methods (lua_State* L);

#endif  // RCL_LUA_NODE_H
