#ifndef RCL_LUA_PUBLISHER_H
#define RCL_LUA_PUBLISHER_H

#include <lua.h>

enum PubReg {
  PUB_REG_NODE = 1,
  PUB_REG_MT
};

void rcl_lua_add_publisher_methods (lua_State* L);

#endif  // RCL_LUA_PUBLISHER_H
