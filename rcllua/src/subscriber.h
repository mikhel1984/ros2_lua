#ifndef RCL_LUA_SUBSCRIBER_H
#define RCL_LUA_SUBSCRIBER_H

#include <lua.h>

enum SubReg {
  SUB_REG_NODE = 1,
  SUB_REG_MT,
  SUB_REG_NEW
};

void rcl_lua_add_subscription_methods (lua_State* L);

#endif  // RCL_LUA_SUBSCRIBER_H
