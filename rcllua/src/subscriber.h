#ifndef RCL_LUA_SUBSCRIBER_H
#define RCL_LUA_SUBSCRIBER_H

#include <lua.h>

struct rcl_subscription_s;

extern const char * MT_SUBSCRIPTION;

enum SubReg {
  SUB_REG_NODE = 1,
  SUB_REG_MT,
  SUB_REG_NEW,
  SUB_REG_CALLBACK
};

void rcl_lua_add_subscription_methods (lua_State* L);

void rcl_lua_subscription_callback_and_message (
  lua_State* L, const struct rcl_subscription_s* sub);

#endif  // RCL_LUA_SUBSCRIBER_H
