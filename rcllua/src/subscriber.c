
#include <lauxlib.h>

#include <rcl/subscription.h>
#include <rcl/node.h>
#include <rcl/error_handling.h>

#include <rmw/types.h>
#include <rosidl_runtime_c/message_type_support_struct.h>

#include <rosidl_luacommon/definition.h>

#include "subscriber.h"
#include "node.h"
#include "utils.h"

const char* MT_SUBSCRIPTION = "ROS2.Subscription";

static int rcl_lua_subscription_init (lua_State* L)
{
  /* arg1 - node */
  rcl_node_t* node = luaL_checkudata(L, 1, MT_NODE);

  /* arg2 - message (table) */
  bool is_ref = false;
  rosidl_message_type_support_t *ts = NULL;
  if (lua_istable(L, 2)) {
    lua_getfield(L, 2, "_type_support");
    if (lua_islightuserdata(L, -1)) {
      ts = lua_touserdata(L, -1);
      lua_pop(L, 1);
      is_ref = true;
    }
  }
  if (!is_ref) {
    luaL_argerror(L, 2, "expected message type");
  }
  if (NULL == ts) {
    luaL_error(L, "message not found");
  }

  /* arg3 - topic name */
  const char* topic = luaL_checkstring(L, 3);

  /* arg4 - callback function */
  luaL_argcheck(L, LUA_TFUNCTION == lua_type(L, 4), 4, "function expected");
  // TODO read qos profile

  rcl_subscription_t* subscription = lua_newuserdata(L, sizeof(rcl_subscription_t));
  *subscription = rcl_get_zero_initialized_subscription();

  rcl_subscription_options_t subscription_ops = rcl_subscription_get_default_options();
  // TODO update options with qos

  rcl_ret_t ret = rcl_subscription_init(subscription, node, ts, topic, &subscription_ops);
  switch (ret) {
    case RCL_RET_OK: break;
    case RCL_RET_TOPIC_NAME_INVALID:
      luaL_error(L, "invalid topic name %s", topic); break;
    default:
      luaL_error(L, "failed to create subscription");
  }

  /* set metatable */
  luaL_getmetatable(L, MT_SUBSCRIPTION);  // push metatable
  lua_setmetatable(L, -2);   // pop metatable, set

  /* save node reference and metatable */
  lua_createtable(L, 4, 0);  // push table
  lua_pushvalue(L, 1);  // push node
  lua_rawseti(L, -2, SUB_REG_NODE);   // pop node, set t[1] = node

  lua_getfield(L, 2, "_metatable");  // push name
  lua_rawseti(L, -2, SUB_REG_MT);   // pop name, set t[2] = name

  lua_getfield(L, 2, "_new");  // push function
  lua_rawseti(L, -2, SUB_REG_NEW);   // pop function, set t[3] = function

  lua_pushvalue(L, 4);  // push function
  lua_rawseti(L, -2, SUB_REG_CALLBACK);

  lua_rawsetp(L, LUA_REGISTRYINDEX, subscription);  // pop table, reg[pub] = t

  return 1;
}

static int rcl_lua_subscription_free (lua_State* L)
{
  rcl_subscription_t* subscription = lua_touserdata(L, 1);

  /* get node */
  lua_rawgetp(L, LUA_REGISTRYINDEX, subscription);  
  lua_rawgeti(L, -1, SUB_REG_NODE);                              
  rcl_node_t* node = lua_touserdata(L, -1);
  
  rcl_ret_t ret = rcl_subscription_fini(subscription, node);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to fini subscription: %s", rcl_get_error_string().str);
    rcl_reset_error();
  }

  /* remove node pointer */
  lua_pushnil(L);
  lua_rawsetp(L, LUA_REGISTRYINDEX, subscription);
  
  return 0;
}

void rcl_lua_subscription_callback_and_message (lua_State* L, const rcl_subscription_t* sub)
{
  /* save into table */
  lua_createtable(L, 2, 0);                         // push table a

  /* get message constructor */
  lua_rawgetp(L, LUA_REGISTRYINDEX, sub);  // push table b
  if (lua_isnil(L, -1)) {
    luaL_error(L, "subscriber bindings not found");
  }
  lua_rawgeti(L, -1, SUB_REG_NEW);                  // push function from b

  /* make empty message */
  lua_call(L, 0, 1);                                // pop function, push message
  idl_lua_msg_t *msg = lua_touserdata(L, -1);

  /* get message */
  rmw_message_info_t message_info;
  rcl_ret_t ret = rcl_take(sub, msg->obj, &message_info, NULL);
  switch (ret) {
    case RCL_RET_OK: break;
    case RCL_RET_BAD_ALLOC:
      luaL_error(L, "failed to allocate memory for message"); break;
    default:
      luaL_error(L, "failed to take message from subscription");
  }
  lua_rawseti(L, -3, 1);                  // pop message, a[1] = message

  // TODO save values: message_info.source_timestamp, L, message_info.received_timestamp

  /* callback function */
  lua_rawgeti(L, -1, SUB_REG_CALLBACK);   // push function brom b
  lua_rawseti(L, -3, 2);                  // pop function, a[2] = message

  lua_pop(L, 1);                          // pop table b
  /* keep table a on the stack */
}

static const struct luaL_Reg sub_methods[] = {
  {"__gc", rcl_lua_subscription_free},
  {NULL, NULL}
};

void rcl_lua_add_subscription_methods (lua_State* L)
{
  lua_pushcfunction(L, rcl_lua_subscription_init);
  lua_setfield(L, -2, "subscription_init");

  /* metatable */
  rcl_lua_utils_add_mt(L, MT_SUBSCRIPTION, sub_methods);
}
