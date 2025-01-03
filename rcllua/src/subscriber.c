// Copyright 2025 Stanislav Mikhel
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <lauxlib.h>

#include <rcl/subscription.h>
#include <rcl/node.h>
#include <rcl/error_handling.h>
#include <rmw/types.h>
#include <rosidl_runtime_c/message_type_support_struct.h>

#include <rosidl_luacommon/definition.h>

#include "rcllua/subscriber.h"
#include "rcllua/qos.h"
#include "rcllua/node.h"
#include "rcllua/utils.h"

/** Indices of subscription bindings in register. */
enum SubReg {
  /** node reference */
  SUB_REG_NODE = 1,
  /** metatable name */
  SUB_REG_MT,
  /** message constructor */
  SUB_REG_NEW,
  /** callback function */
  SUB_REG_CALLBACK,
  /** number of elements + 1 */
  SUB_REG_NUMBER
};

/** Indices of output elements. */
enum SubOut {
  /** received message */
  SUB_OUT_MSG = 1,
  /** callback function */
  SUB_OUT_CALLBACK,
  /** number of elements + 1 */
  SUB_OUT_NUMBER
};

/** Subscription object metatable name */
const char* MT_SUBSCRIPTION = "ROS2.Subscription";

/**
 * Create subscription object. Save bindings to register.
 *
 * Arguments:
 * - node object
 * - message type (table)
 * - topic name
 * - callback function: fn(message) -> nil
 * - qos profile (optional)
 *
 * Return:
 * - subscription object
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_subscription_init (lua_State* L)
{
  /* arg1 - node */
  rcl_node_t* node = luaL_checkudata(L, 1, MT_NODE);

  /* arg2 - message type */
  rosidl_message_type_support_t *ts = NULL;
  if (lua_istable(L, 2)) {
    lua_getfield(L, 2, "_type_support");  // push pointer
    if (lua_islightuserdata(L, -1)) {
      ts = lua_touserdata(L, -1);
      lua_pop(L, 1);                      // pop pointer
    }
  }
  if (NULL == ts) {
    luaL_error(L, "message not found");
  }

  /* arg3 - topic name */
  const char* topic = luaL_checkstring(L, 3);
  /* arg4 - callback function */
  luaL_argcheck(L, LUA_TFUNCTION == lua_type(L, 4), 4, "function expected");

  /* create and init subscription */
  rcl_subscription_options_t subscription_ops = rcl_subscription_get_default_options();
  /* arg5 - QoS profile */
  if (!lua_isnoneornil(L, 5)) {
    rmw_qos_profile_t* qos = luaL_checkudata(L, 5, MT_QOS);
    subscription_ops.qos = *qos;
  }
  rcl_subscription_t* subscription = lua_newuserdata(L, sizeof(rcl_subscription_t));
  *subscription = rcl_get_zero_initialized_subscription();

  rcl_ret_t ret = rcl_subscription_init(subscription, node, ts, topic, &subscription_ops);
  switch (ret) {
    case RCL_RET_OK: break;
    case RCL_RET_TOPIC_NAME_INVALID:
      luaL_error(L, "invalid topic name %s", topic); break;
    default:
      luaL_error(L, "failed to create subscription");
  }

  /* set metamethods */
  luaL_getmetatable(L, MT_SUBSCRIPTION);  // push metatable
  lua_setmetatable(L, -2);                // pop metatable

  /* save node reference and metatable */
  lua_createtable(L, SUB_REG_NUMBER-1, 0);   // push table a
  lua_pushvalue(L, 1);                    // push node
  lua_rawseti(L, -2, SUB_REG_NODE);       // pop node, a[1] = node

  lua_getfield(L, 2, "_metatable");       // push name
  lua_rawseti(L, -2, SUB_REG_MT);         // pop name, a[2] = name

  lua_getfield(L, 2, "_new");             // push function
  lua_rawseti(L, -2, SUB_REG_NEW);        // pop function, a[3] = function

  lua_pushvalue(L, 4);                    // push callback
  lua_rawseti(L, -2, SUB_REG_CALLBACK);   // pop function, a[4] = callback
  lua_rawsetp(L, LUA_REGISTRYINDEX, subscription);  // pop table, reg[pub] = a

  return 1;
}

/**
 * Subscription destructor.
 *
 * Arguments:
 * - subscription object
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_subscription_free (lua_State* L)
{
  /* arg1 - subscription */
  rcl_subscription_t* subscription = lua_touserdata(L, 1);

  /* get node */
  lua_rawgetp(L, LUA_REGISTRYINDEX, subscription);  // push table
  lua_rawgeti(L, -1, SUB_REG_NODE);                 // push node
  rcl_node_t* node = lua_touserdata(L, -1);

  /* finalize */
  rcl_ret_t ret = rcl_subscription_fini(subscription, node);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to fini subscription: %s", rcl_get_error_string().str);
    rcl_reset_error();
  }

  /* free dependencies */
  lua_pushnil(L);
  lua_rawsetp(L, LUA_REGISTRYINDEX, subscription);

  return 0;
}

/** List of subscription methods */
static const struct luaL_Reg sub_methods[] = {
  {"__gc", rcl_lua_subscription_free},
  {NULL, NULL}
};

/* Add subscription to library */
void rcl_lua_add_subscription_methods (lua_State* L)
{
  /* constructor */
  lua_pushcfunction(L, rcl_lua_subscription_init);  // push function
  lua_setfield(L, -2, "new_subscription");          // pop, lib['new_subscription'] = function

  /* metamethods */
  rcl_lua_utils_add_mt(L, MT_SUBSCRIPTION, sub_methods);
}

/* Return table {message, callback}. */
void rcl_lua_subscription_push_callback (lua_State* L, const rcl_subscription_t* sub)
{
  /* save result into table */
  lua_createtable(L, SUB_OUT_NUMBER-1, 0);  // push table a

  /* get message constructor */
  lua_rawgetp(L, LUA_REGISTRYINDEX, sub);  // push table b
  if (lua_isnil(L, -1)) {
    luaL_error(L, "subscriber bindings not found");
  }
  lua_rawgeti(L, -1, SUB_REG_NEW);         // push function from b

  /* make empty message */
  lua_call(L, 0, 1);                       // pop function, push message
  idl_lua_msg_t *msg = lua_touserdata(L, -1);

  /* get and save message */
  rmw_message_info_t message_info;
  rcl_ret_t ret = rcl_take(sub, msg->obj, &message_info, NULL);
  switch (ret) {
    case RCL_RET_OK: break;
    case RCL_RET_BAD_ALLOC:
      luaL_error(L, "failed to allocate memory for message"); break;
    default:
      luaL_error(L, "failed to take message from subscription");
  }
  lua_rawseti(L, -3, SUB_OUT_MSG);        // pop message, a[1] = message
  // TODO(Mikhel) save values: message_info.source_timestamp, message_info.received_timestamp

  /* save callback function */
  lua_rawgeti(L, -1, SUB_REG_CALLBACK);   // push function brom b
  lua_rawseti(L, -3, SUB_OUT_CALLBACK);   // pop function, a[2] = callback

  lua_pop(L, 1);                          // pop table b
  /* keep table 'a' on the stack */
}
