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

#include <rcl/publisher.h>
#include <rcl/node.h>
#include <rmw/types.h>
#include <rosidl_runtime_c/message_type_support_struct.h>

#include <rosidl_luacommon/definition.h>

#include "rcllua/publisher.h"
#include "rcllua/qos.h"
#include "rcllua/node.h"
#include "rcllua/time.h"
#include "rcllua/utils.h"

/** Indices of publisher bindings in register */
enum PubReg {
  /** node reference */
  PUB_REG_NODE = 1,
  /** metatable name */
  PUB_REG_MT,
  /** number of elements + 1*/
  PUB_REG_NUMBER
};

/** Publisher object metatable name. */
const char* MT_PUBLISHER = "ROS2.Publisher";

/**
 * Create publisher object.
 *
 * Table: rclbind
 * Method: new_publisher
 *
 * Arguments:
 * - node object
 * - message type (table)
 * - topic name
 * - qos profile (optional)
 *
 * Return:
 * - publisher object
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_publisher_init (lua_State* L)
{
  /* arg1 - node */
  rcl_node_t* node = luaL_checkudata(L, 1, MT_NODE);

  /* arg2 - message type */
  rosidl_message_type_support_t *ts = NULL;
  /* check table */
  if (lua_istable(L, 2)) {
    if (ROSIDL_LUA_PUSH_TYPESUPPORT(L, 2) == LUA_TLIGHTUSERDATA) {
      ts = lua_touserdata(L, -1);
    }
    lua_pop(L, 1);                       // pop pointer
  }
  if (NULL == ts) {
    luaL_argerror(L, 2, "expected message type");
  }

  /* arg3 - topic name */
  const char* topic = luaL_checkstring(L, 3);

  /* init object */
  rcl_publisher_options_t publisher_opt = rcl_publisher_get_default_options();
  /* arg4 - QoS profile */
  if (!lua_isnoneornil(L, 4)) {
    rmw_qos_profile_t* qos = luaL_checkudata(L, 4, MT_QOS);
    publisher_opt.qos = *qos;
  }
  rcl_publisher_t *publisher = lua_newuserdata(L, sizeof(rcl_publisher_t));  // push object
  *publisher = rcl_get_zero_initialized_publisher();

  rcl_ret_t ret = rcl_publisher_init(publisher, node, ts, topic, &publisher_opt);
  switch (ret) {
    case RCL_RET_OK: break;
    case RCL_RET_TOPIC_NAME_INVALID:
      luaL_error(L, "invalid topic name %s", topic); break;
    default:
      luaL_error(L, "failed to create publisher");
  }

  /* set metamethods */
  luaL_getmetatable(L, MT_PUBLISHER);  // push metatable
  lua_setmetatable(L, -2);             // pop metatable

  /* save publisher dependencies */
  lua_createtable(L, PUB_REG_NUMBER-1, 0);  // push table a
  lua_pushvalue(L, 1);                 // push node
  lua_rawseti(L, -2, PUB_REG_NODE);    // pop node, a[1] = node

  ROSIDL_LUA_PUSH_MT(L, 2);            // push name
  lua_rawseti(L, -2, PUB_REG_MT);      // pop name, a[2] = name

  lua_rawsetp(L, LUA_REGISTRYINDEX, publisher);  // pop table a, reg[pub] = a

  return 1;
}

/**
 * Publisher destructor.
 *
 * Arguments:
 * - publisher object
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_publisher_free (lua_State* L)
{
  /* arg1 - publisher */
  rcl_publisher_t* publisher = lua_touserdata(L, 1);

  /* get node */
  lua_rawgetp(L, LUA_REGISTRYINDEX, publisher);  // push table
  lua_rawgeti(L, -1, PUB_REG_NODE);              // push node
  rcl_node_t* node = lua_touserdata(L, -1);

  /* finalize */
  rcl_ret_t ret = rcl_publisher_fini(publisher, node);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to fini publisher");
  }

  /* free dependencies */
  lua_pushnil(L);
  lua_rawsetp(L, LUA_REGISTRYINDEX, publisher);

  return 0;
}

/**
 * Send message.
 *
 * Table: Publisher
 * Method: publish
 *
 * Arguments:
 * - publisher object
 * - message object
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_publisher_publish (lua_State* L)
{
  /* arg1 - publisher */
  rcl_publisher_t *publisher = luaL_checkudata(L, 1, MT_PUBLISHER);

  /* arg2 - message object */
  lua_rawgetp(L, LUA_REGISTRYINDEX, publisher);  // push table
  lua_rawgeti(L, -1, PUB_REG_MT);                // push metatable name
  const char* mt = lua_tostring(L, -1);
  idl_lua_msg_t *msg = luaL_checkudata(L, 2, mt);

  /* send */
  rcl_ret_t ret = rcl_publish(publisher, ROSIDL_LUA_GET_MSG(msg), NULL);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to publish");
  }

  return 0;
}

/**
 * Get node logger name.
 *
 * Table: Publisher
 * Method: get_logger_name
 *
 * Arguments:
 * - publisher object
 *
 * Return:
 * - logger name
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_publisher_logger_name (lua_State* L)
{
  /* arg1 - publisher */
  rcl_publisher_t* pub = lua_touserdata(L, 1);
  luaL_argcheck(L, NULL != pub, 1, "publisher is expected");

  /* get node */
  lua_rawgetp(L, LUA_REGISTRYINDEX, pub);     // push table
  lua_rawgeti(L, -1, PUB_REG_NODE);           // push node
  rcl_node_t* node = lua_touserdata(L, -1);

  const char* logger_name = rcl_node_get_logger_name(node);
  if (NULL == logger_name) {
    luaL_error(L, "node logger name not set");
  }

  lua_pushstring(L, logger_name);
  return 1;
}

/**
 * Get number of subscriptions.
 *
 * Table: Publisher
 * Method: get_subscription_count
 *
 * Arguments:
 * - publisher object
 *
 * Return:
 * - subscription number.
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_publisher_subscription_count (lua_State* L)
{
  /* arg1 - publisher */
  rcl_publisher_t* pub = lua_touserdata(L, 1);
  luaL_argcheck(L, NULL != pub, 1, "publisher is expected");

  size_t count = 0;
  rcl_ret_t ret = rcl_publisher_get_subscription_count(pub, &count);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to get subscription count");
  }

  lua_pushinteger(L, count);
  return 1;
}

/**
 * Get topic name.
 *
 * Table: Publisher
 * Method: get_topic_name
 *
 * Arguments:
 * - publisher object
 *
 * Return:
 * - topic name.
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_publisher_topic_name (lua_State* L)
{
  /* arg1 - publisher */
  rcl_publisher_t* pub = lua_touserdata(L, 1);
  luaL_argcheck(L, NULL != pub, 1, "publisher is expected");

  const char* topic = rcl_publisher_get_topic_name(pub);
  if (NULL == topic) {
    luaL_error(L, "failed to get topic name");
  }

  lua_pushstring(L, topic);
  return 1;
}

/**
 * Wait untill all published message data is acknowledged.
 *
 * Table: Publisher
 * Method: wait_for_all_acked
 *
 * Arguments:
 * - publisher object
 * - duration object
 *
 * Return:
 * - false when time is out
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_publisher_wait_for_acked (lua_State* L)
{
  /* arg1 - publisher */
  rcl_publisher_t* pub = luaL_checkudata(L, 1, MT_PUBLISHER);
  /* arg2 - duration */
  rcl_duration_t* dur = luaL_checkudata(L, 1, MT_DURATION);

  bool result = true;
  rcl_ret_t ret = rcl_publisher_wait_for_all_acked(pub, dur->nanoseconds);
  switch (ret) {
    case RCL_RET_OK: break;
    case RCL_RET_TIMEOUT:
      result = false;
      break;
    default:
      luaL_error(L, "failed to wait for all acknowledgements");
  }

  lua_pushboolean(L, result);
  return 1;
}

/** List of publisher methods */
static const struct luaL_Reg pub_methods[] = {
  {"publish", rcl_lua_publisher_publish},
  {"get_logger_name", rcl_lua_publisher_logger_name},
  {"get_topic_name", rcl_lua_publisher_topic_name},
  {"get_subscription_count", rcl_lua_publisher_subscription_count},
  {"wait_for_all_acked", rcl_lua_publisher_wait_for_acked},
  {"__gc", rcl_lua_publisher_free},
  {NULL, NULL}
};

/* Add publisher to library */
void rcl_lua_add_publisher_methods (lua_State* L)
{
  /* constructor */
  lua_pushcfunction(L, rcl_lua_publisher_init);   // push function
  lua_setfield(L, -2, "new_publisher");           // pop, lib['new_publisher'] = function

  /* metamethods */
  rcl_lua_utils_add_mt(L, MT_PUBLISHER, pub_methods);
}
