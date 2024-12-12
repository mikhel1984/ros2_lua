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
#include <rcl/error_handling.h>
#include <rmw/types.h>
#include <rosidl_runtime_c/message_type_support_struct.h>

#include <rosidl_luacommon/definition.h>

#include "publisher.h"
#include "qos.h"
#include "node.h"
#include "utils.h"

/** Indices of publisher bindings in register */
enum PubReg {
  /** node reference */
  PUB_REG_NODE = 1,
  /** metatable name */
  PUB_REG_MT
};

/** Publisher object metatable name. */
const char* MT_PUBLISHER = "ROS2.Publisher";

/**
 * Create publisher object.
 *
 * Arguments:
 * - node object
 * - message type (table)
 * - topic name
 * - qos profile
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
    lua_getfield(L, 2, "_type_support");   // push pointer
    if (lua_islightuserdata(L, -1)) {
      ts = lua_touserdata(L, -1);
      lua_pop(L, 1);                       // pop pointer
    }
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
  lua_createtable(L, 2, 0);            // push table a
  lua_pushvalue(L, 1);                 // push node
  lua_rawseti(L, -2, PUB_REG_NODE);    // pop node, a[1] = node

  lua_getfield(L, 2, "_metatable");    // push name
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
    luaL_error(L, "failed to fini publisher: %s", rcl_get_error_string().str);
    rcl_reset_error();
  }

  /* free dependencies */
  lua_pushnil(L);
  lua_rawsetp(L, LUA_REGISTRYINDEX, publisher);

  return 0;
}

/**
 * Send message.
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
  rcl_ret_t ret = rcl_publish(publisher, msg->obj, NULL);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to publish");
  }

  return 0;
}

/** List of publisher methods */
static const struct luaL_Reg pub_methods[] = {
  {"publish", rcl_lua_publisher_publish},
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
