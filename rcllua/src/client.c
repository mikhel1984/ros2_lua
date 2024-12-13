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

#include <rcl/client.h>
#include <rcl/error_handling.h>
#include <rcl/node.h>
#include <rcl/graph.h>
#include <rosidl_runtime_c/service_type_support_struct.h>
#include <rmw/types.h>

#include <rosidl_luacommon/definition.h>

#include "client.h"
#include "qos.h"
#include "node.h"
#include "utils.h"

enum CliReg {
  CLI_REG_NODE = 1,

};

const char* MT_CLIENT = "ROS2.Client";

/**
 * Create client object. Save bindings to register.
 *
 * Arguments:
 * - node object
 * - service type (table)
 * - topic name
 * - qos profile (optional)
 *
 * Return:
 * - client object
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_client_init (lua_State* L)
{
  /* arg1 - node */
  rcl_node_t* node = luaL_checkudata(L, 1, MT_NODE);

  /* arg2 - message type */
  rosidl_service_type_support_t *ts = NULL;
  /* check table */
  if (lua_istable(L, 2)) {
    lua_getfield(L, 2, "_type_support");   // push pointer
    if (lua_islightuserdata(L, -1)) {
      ts = lua_touserdata(L, -1);
      lua_pop(L, 1);                       // pop pointer
    }
  }
  if (NULL == ts) {
    luaL_argerror(L, 2, "expected service type");
  }

  /* arg3 - service name */
  const char* srv_name = luaL_checkstring(L, 3);

  /* init client */
  rcl_client_options_t client_ops = rcl_client_get_default_options();
  /* arg4 - QoS profile */
  if (!lua_isnoneornil(L, 4)) {
    rmw_qos_profile_t* qos = luaL_checkudata(L, 4, MT_QOS);
    client_ops.qos = *qos;
  }
  rcl_client_t *cli = lua_newuserdata(L, sizeof(rcl_client_t));  // push object
  *cli = rcl_get_zero_initialized_client();

  rcl_ret_t ret = rcl_client_init(cli, node, ts, srv_name, &client_ops);
  switch (ret) {
    case RCL_RET_OK: break;
    case RCL_RET_SERVICE_NAME_INVALID:
      luaL_error(L, "invalid service name %s", srv_name); break;
    default:
      luaL_error(L, "failed to create client");
  }

  /* set metamethods */
  luaL_getmetatable(L, MT_CLIENT);  // push metatable
  lua_setmetatable(L, -2);          // pop metatable

  /* save reference objects */
  // TODO need table ?
  lua_createtable(L, 0, 0);            // push table a
  lua_pushvalue(L, 1);                 // push node
  lua_rawseti(L, -2, CLI_REG_NODE);    // pop node, a[1] = node

  lua_rawsetp(L, LUA_REGISTRYINDEX, cli);  // pop table a

  return 1;
}

/**
 * Client destructor.
 *
 * Arguments:
 * - client object
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_client_free (lua_State* L)
{
  /* arg1 - client object */
  rcl_client_t* cli = lua_touserdata(L, 1);

  /* get node */
  lua_rawgetp(L, LUA_REGISTRYINDEX, cli);  // push table
  lua_rawgeti(L, -1, CLI_REG_NODE);        // push node
  rcl_node_t* node = lua_touserdata(L, -1);

  /* finalize */
  rcl_ret_t ret = rcl_client_fini(cli, node);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to fini client: %s", rcl_get_error_string().str);
    rcl_reset_error();
  }

  /* free dependencies */
  lua_pushnil(L);
  lua_rawsetp(L, LUA_REGISTRYINDEX, cli);

  return 0;
}

/**
 * Check if the service is available.
 *
 * Arguments:
 * - client object
 *
 * Return:
 * - true when available
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_client_service_is_available (lua_State* L)
{
   /* arg1 - client object */
  rcl_client_t* cli = luaL_checkudata(L, 1, MT_CLIENT);

  /* get node */
  lua_rawgetp(L, LUA_REGISTRYINDEX, cli);  // push table
  lua_rawgeti(L, -1, CLI_REG_NODE);        // push node
  rcl_node_t* node = lua_touserdata(L, -1);

  bool is_ready = false;
  rcl_ret_t ret = rcl_service_server_is_available(node, cli, &is_ready);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to check service availability");
  }

  lua_pushboolean(L, is_ready);
  return 1;
}

/**
 * Send request.
 *
 * Arguments:
 * - client object
 * - request message
 *
 * Return:
 * - sequence id
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_client_send_request (lua_State* L)
{
  /* arg1 - client object */
  rcl_client_t* cli = luaL_checkudata(L, 1, MT_CLIENT);

  // TODO check type
  idl_lua_msg_t* msg = lua_touserdata(L, 2);

  /* send */
  int64_t seq_num = 0;
  rcl_ret_t ret = rcl_send_request(cli, msg, &seq_num);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to send request");
  }

  lua_pushinteger(L, seq_num);
  return 1;
}

/** List of client methods */
static const struct luaL_Reg cli_methods[] = {
  {"service_is_available", rcl_lua_client_service_is_available},
  {"send_request", rcl_lua_client_send_request},
  {"__gc", rcl_lua_client_free},
  {NULL, NULL}
};

/* Add client to library. */
void rcl_lua_add_client_methods (lua_State* L)
{
  /* constructor */
  lua_pushcfunction(L, rcl_lua_client_init);  // push function
  lua_setfield(L, -2, "new_client");          // pop, lib['new_client'] = function

  /* metamethods */
  rcl_lua_utils_add_mt(L, MT_CLIENT, cli_methods);
}
