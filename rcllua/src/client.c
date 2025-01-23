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

#include "rcllua/client.h"
#include "rcllua/qos.h"
#include "rcllua/node.h"
#include "rcllua/utils.h"

/** Indices of client bindings in register. */
enum CliReg {
  /** node reference */
  CLI_REG_NODE = 1,
  /** request metatable */
  CLI_REG_MT_REQUEST,
  /** response message constructor */
  CLI_REG_NEW_RESPONSE,
  /** requests */
  CLI_REG_LIST_REQ,
  /** number of elements + 1 */
  CLI_REG_NUMBER
};

/** List of output elements */
enum CliOut {
  /** response message */
  CLI_OUT_RESPONSE = 1,
  /** callback function */
  CLI_OUT_CALLBACK,
  /** number of elements + 1 */
  CLI_OUT_NUMBER
};

/** Client object metatable name. */
const char* MT_CLIENT = "ROS2.Client";

/**
 * Create client object. Save bindings to register.
 *
 * Arguments:
 * - node object
 * - service type (table)
 * - service name
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
  lua_createtable(L, CLI_REG_NUMBER-1, 0);    // push table a
  lua_pushvalue(L, 1);                 // push node
  lua_rawseti(L, -2, CLI_REG_NODE);    // pop node, a[.] = node

  lua_newtable(L);                     // push empty table
  lua_rawseti(L, -2, CLI_REG_LIST_REQ);  // pop table, a[.] = table

  lua_getfield(L, 2, "Request");       // push table b
  lua_getfield(L, -1, "_metatable");   // push name
  lua_rawseti(L, -3, CLI_REG_MT_REQUEST);  // pop name, a[.] = name
  lua_pop(L, 1);                       // pop table b

  lua_getfield(L, 2, "Response");      // push table b
  lua_getfield(L, -1, "_new");         // push function
  lua_rawseti(L, -3, CLI_REG_NEW_RESPONSE);  // pop function, a[.] = function
  lua_pop(L, 1);                       // pop table b

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
 * - callback
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

  /* arg2 - request object */
  lua_rawgetp(L, LUA_REGISTRYINDEX, cli);  // push table a
  lua_rawgeti(L, -1, CLI_REG_MT_REQUEST);  // push metatable name
  const char* mt = lua_tostring(L, -1);
  idl_lua_msg_t* req = luaL_checkudata(L, 2, mt);
  lua_pop(L, 1);                           // pop name

  /* arg3 - callback function */
  luaL_argcheck(L, lua_isfunction(L, 3), 3, "callback is expected");

  /* send */
  int64_t seq_num = 0;
  rcl_ret_t ret = rcl_send_request(cli, req->obj, &seq_num);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to send request");
  }

  /* save callback */
  lua_rawgeti(L, -1, CLI_REG_LIST_REQ);    // push table b
  lua_pushinteger(L, seq_num);             // push key
  lua_pushvalue(L, 3);                     // push value (function)
  lua_rawset(L, -3);                       // pop key, pop value, b[key] = value

  lua_pushinteger(L, seq_num);             // push return value
  return 1;
}

/**
 * Remove pending request.
 *
 * Arguments:
 * - client object
 * - request sequence number
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_client_remove_request (lua_State* L)
{
  /* arg1 - client object */
  rcl_client_t* cli = luaL_checkudata(L, 1, MT_CLIENT);
  /* arg2 - request id */
  luaL_argcheck(L, lua_isinteger(L, 2), 2, "sequence ID is expected");

  /* remove request and callback */
  lua_rawgetp(L, LUA_REGISTRYINDEX, cli);  // push table a
  lua_rawgeti(L, -1, CLI_REG_LIST_REQ);    // push table b
  lua_pushvalue(L, 2);                     // push key (ID)
  lua_pushnil(L);                          // push value
  lua_rawset(L, -3);                       // pop key & value, clear record

  return 0;
}

/** List of client methods */
static const struct luaL_Reg cli_methods[] = {
  {"__gc", rcl_lua_client_free},
  {"service_is_available", rcl_lua_client_service_is_available},
  {"send_request", rcl_lua_client_send_request},
  {"remove_pending_request", rcl_lua_client_remove_request},
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

/* Receive response */
void rcl_lua_client_push_response (lua_State* L, const rcl_client_t* cli)
{
  /* save result into table */
  lua_createtable(L, CLI_OUT_NUMBER-1, 0);  // push table a

  /* prepare response message */
  lua_rawgetp(L, LUA_REGISTRYINDEX, cli);   // push table b (bindings)
  if (lua_isnil(L, -1)) {
    luaL_error(L, "client bindings not found");
  }
  lua_rawgeti(L, -1, CLI_REG_NEW_RESPONSE);   // push constructor from b
  lua_call(L, 0, 1);                        // pop constructor, push message
  idl_lua_msg_t* msg = lua_touserdata(L, -1);

  /* get response */
  rmw_service_info_t header;
  rcl_ret_t ret = rcl_take_response_with_info(cli, &header, msg->obj);
  switch (ret) {
    case RCL_RET_OK: break;
    case RCL_RET_CLIENT_TAKE_FAILED:
      lua_pop(L, 2);  // keep only a
      return;         // {nil, nil}
    default:
      luaL_error(L, "encountered error when taking client response");
  }

  /* check request/callback exists */
  lua_rawgeti(L, -2, CLI_REG_LIST_REQ);    // push table c, callbacks
  lua_pushinteger(L, header.request_id.sequence_number);  // push response sequence
  if (lua_rawget(L, -2) != LUA_TFUNCTION) {               // pop request seq, push callback
    /* callback not found */
    lua_pop(L, 2);
    return;
  }
  lua_rawseti(L, -5, CLI_OUT_CALLBACK);    // pop function, a[.] = callback
  
  /* remove this request */
  lua_pushinteger(L, header.request_id.sequence_number);  // push response sequence
  lua_pushnil(L);                          // push nil
  lua_rawset(L, -3);                       // pop sequence, pop nil, remove callback
  lua_pop(L, 1);                           // pop table c

  lua_rawseti(L, -3, CLI_OUT_RESPONSE);    // pop message, a[.] = response

  lua_pop(L, 1);                           // pop b
  /* keep table 'a' on stack */
}
