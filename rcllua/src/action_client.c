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

#include <rcl_action/action_client.h>
#include <rcl_action/wait.h>
#include <rosidl_runtime_c/action_type_support_struct.h>
#include <rcl/error_handling.h>
#include <rmw/types.h>
#include <rmw/qos_profiles.h>

#include "rcllua/node.h"
#include "rcllua/qos.h"
#include "rcllua/wait_set.h"
#include "rcllua/utils.h"

/** Indices of action client binding in register. */
enum ActCliReg {
  /** node reference */
  ACT_CLI_REG_NODE = 1,
  /** result request metatable */
  ACT_CLI_REG_MT_RESULT_REQ,
  /** result message constructor */
  ACT_CLI_REG_NEW_RESULT,
  /** result callbacks */
  ACT_CLI_REG_RESULT_LIST,
  /** cancel request */
  ACT_CLI_REG_MT_CANCEL_REQ,
  /** cancel message constructor */
  ACT_CLI_REG_NEW_CANCEL,
  /** cancel callbacks */
  ACT_CLI_REG_CANCEL_LIST,
  /** goal request */
  ACT_CLI_REG_MT_GOAL_REQ,
  /** goal message constructor */
  ACT_CLI_GOAL_NEW_GOAL,
  /** goal callback */
  ACT_CLI_REG_GOAL_LIST,
  /** feedback constructor */
  ACT_CLI_REG_NEW_FEEDBACK,
  /** feedback callback */
  ACT_CLI_REG_FEEDBACK_CB,
  /** status constructor */
  ACT_CLI_REG_NEW_STATUS,
  /** status callback */
  ACT_CLI_REG_STATUS_CB,
  /** number of elements + 1 */
  ACT_CLI_REG_NUMBER
};

/** List of outputs for clients and subscriptions */
enum ActCliOut {
  /** response message */
  ACT_CLI_OUT_RESPONSE = 1,
  /** callback function */
  ACT_CLI_OUT_CALLBACK,
  /** number of elements + 1 */
  ACT_CLI_OUT_NUMBER
};

/** Action client object metatable name. */
const char* MT_ACTION_CLIENT = "ROS2.ActionClient";

/**
 * Create action client object. Save bindings to register.
 *
 * Arguments:
 * - node object
 * - action type (table)
 * - action name
 * - QoS table (={})
 * -- goal_service_qos
 * -- result_service_qos
 * -- cancel_service_qos
 * -- feedback_topic_qos
 * -- status_topic_qos
 *
 * Return:
 * - action client object
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */

static int rcl_lua_action_client_init (lua_State* L)
{
  /* arg1 - node */
  rcl_node_t* node = luaL_checkudata(L, 1, MT_NODE);

  /* arg2 - action type */
  rosidl_action_type_support_t* ts = NULL;
  /* check table */
  if (lua_istable(L, 2)) {
    lua_getfield(L, 2, "_type_support");   // push pointer
    if (lua_islightuserdata(L, -1)) {
      ts = lua_touserdata(L, -1);
      lua_pop(L, 1);                       // pop pointer
    }
  }
  if (NULL == ts) {
    luaL_argerror(L, 2, "expected action type");
  }

  /* arg3 - action service name */
  const char* srv_name = luaL_checkstring(L, 3);

  /* arg4 - QoS table */
  rcl_action_client_options_t action_client_ops = rcl_action_client_get_default_options();
  if (lua_istable(L, 4)) {
    lua_getfield(L, 4, "goal_service_qos");
    if (!lua_isnil(L, -1)) {
      action_client_ops.goal_service_qos = *((rmw_qos_profile_t*) luaL_checkudata(L, -1, MT_QOS));
    }
    lua_pop(L, 1);
    lua_getfield(L, 4, "result_service_qos");
    if (!lua_isnil(L, -1)) {
      action_client_ops.result_service_qos = *((rmw_qos_profile_t*) luaL_checkudata(L, -1, MT_QOS));
    }
    lua_pop(L, 1);
    lua_getfield(L, 4, "cancel_service_qos");
    if (!lua_isnil(L, -1)) {
      action_client_ops.cancel_service_qos = *((rmw_qos_profile_t*) luaL_checkudata(L, -1, MT_QOS));
    }
    lua_pop(L, 1);
    lua_getfield(L, 4, "feedback_topic_qos");
    if (!lua_isnil(L, -1)) {
      action_client_ops.feedback_topic_qos = *((rmw_qos_profile_t*) luaL_checkudata(L, -1, MT_QOS));
    }
    lua_pop(L, 1);
    lua_getfield(L, 4, "status_topic_qos");
    if (!lua_isnil(L, -1)) {
      action_client_ops.status_topic_qos = *((rmw_qos_profile_t*) luaL_checkudata(L, -1, MT_QOS));
    }
    lua_pop(L, 1);
  }

  /* new action client */
  rcl_action_client_t* cli = lua_newuserdata(L, sizeof(rcl_action_client_t));
  *cli = rcl_action_get_zero_initialized_client();

  rcl_ret_t ret = rcl_action_client_init(cli, node, ts, srv_name, &action_client_ops);
  switch (ret) {
    case RCL_RET_OK: break;
    case RCL_RET_ACTION_NAME_INVALID:
      luaL_error(L, "Invalid action name: %s", srv_name); break;
    default:
      luaL_error(L, "Failed to create action client: %s", rcl_get_error_string().str);
  }

  /* set metamethods */
  luaL_getmetatable(L, MT_ACTION_CLIENT);  // push metatable
  lua_setmetatable(L, -2);                 // pop metatable

  /* save reference objects */
  lua_createtable(L, 0, ACT_CLI_REG_NUMBER-1);  // push table a

  lua_pushvalue(L, 1);                   // push node
  lua_rawseti(L, -2, ACT_CLI_REG_NODE);  // pop node

  lua_newtable(L);                       // push result list
  lua_rawseti(L, -2, ACT_CLI_REG_RESULT_LIST);  // pop results

  lua_newtable(L);                       // push cancel list
  lua_rawseti(L, -2, ACT_CLI_REG_CANCEL_LIST);  // pop cancels

  lua_newtable(L);                       // push goal list
  lua_rawseti(L, -2, ACT_CLI_REG_GOAL_LIST);  // pop goals

  lua_rawsetp(L, LUA_REGISTRYINDEX, cli);  // pop table, save to registry

  return 1;
}

/**
 * Action client destructor.
 *
 * Arguments:
 * - action client object
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_action_client_free (lua_State* L)
{
  /* arg1 - action client object */
  rcl_action_client_t* cli = lua_touserdata(L, 1);

  /* get node */
  lua_rawgetp(L, LUA_REGISTRYINDEX, cli);  // push table
  lua_rawgeti(L, -1, ACT_CLI_REG_NODE);        // push node
  rcl_node_t* node = lua_touserdata(L, -1);

  /* finalize */
  rcl_ret_t ret = rcl_action_client_fini(cli, node);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to fini action client: %s", rcl_get_error_string().str);
  }

  /* free dependencies */
  lua_pushnil(L);
  lua_rawsetp(L, LUA_REGISTRYINDEX, cli);

  return 0;
}

/**
 * Request template.
 * \param Type Service type.
 * \param MT_ID Metatable index in register.
 * \param CB_ID Index of callbacks in register.
 */
#define SEND_SERVICE_REQUEST(Type, MT_ID, CB_ID) \
  /* arg1 - action client */ \
  rcl_action_client_t* cli = luaL_checkudata(L, 1, MT_ACTION_CLIENT); \
  /* arg2 - request */ \
  lua_rawgetp(L, LUA_REGISTRYINDEX, cli); \
  lua_rawgeti(L, -1, MT_ID); \
  const char* mt = lua_tostring(L, -1); \
  idl_lua_msg_t* req = luaL_checkudata(L, 2, mt); \
  lua_pop(L, 1); \
  /* arg3 - callback */ \
  luaL_argcheck(L, lua_isfunction(L, 3), 3, "callback is expected"); \
  /* send */ \
  int64_t seq_num = 0; \
  rcl_ret_t ret = rcl_action_send_ ## Type ## _request(cli, req->obj, &seq_num); \
  if (RCL_RET_OK != ret) { \
    luaL_error(L, "failed to send " #Type " request"); \
  } \
  /* save callback */ \
  lua_rawgeti(L, -1, CB_ID); \
  lua_pushinteger(L, seq_num); \
  lua_pushvalue(L, 3); \
  lua_rawset(L, -3); \
  /* return ID */ \
  lua_pushinteger(L, seq_num); \
  return 1;

/**
 * Send request to result service.
 *
 * Arguments:
 * - action client object
 * - request message
 * - callback
 *
 * Return:
 * - sequence id
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_action_client_send_result_request (lua_State* L)
{
  SEND_SERVICE_REQUEST(result, ACT_CLI_REG_MT_RESULT_REQ, ACT_CLI_REG_RESULT_LIST)
}

/**
 * Send request to cancel service.
 *
 * Arguments:
 * - action client object
 * - request message
 * - callback
 *
 * Return:
 * - sequence id
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_action_client_send_cancel_request (lua_State* L)
{
  SEND_SERVICE_REQUEST(cancel, ACT_CLI_REG_MT_CANCEL_REQ, ACT_CLI_REG_CANCEL_LIST)
}

/**
 * Send request to goal service.
 *
 * Arguments:
 * - action client object
 * - request message
 * - callback
 *
 * Return:
 * - sequence id
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_action_client_send_goal_request (lua_State* L)
{
  SEND_SERVICE_REQUEST(goal, ACT_CLI_REG_MT_GOAL_REQ, ACT_CLI_REG_GOAL_LIST)
}

#define TAKE_SERVICE_RESPONSE(Type, NEW_ID, CB_ID) \
  /* arg1 - action client */ \
  rcl_action_client_t* cli = luaL_checkudata(L, 1, MT_ACTION_CLIENT); \
  /* save result into table */ \
  lua_createtable(L, ACT_CLI_OUT_NUMBER-1, 0); \
  /* prepare response message */ \
  lua_rawgetp(L, LUA_REGISTRYINDEX, cli); \
  lua_rawgeti(L, -1, NEW_ID); \
  lua_call(L, 0, 1); \
  idl_lua_msg_t* msg = lua_touserdata(L, -1); \
  /* get response */ \
  rmw_request_id_t header; \
  rcl_ret_t ret = rcl_action_take_ ## Type ## _response(cli, &header, msg->obj); \
  switch (ret) { \
    case RCL_RET_OK: break; \
    case RCL_RET_ACTION_CLIENT_TAKE_FAILED: \
    case RCL_RET_ACTION_SERVER_TAKE_FAILED: \
      lua_pop(L, 2); \
      return 1; \
    default: \
      luaL_error(L, "failed to take " #Type); \
  } \
  /* check request id */ \
  lua_rawgeti(L, -2, CB_ID); \
  lua_pushinteger(L, header.sequence_number); \
  if (lua_rawget(L, -2) != LUA_TFUNCTION) { \
    lua_pop(L, 2); \
    return 1; \
  } \
  lua_rawseti(L, -5, ACT_CLI_OUT_CALLBACK); \
  /* remove request */ \
  lua_pushinteger(L, header.sequence_number); \
  lua_pushnil(L); \
  lua_rawset(L, -3); \
  lua_pop(L, 1); \
  /* response */ \
  lua_rawseti(L, -3, ACT_CLI_OUT_RESPONSE); \
  lua_pop(L, 1); \
  return 1;

/**
 * Get response from result service.
 * Push to the stack {response, callback}.
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_action_client_get_result_response (lua_State* L)
{
  TAKE_SERVICE_RESPONSE(result, ACT_CLI_REG_NEW_RESULT, ACT_CLI_REG_RESULT_LIST)
}

/**
 * Get response from cancel service.
 * Push to the stack {response, callback}.
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_action_client_get_cancel_response (lua_State* L)
{
  TAKE_SERVICE_RESPONSE(cancel, ACT_CLI_REG_NEW_CANCEL, ACT_CLI_REG_CANCEL_LIST)
}

/**
 * Get response from goal service.
 * Push to the stack {response, callback}.
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_action_client_get_goal_response (lua_State* L)
{
  TAKE_SERVICE_RESPONSE(goal, ACT_CLI_REG_NEW_GOAL, ACT_CLI_REG_GOAL_LIST)
}

#define TAKE_MESSAGE(Type, NEW_ID, CB_ID) \
  /* arg1 - action client */ \
  rcl_action_client_t* cli = luaL_checkudata(L, 1, MT_ACTION_CLIENT); \
  lua_createtable(L, ACT_CLI_OUT_NUMBER-1, 0); \
  /* get message constructor */ \
  lua_rawgetp(L, LUA_REGISTRYINDEX, cli); \
  lua_rawgeti(L, -1, NEW_ID); \
  /* make empty message */ \
  lua_call(L, 0, 1); \
  idl_lua_msg_t *msg = lua_touserdata(L, -1); \
  /* get and save message */ \
  rcl_ret_t ret = rcl_action_take_ ## Type(cli, msg->obj); \
  switch (ret) { \
    case RCL_RET_OK: break; \
    case RCL_RET_ACTION_CLIENT_TAKE_FAILED: \
      lua_pop(L, 2); \
      return 1; \
    default: \
      luaL_error(L, "failed to take " #Type); \
  } \
  lua_rawseti(L, -3, ACT_CLI_OUT_RESPONSE); \
  /* save callback function */ \
  lua_rawgeti(L, -1, CB_ID); \
  lua_rawseti(L, -3, ACT_CLI_OUT_CALLBACK); \
  lua_pop(L, 1); \
  return 1;

/**
 * Get feedback message.
 * Push to the stack {response, callback}.
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_action_client_take_feedback (lua_State* L)
{
  TAKE_MESSAGE(feedback, ACT_CLI_REG_NEW_FEEDBACK, ACT_CLI_REG_FEEDBACK_CB)
}

/**
 * Get status message.
 * Push to the stack {response, callback}.
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_action_client_take_status (lua_State* L)
{
  TAKE_MESSAGE(status, ACT_CLI_REG_NEW_STATUS, ACT_CLI_REG_STATUS_CB)
}

/**
 * Get number of enities to update wait set.
 *
 * Arguments:
 * - action client
 * 
 * Return:
 * - subcription number
 * - guard conditions number
 * - timers number
 * - clients number
 * - servers number
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_action_client_num_entities (lua_State* L)
{
  /* arg1 - action client */
  rcl_action_client_t* cli = luaL_checkudata(L, 1, MT_ACTION_CLIENT);

  size_t count[] = {0, 0, 0, 0, 0};
  rcl_ret_t ret = rcl_action_client_wait_set_get_num_entities(
    cli, count, count+1, count+2, count+3, count+4);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to get number of entities");
  }

  /* put to stack */
  for(size_t i = 0; i < 5; i++) {
    lua_pushinteger(L, count[i]);
  }

  return 5;
}

/**
 * Check if the action server is available.
 *
 * Arguments:
 * - action client
 *
 * Return:
 * - true when server is available
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_action_client_server_is_available (lua_State* L)
{
  /* arg1 - action client object */
  rcl_action_client_t* cli = lua_touserdata(L, 1);

  /* get node */
  lua_rawgetp(L, LUA_REGISTRYINDEX, cli);    // push table
  lua_rawgeti(L, -1, ACT_CLI_REG_NODE);      // push node
  rcl_node_t* node = lua_touserdata(L, -1);

  bool available = false;
  rcl_ret_t ret = rcl_action_server_is_available(node, cli, &available);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to check server");
  }

  lua_pushboolean(L, available);
  return 1;
}

/**
 * Add action client to wait set.
 *
 * Arguments:
 * - action client
 * - wait set object
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_action_client_add_wait_set (lua_State* L)
{
  /* arg1 - action client */
  rcl_action_client_t* cli = luaL_checkudata(L, 1, MT_ACTION_CLIENT);
  /* arg2 - WaitSet */
  rcl_wait_set_t* wait_set = luaL_checkudata(L, 2, MT_WAIT_SET);

  rcl_ret_t ret = rcl_action_wait_set_add_action_client(wait_set, cli, NULL, NULL);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to add action client to wait set");
  }

  return 0;
}

/**
 * Check if there are any ready entries.
 *
 * Arguments:
 * - action client
 * - wait set object
 *
 * Return:
 * - feedback message flag
 * - status message flag
 * - goal response flag
 * - cancel response flag
 * - result response flag
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_action_client_is_ready (lua_State* L)
{
  /* arg1 - action client */
  rcl_action_client_t* cli = luaL_checkudata(L, 1, MT_ACTION_CLIENT);
  /* arg2 - WaitSet */
  rcl_wait_set_t* wait_set = luaL_checkudata(L, 2, MT_WAIT_SET);

  bool status[] = {false, false, false, false, false};
  rcl_ret_t ret = rcl_action_client_wait_set_get_entities_ready(
    wait_set, cli, status, status+1, status+2, status+3, status+4);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to get ready action client entries");
  }

  /* put to stack */
  for(size_t i = 0; i < 5; i++) {
    lua_pushboolean(L, status[i]);
  }

  return 5;
}

/** List of the action client methods. */
static const struct luaL_Reg act_cli_methods[] = {
  {"__gc", rcl_lua_action_client_free},
  {"send_goal_request", rcl_lua_action_client_send_goal_request},
  {"take_goal_response", rcl_lua_action_client_get_goal_response},
  {"send_result_request", rcl_lua_action_client_send_result_request},
  {"take_result_response", rcl_lua_action_client_get_result_response},
  {"send_cancel_request", rcl_lua_action_client_send_cancel_request},
  {"take_cancel_response", rcl_lua_action_client_get_cancel_response},
  {"take_feedback", rcl_lua_action_client_take_feedback},
  {"take_status", rcl_lua_action_client_take_status},
  {"get_num_entities", rcl_lua_action_client_num_entities},
  {"is_action_server_available", rcl_lua_action_client_server_is_available},
  {"add_to_waitset", rcl_lua_action_client_add_wait_set},
  {"is_ready", rcl_lua_action_client_is_ready},
  {NULL, NULL}
};

/* Add action client to library. */
void rcl_lua_add_action_client (lua_State* L)
{
  /* constructor */
  lua_pushcfunction(L, rcl_lua_action_client_init);  // push function
  lua_setfield(L, -2, "new_action_client");          // pop, lib['new_action_client'] = function

  /* metamethods */
  rcl_lua_utils_add_mt(L, MT_ACTION_CLIENT, act_cli_methods);
}
