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
#include <rmw/types.h>
#include <rmw/qos_profiles.h>

#include <rosidl_luacommon/definition.h>

#include "rcllua/action_client.h"
#include "rcllua/node.h"
#include "rcllua/qos.h"
#include "rcllua/wait_set.h"
#include "rcllua/utils.h"

/** Indices of action client binding in register. */
enum ActCliReg {
  /** node reference */
  ACT_CLI_REG_NODE = 1,
  /** interface library */
  ACT_CLI_REG_INTERFACE,
  /** goal requests and callbacks */
  ACT_CLI_REG_GOAL_LIST,
  /** cancel requests and callbacks */
  ACT_CLI_REG_CANCEL_LIST,
  /** result requests and callbacks */
  ACT_CLI_REG_RESULT_LIST,
  /** feedback callbacks */
  ACT_CLI_REG_FEEDBACK_LIST,
  /** feedback constructor */
  ACT_CLI_REG_FEEDBACK_NEW,
  /** status interface */
  ACT_CLI_REG_STATUS_NEW,
 /** number of elements + 1 */
  ACT_CLI_REG_NUMBER
};

/** List of outputs for clients and subscriptions */
enum ActCliOut {
  /** response message */
  ACT_CLI_OUT_RESPONSE = 1,
  /** callback function */
  ACT_CLI_OUT_CALLBACK,
  /** sequence number */
  ACT_CLI_OUT_SEQUENCE,
  /** number of elements + 1 */
  ACT_CLI_OUT_NUMBER
};

/** Action client object metatable name. */
const char* MT_ACTION_CLIENT = "ROS2.ActionClient";

/**
 * Create action client object. Save bindings to register.
 *
 * Table: rclbind
 * Method: new_action_client
 *
 * Arguments:
 * - node object
 * - action type (table)
 * - action name
 * - QoS table (=nil)
 * -- goal_service_qos
 * -- result_service_qos
 * -- cancel_service_qos
 * -- feedback_topic_qos
 * -- status_topic_qos
 * - cancel interface
 * - state interface
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
    if (ROSIDL_LUA_PUSH_TYPESUPPORT(L, 2) == LUA_TLIGHTUSERDATA) {   // push pointer
      ts = lua_touserdata(L, -1);
    }
    lua_pop(L, 1);                       // pop pointer
  }
  if (NULL == ts) {
    luaL_argerror(L, 2, "expected action type");
  }

  /* arg3 - action service name */
  const char* srv_name = luaL_checkstring(L, 3);

  /* arg4 - QoS table */
  rcl_action_client_options_t action_client_ops = rcl_action_client_get_default_options();
  if (lua_istable(L, 4)) {
    if (lua_getfield(L, 4, "goal_service_qos") != LUA_TNIL) {
      action_client_ops.goal_service_qos = *((rmw_qos_profile_t*) luaL_checkudata(L, -1, MT_QOS));
    }
    lua_pop(L, 1);
    if (lua_getfield(L, 4, "result_service_qos") != LUA_TNIL) {
      action_client_ops.result_service_qos = *((rmw_qos_profile_t*) luaL_checkudata(L, -1, MT_QOS));
    }
    lua_pop(L, 1);
    if (lua_getfield(L, 4, "cancel_service_qos") != LUA_TNIL) {
      action_client_ops.cancel_service_qos = *((rmw_qos_profile_t*) luaL_checkudata(L, -1, MT_QOS));
    }
    lua_pop(L, 1);
    if (lua_getfield(L, 4, "feedback_topic_qos") != LUA_TNIL) {
      action_client_ops.feedback_topic_qos = *((rmw_qos_profile_t*) luaL_checkudata(L, -1, MT_QOS));
    }
    lua_pop(L, 1);
    if (lua_getfield(L, 4, "status_topic_qos") != LUA_TNIL) {
      action_client_ops.status_topic_qos = *((rmw_qos_profile_t*) luaL_checkudata(L, -1, MT_QOS));
    }
    lua_pop(L, 1);
  }

  /* arg5 - cancel interface */
  bool is_interface = false;
  if (lua_istable(L, 5)) {
    is_interface = (ROSIDL_LUA_PUSH_TYPESUPPORT(L, 5) == LUA_TLIGHTUSERDATA);  // push
    lua_pop(L, 1);             // pop
  }
  luaL_argcheck(L, is_interface, 5, "CancelGoal interface is expected");

  /* arg6 - state */
  is_interface = false;
  if (lua_istable(L, 6)) {
    is_interface = (ROSIDL_LUA_PUSH_TYPESUPPORT(L, 6) == LUA_TLIGHTUSERDATA);
    lua_pop(L, 1);
  }
  luaL_argcheck(L, is_interface, 6, "GoalStatus interface is expected");

  /* new action client */
  rcl_action_client_t* cli = lua_newuserdata(L, sizeof(rcl_action_client_t));  // push object
  *cli = rcl_action_get_zero_initialized_client();

  rcl_ret_t ret = rcl_action_client_init(cli, node, ts, srv_name, &action_client_ops);
  switch (ret) {
    case RCL_RET_OK: break;
    case RCL_RET_ACTION_NAME_INVALID:
      luaL_error(L, "Invalid action name: %s", srv_name); break;
    default:
      luaL_error(L, "Failed to create action client");
  }

  /* set metamethods */
  luaL_getmetatable(L, MT_ACTION_CLIENT);  // push metatable
  lua_setmetatable(L, -2);                 // pop metatable

  if (!lua_checkstack(L, 5)) {
    luaL_error(L, "not enough space for action client initialization");
  }

  /* save reference objects */
  lua_createtable(L, 0, ACT_CLI_REG_NUMBER-1);  // push table a

  lua_pushvalue(L, 1);                   // push node
  lua_rawseti(L, -2, ACT_CLI_REG_NODE);  // pop node

  /* copy interfaces */
  lua_newtable(L);                       // push interface table
  lua_pushnil(L);                        // push key
  while (lua_next(L, 2) != 0) {
    if (strcmp(lua_tostring(L, -2), "_type_support") == 0) {
      lua_pop(L, 1);
      continue;
    }
    lua_pushvalue(L, -2);                // push, duplicate key
    lua_rotate(L, -3, 1);                // key, key, value
    lua_rawset(L, -4);                   // pop key and value
  }
  lua_pushvalue(L, 5);                   // push CancelGoal interface
  lua_setfield(L, -2, "CancelGoal");     // pop, save interface
  lua_pushvalue(L, 6);                   // push GoalStatus
  lua_setfield(L, -2, "GoalStatus");     // pop, save interface
  lua_rawseti(L, -2, ACT_CLI_REG_INTERFACE);  // pop interface

  /* simplify feedback constructor call */
  lua_getfield(L, 2, "FeedbackMessage");  // push table
  ROSIDL_LUA_PUSH_CONSTRUCTOR(L, -1);            // push constructor
  lua_rawseti(L, -3, ACT_CLI_REG_FEEDBACK_NEW);  // pop constructor
  lua_pop(L, 1);                         // pop table

  /* simplify status constructor call */
  ROSIDL_LUA_PUSH_CONSTRUCTOR(L, 6);           // push constructor
  lua_rawseti(L, -2, ACT_CLI_REG_STATUS_NEW);  // pop constructor

  /* prepare tables for feedback */
  for (int i = ACT_CLI_REG_GOAL_LIST; i <= ACT_CLI_REG_FEEDBACK_LIST; i++) {
    lua_newtable(L);                     // push empty table
    lua_rawseti(L, -2, i);               // pop table
  }

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
    luaL_error(L, "failed to fini action client");
  }

  /* free dependencies */
  lua_pushnil(L);
  lua_rawsetp(L, LUA_REGISTRYINDEX, cli);

  return 0;
}

/**
 * Request template.
 * \param Type Service type.
 * \param CB_ID Index of callbacks in register.
 */
#define SEND_SERVICE_REQUEST(Type, CB_ID) \
  /* arg1 - action client */ \
  rcl_action_client_t* cli = luaL_checkudata(L, 1, MT_ACTION_CLIENT); \
  /* arg2 - request */ \
  idl_lua_msg_t* req = lua_touserdata(L, 2); \
  if (NULL == req) { luaL_argerror(L, 2, "request is expected"); } \
  /* arg3 - callback */ \
  luaL_argcheck(L, lua_isfunction(L, 3), 3, "callback is expected"); \
  /* send */ \
  int64_t seq_num = 0; \
  rcl_ret_t ret = rcl_action_send_ ## Type ## _request(cli, ROSIDL_LUA_GET_MSG(req), &seq_num); \
  if (RCL_RET_OK != ret) { \
    luaL_error(L, "failed to send " #Type " request"); \
  } \
  /* save callback */ \
  lua_rawgetp(L, LUA_REGISTRYINDEX, cli); \
  lua_rawgeti(L, -1, CB_ID); \
  lua_pushvalue(L, 3); \
  lua_rawseti(L, -2, seq_num); \
  /* return sequence number */ \
  lua_pushinteger(L, seq_num); \
  return 1;

/**
 * Send request to result service.
 *
 * Table: ActionClient
 * Method: send_result_request
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
  SEND_SERVICE_REQUEST(result, ACT_CLI_REG_RESULT_LIST)
}

/**
 * Send request to cancel service.
 *
 * Table: ActionClient
 * Method: send_cancel_request
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
  SEND_SERVICE_REQUEST(cancel, ACT_CLI_REG_CANCEL_LIST)
}

/**
 * Send request to goal service.
 *
 * Table: ActionClient
 * Method: send_goal_request
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
  SEND_SERVICE_REQUEST(goal, ACT_CLI_REG_GOAL_LIST)
}

/**
 * Get service response.
 * \param TBL_NAME String with service name.
 * \param CB_ID Index of table with service callbacks.
 */
#define TAKE_SERVICE_RESPONSE(Type, TBL_NAME, CB_ID) \
  /* arg1 - action client */ \
  rcl_action_client_t* cli = luaL_checkudata(L, 1, MT_ACTION_CLIENT); \
  /* prepare response message */ \
  lua_rawgetp(L, LUA_REGISTRYINDEX, cli); \
  lua_rawgeti(L, -1, ACT_CLI_REG_INTERFACE); \
  lua_getfield(L, -1, TBL_NAME); \
  lua_getfield(L, -1, "Response"); \
  ROSIDL_LUA_PUSH_CONSTRUCTOR(L, -1); \
  lua_rotate(L, -3, 1); lua_pop(L, 2); \
  lua_call(L, 0, 1); \
  idl_lua_msg_t* msg = lua_touserdata(L, -1); \
  /* get response */ \
  rmw_request_id_t header; \
  rcl_ret_t ret = rcl_action_take_ ## Type ## _response(cli, &header, ROSIDL_LUA_GET_MSG(msg)); \
  switch (ret) { \
    case RCL_RET_OK: break; \
    case RCL_RET_ACTION_CLIENT_TAKE_FAILED: \
    case RCL_RET_ACTION_SERVER_TAKE_FAILED: \
      lua_pushnil(L); \
      return 1; \
    default: \
      luaL_error(L, "failed to take " #Type); \
  } \
  /* check request id */ \
  lua_rawgeti(L, 2, CB_ID); \
  if (lua_rawgeti(L, -1, header.sequence_number) == LUA_TNIL) { \
    return 1; \
  } \
  /* save result into table */ \
  lua_createtable(L, ACT_CLI_OUT_NUMBER-1, 0); \
  lua_pushvalue(L, -2); \
  lua_rawseti(L, -2, ACT_CLI_OUT_CALLBACK); \
  lua_pushvalue(L, -4); \
  lua_rawseti(L, -2, ACT_CLI_OUT_RESPONSE); \
  lua_pushinteger(L, header.sequence_number); \
  lua_rawseti(L, -2, ACT_CLI_OUT_SEQUENCE); \
  /* remove request */ \
  lua_pushnil(L); \
  lua_rawseti(L, -4, header.sequence_number); \
  return 1;

/**
 * Get response from result service.
 *
 * Table: ActionClient
 * Method: take_result_response
 *
 * Arguments:
 * - action client
 *
 * Return:
 * - nil or table {response, feedback, sequence}
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_action_client_get_result_response (lua_State* L)
{
  TAKE_SERVICE_RESPONSE(result, "GetResult", ACT_CLI_REG_RESULT_LIST)
}

/**
 * Get response from cancel service.
 *
 * Table: ActionClient
 * Method: take_cancel_response
 *
 * Arguments:
 * - action client
 *
 * Return:
 * - nil or table {response, feedback, sequence}
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_action_client_get_cancel_response (lua_State* L)
{
  TAKE_SERVICE_RESPONSE(cancel, "CancelGoal", ACT_CLI_REG_CANCEL_LIST)
}

/**
 * Get response from goal service.
 *
 * Table: ActionClient
 * Method: take_goal_response
 *
 * Arguments:
 * - action client
 *
 * Return:
 * - nil or table {response, feedback, sequence}
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_action_client_get_goal_response (lua_State* L)
{
  TAKE_SERVICE_RESPONSE(goal, "SendGoal", ACT_CLI_REG_GOAL_LIST)
}

/**
 * Get feedback message. If callback is registered then
 * return {message, callback} else nil.
 *
 * Table: ActionClient
 * Method: take_feedback
 *
 * Arguments:
 * - action client
 *
 * Return:
 * - nil or {message, callback}
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_action_client_take_feedback (lua_State* L)
{
  /* arg1 - action client */
  rcl_action_client_t* cli = luaL_checkudata(L, 1, MT_ACTION_CLIENT);

  /* make empty message */
  lua_rawgetp(L, LUA_REGISTRYINDEX, cli);      // push table a
  lua_rawgeti(L, -1, ACT_CLI_REG_FEEDBACK_NEW);  // push constructor
  lua_call(L, 0, 1);                           // pop constructor, push message
  idl_lua_msg_t *msg = lua_touserdata(L, -1);

  /* get message */
  rcl_ret_t ret = rcl_action_take_feedback(cli, ROSIDL_LUA_GET_MSG(msg));
  switch (ret) {
    case RCL_RET_OK: break;
    case RCL_RET_ACTION_CLIENT_TAKE_FAILED:
      lua_pushnil(L);
      return 1;
    default:
      luaL_error(L, "failed to take feedbak");
  }

  /* get uuid */
  lua_getfield(L, -1, "goal_id");   // push userdata
  lua_getfield(L, -1, "uuid");      // push uuid
  lua_remove(L, -2);                // pop goal_id
  rcl_lua_utils_push_uuid_str(L, -1);  // push uuid str
  lua_rawgeti(L, -4, ACT_CLI_REG_FEEDBACK_LIST);  // push table b
  lua_replace(L, -3);               // pop, replace userdata
  if (lua_rawget(L, -2) == LUA_TNIL) {    // pop uuid, push callback or nil
    return 1;  // callback not found
  }

  /* prepare result */
  lua_createtable(L, ACT_CLI_OUT_NUMBER-1, 0);  // push table c
  lua_pushvalue(L, -2);                  // push feedback
  lua_rawseti(L, -2, ACT_CLI_OUT_CALLBACK);  // pop feedback
  lua_pushvalue(L, -4);                  // push message
  lua_rawseti(L, -2, ACT_CLI_OUT_RESPONSE);  // pop message

  return 1;
}

/**
 * Get status message.
 *
 * Table: ActionClient
 * Method: take_status
 *
 * Arguments:
 * - action client
 *
 * Return:
 * - status message
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_action_client_take_status (lua_State* L)
{
  /* arg1 - action client */
  rcl_action_client_t* cli = luaL_checkudata(L, 1, MT_ACTION_CLIENT);

  /* make empty message */
  lua_rawgetp(L, LUA_REGISTRYINDEX, cli);      // push table a
  lua_rawgeti(L, -1, ACT_CLI_REG_STATUS_NEW);  // push constructor
  lua_call(L, 0, 1);                           // pop constructor, push message
  idl_lua_msg_t *msg = lua_touserdata(L, -1);

  /* get message */
  rcl_ret_t ret = rcl_action_take_status(cli, ROSIDL_LUA_GET_MSG(msg));
  switch (ret) {
    case RCL_RET_OK: break;
    case RCL_RET_ACTION_CLIENT_TAKE_FAILED:
      lua_pushnil(L);
      return 1;
    default:
      luaL_error(L, "failed to take status");
  }

  return 1;
}

/**
 * Get number of enities to update wait set.
 *
 * Table: ActionClient
 * Method: get_num_entities
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
  rcl_action_client_t* cli = lua_touserdata(L, 1);

  size_t count[5] = {0, 0, 0, 0, 0};
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
 * Table: ActionClient
 * Method: is_action_server_available
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
  rcl_action_client_t* cli = luaL_checkudata(L, 1, MT_ACTION_CLIENT);

  /* get node */
  lua_rawgetp(L, LUA_REGISTRYINDEX, cli);    // push table
  lua_rawgeti(L, -1, ACT_CLI_REG_NODE);      // push node
  rcl_node_t* node = lua_touserdata(L, -1);

  bool available = false;
  rcl_ret_t ret = rcl_action_server_is_available(node, cli, &available);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to check action server");
  }

  lua_pushboolean(L, available);
  return 1;
}

/**
 * Add action client to wait set.
 *
 * Table: ActionClient
 * Method: add_to_waitset
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
 * Check ready entries.
 *
 * Table: ActionClient
 * Method: is_ready
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

  bool status[5] = {false, false, false, false, false};
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

/**
 * Get message constructor for the specific action structure.
 *
 * Table: ActionClient
 * Method: get_interface
 *
 * Arguments:
 * - action client
 * - interface name
 *
 * Return:
 * - found interface table or nil
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_action_client_get_interface (lua_State* L)
{
  /* arg1 - action client */
  rcl_action_client_t* cli = luaL_checkudata(L, 1, MT_ACTION_CLIENT);
  /* arg2 - interface name */
  const char* type = luaL_checkstring(L, 2);

  lua_rawgetp(L, LUA_REGISTRYINDEX, cli);
  lua_rawgeti(L, -1, ACT_CLI_REG_INTERFACE);

  lua_getfield(L, -1, type);
  return 1;
}

/**
 * Set function to call for action server feedback.
 *
 * Table: ActionClient
 * Method: set_feedback_method
 *
 * Arguments:
 * - action client
 * - UUID object (table or message)
 * - function
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_action_client_set_feedback (lua_State* L)
{
  /* arg1 - action client */
  rcl_action_client_t* cli = luaL_checkudata(L, 1, MT_ACTION_CLIENT);
  /* arg2 - uuid */
  luaL_argcheck(L, lua_istable(L, 2) || lua_isuserdata(L, 2), 2, "expected message field or table");
  /* arg3 - feedback callback */
  luaL_argcheck(L, lua_isfunction(L, 3), 3, "callback is expected");

  /* save callback */
  lua_rawgetp(L, LUA_REGISTRYINDEX, cli);
  lua_rawgeti(L, -1, ACT_CLI_REG_FEEDBACK_LIST);
  rcl_lua_utils_push_uuid_str(L, 2);
  lua_pushvalue(L, 3);
  lua_rawset(L, -3);

  return 0;
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
  {"get_interface", rcl_lua_action_client_get_interface},
  {"set_feedback_method", rcl_lua_action_client_set_feedback},
  {NULL, NULL}
};

/* Add action client to library. */
void rcl_lua_add_action_client_methods (lua_State* L)
{
  /* constructor */
  lua_pushcfunction(L, rcl_lua_action_client_init);  // push function
  lua_setfield(L, -2, "new_action_client");          // pop, lib['new_action_client'] = function

  /* metamethods */
  rcl_lua_utils_add_mt(L, MT_ACTION_CLIENT, act_cli_methods);
}
