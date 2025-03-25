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

#include <rcl_action/action_server.h>
#include <rcl_action/wait.h>
#include <rcl_action/types.h>
#include <rcl/error_handling.h>

#include <rosidl_luacommon/definition.h>
#include <rosidl_luacommon/utility.h>

#include "rcllua/action_server.h"
#include "rcllua/node.h"
#include "rcllua/clock.h"
#include "rcllua/wait_set.h"
#include "rcllua/qos.h"
#include "rcllua/utils.h"


/** Indices of action server binding in register. */
enum ActSrvReg
{
  /** node reference */
  ACT_SRV_REG_NODE = 1,
  /** interface library */
  ACT_SRV_REG_INTERFACE,
  /** main process */
  ACT_SRV_REG_EXEC_CB,
  /** check goal acceptance criteria fn(request) -> bool */
  ACT_SRV_REG_GOAL_CB,
  /** prerequisit function that calls main process */
  ACT_SRV_REG_HANDLE_CB,
  /** check cancel acceptance criteria fn(request) -> bool */
  ACT_SRV_REG_CANCEL_CB,
  /** feedback metatable name */
  ACT_SRV_REG_FEEDBACK_MT,
  /** number of elements + 1 */
  ACT_SRV_REG_NUMBER
};

/** List of output elements. */
enum ActSrvOut
{
  /** request message */
  ACT_SRV_OUT_REQUEST = 1,
  /** callback funciton */
  ACT_SRV_OUT_CALLBACK,
  /** header */
  ACT_SRV_OUT_HEADER,
  /** number of elements + 1 */
  ACT_SRV_OUT_NUMBER
};

/** Action server object metatable name. */
const char * MT_ACTION_SERVER = "ROS2.ActionServer";
/** Action goal handle object metatable name. */
const char * MT_ACTION_GOAL_HANDLE = "ROS2.ActionGoalHandle";

/**
 * Default acceptance criteria.
 *
 * Return:
 * - always true
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_action_server_push_true(lua_State * L)
{
  lua_pushboolean(L, true);
  return 1;
}

/**
 * Create action server object. Save bindings to register.
 *
 * Table: rclbind
 * Method: new_action_server
 *
 * Arguments:
 * - node object
 * - clock object
 * - action type (table)
 * - action name
 * - parameter table (=nil)
 * -- goal_service_qos
 * -- result_service_qos
 * -- cancel_service_qos
 * -- feedback_topic_qos
 * -- status_topic_qos
 * -- result_timeout (=900)
 * -- goal_callback (=fn()->true)
 * -- handle_accepted_callback
 * -- cancel_callback (=fn()->true)
 * - process function
 * - cancel interface
 *
 * Return:
 * - action server object
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_action_server_init(lua_State * L)
{
  /* arg1 - node */
  rcl_node_t * node = luaL_checkudata(L, 1, MT_NODE);
  /* arg2 - clock */
  rcl_clock_t * clock = luaL_checkudata(L, 2, MT_CLOCK);

  /* arg3 - action type */
  rosidl_action_type_support_t * ts = NULL;
  /* check table */
  if (lua_istable(L, 3)) {
    if (ROSIDL_LUA_PUSH_TYPESUPPORT(L, 3) == LUA_TLIGHTUSERDATA) {  // push pointer
      ts = lua_touserdata(L, -1);
    }
    lua_pop(L, 1);                       // pop pointer
  }
  if (NULL == ts) {
    luaL_argerror(L, 3, "expected action type");
  }

  /* arg4 - action service name */
  const char * srv_name = luaL_checkstring(L, 4);

  /* arg5 - parameters table */
  rcl_action_server_options_t action_server_ops = rcl_action_server_get_default_options();
  if (lua_istable(L, 5)) {
    /* QoS */
    if (lua_getfield(L, 5, "goal_service_qos") != LUA_TNIL) {
      action_server_ops.goal_service_qos = *((rmw_qos_profile_t *) luaL_checkudata(L, -1, MT_QOS));
    }
    lua_pop(L, 1);
    if (lua_getfield(L, 5, "result_service_qos") != LUA_TNIL) {
      action_server_ops.result_service_qos = *((rmw_qos_profile_t *) luaL_checkudata(L, -1,
        MT_QOS));
    }
    lua_pop(L, 1);
    if (lua_getfield(L, 5, "cancel_service_qos") != LUA_TNIL) {
      action_server_ops.cancel_service_qos = *((rmw_qos_profile_t *) luaL_checkudata(L, -1,
        MT_QOS));
    }
    lua_pop(L, 1);
    if (lua_getfield(L, 5, "feedback_topic_qos") != LUA_TNIL) {
      action_server_ops.feedback_topic_qos = *((rmw_qos_profile_t *) luaL_checkudata(L, -1,
        MT_QOS));
    }
    lua_pop(L, 1);
    if (lua_getfield(L, 5, "status_topic_qos") != LUA_TNIL) {
      action_server_ops.status_topic_qos = *((rmw_qos_profile_t *) luaL_checkudata(L, -1, MT_QOS));
    }
    lua_pop(L, 1);
    /* timeout */
    if (lua_getfield(L, 5, "result_timeout") != LUA_TNIL) {
      action_server_ops.result_timeout.nanoseconds =
        (rcl_duration_value_t) RCL_S_TO_NS(luaL_checknumber(L, -1));
    } else {
      action_server_ops.result_timeout.nanoseconds = (rcl_duration_value_t) 900;  // seconds
    }
    lua_pop(L, 1);
  }

  /* arg6 - execute callback */
  luaL_argcheck(L, lua_isfunction(L, 6), 6, "function is expected");

  /* arg7 - cancel interface */
  bool is_interface = false;
  if (lua_istable(L, 7)) {
    is_interface = (ROSIDL_LUA_PUSH_TYPESUPPORT(L, 7) == LUA_TLIGHTUSERDATA);  // push
    lua_pop(L, 1);            // pop
  }
  luaL_argcheck(L, is_interface, 7, "CancelGoal interface is expected");

  /* new action server */
  rcl_action_server_t *srv = lua_newuserdata(L, sizeof(rcl_action_server_t));  // push object
  *srv = rcl_action_get_zero_initialized_server();

  rcl_ret_t ret = rcl_action_server_init(srv, node, clock, ts, srv_name, &action_server_ops);
  switch (ret) {
    case RCL_RET_OK: break;
    case RCL_RET_ACTION_NAME_INVALID:
      luaL_error(L, "failed topic name %s", srv_name); break;
    default:
      luaL_error(L, "failed to create action server");
  }

  /* set metamethods */
  luaL_getmetatable(L, MT_ACTION_SERVER);  // push metatable
  lua_setmetatable(L, -2);                 // pop metatable

  if (!lua_checkstack(L, 5)) {
    luaL_error(L, "not enough space for action client initialization");
  }

  /* save reference objects */
  lua_createtable(L, 0, ACT_SRV_REG_NUMBER - 1);  // push table a

  lua_pushvalue(L, 1);                   // push node
  lua_rawseti(L, -2, ACT_SRV_REG_NODE);  // pop node

  /* acceptance criteria */
  if (lua_istable(L, 5)) {
    if (lua_getfield(L, 5, "goal_callback") == LUA_TNIL) {    // push
      lua_pop(L, 1);                                          // pop nil
      lua_pushcfunction(L, rcl_lua_action_server_push_true);  // push
    }
    lua_rawseti(L, -2, ACT_SRV_REG_GOAL_CB);    // pop function
    if (lua_getfield(L, 5, "handle_accepted_callback") != LUA_TNIL) {  // push
      lua_rawseti(L, -2, ACT_SRV_REG_HANDLE_CB);  // pop
    } else {
      lua_pop(L, 1);                              // pop
    }
    if (lua_getfield(L, 5, "cancel_callback") == LUA_TNIL) {  // push
      lua_pop(L, 1);                                          // pop nil
      lua_pushcfunction(L, rcl_lua_action_server_push_true);  // push
    }
    lua_rawseti(L, -2, ACT_SRV_REG_CANCEL_CB);  // pop function
  }

  /* copy interfaces */
  lua_newtable(L);                       // push interface table
  lua_pushnil(L);                        // push key
  while (lua_next(L, 3) != 0) {
    if (strcmp(lua_tostring(L, -2), "_type_support") == 0) {
      lua_pop(L, 1);
      continue;
    }
    lua_pushvalue(L, -2);                // push, duplicate key
    lua_rotate(L, -3, 1);                // key, key, value
    lua_rawset(L, -4);                   // pop key and value
  }
  lua_pushvalue(L, 7);                   // push CancelGoal interface
  lua_setfield(L, -2, "CancelGoal");     // pop CancelGoal interface
  lua_rawseti(L, -2, ACT_SRV_REG_INTERFACE);  // pop interface

  lua_pushvalue(L, 6);                      // push process function
  lua_rawseti(L, -2, ACT_SRV_REG_EXEC_CB);  // pop process function

  lua_getfield(L, 3, "FeedbackMessage");  // push message interface
  ROSIDL_LUA_PUSH_MT(L, -1);              // push push metatable name
  lua_rawseti(L, -3, ACT_SRV_REG_FEEDBACK_MT);  // pop metatable
  lua_pop(L, 1);                          // pop message interface

  lua_rawsetp(L, LUA_REGISTRYINDEX, srv);  // pop table, save to registry

  return 1;
}

/**
 * Action server destructor.
 *
 * Arguments:
 * - action server object
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_action_server_free(lua_State * L)
{
  /* arg1 - action server */
  rcl_action_server_t * srv = lua_touserdata(L, 1);

  /* get node */
  lua_rawgetp(L, LUA_REGISTRYINDEX, srv);  // push table
  lua_rawgeti(L, -1, ACT_SRV_REG_NODE);        // push node
  rcl_node_t * node = lua_touserdata(L, -1);

  /* finalize */
  rcl_ret_t ret = rcl_action_server_fini(srv, node);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to fini server");
  }

  /* free dependencies */
  lua_pushnil(L);
  lua_rawsetp(L, LUA_REGISTRYINDEX, srv);

  return 0;
}

/**
 * Get message constructor for the specific action structure.
 *
 * Table: ActionServer
 * Method: get_interface
 *
 * Arguments:
 * - action server
 * - interface name
 *
 * Return:
 * - found interface table or nil
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_action_server_get_interface(lua_State * L)
{
  /* arg1 - action client */
  rcl_action_server_t * srv = luaL_checkudata(L, 1, MT_ACTION_SERVER);
  /* arg2 - interface name */
  const char * type = luaL_checkstring(L, 2);

  lua_rawgetp(L, LUA_REGISTRYINDEX, srv);
  lua_rawgeti(L, -1, ACT_SRV_REG_INTERFACE);

  lua_getfield(L, -1, type);
  return 1;
}

/**
 * Template for taking client request.
 * \param Type Service type.
 * \param Interface table name.
 * \param CB_REQ_ID Index of callback in register.
 */
#define TAKE_SERVICE_REQUEST(Type, TBL_NAME, CB_REQ_ID) \
  /* arg1 - action server */ \
  rcl_action_server_t * srv = luaL_checkudata(L, 1, MT_ACTION_SERVER); \
  lua_createtable(L, ACT_SRV_OUT_NUMBER - 1, 0); \
  /* prepare request message */ \
  lua_rawgetp(L, LUA_REGISTRYINDEX, srv); \
  lua_rawgeti(L, -1, ACT_SRV_REG_INTERFACE); \
  lua_getfield(L, -1, TBL_NAME); \
  lua_getfield(L, -1, "Request"); \
  ROSIDL_LUA_PUSH_CONSTRUCTOR(L, -1); \
  lua_rotate(L, -4, 1); lua_pop(L, 3); \
  lua_call(L, 0, 1); \
  idl_lua_msg_t *msg = lua_touserdata(L, -1); \
  /* get request */ \
  rmw_request_id_t header; \
  rcl_ret_t ret = rcl_action_take_ ## Type ## _request(srv, &header, ROSIDL_LUA_GET_MSG(msg)); \
  switch (ret) { \
    case RCL_RET_OK: break; \
    case RCL_RET_ACTION_CLIENT_TAKE_FAILED: \
    case RCL_RET_ACTION_SERVER_TAKE_FAILED: \
      lua_pushnil(L); \
      return 1; \
    default: \
      luaL_error(L, "failed to take " #Type); \
  } \
  lua_rawseti(L, -3, ACT_SRV_OUT_REQUEST); \
  /* save header */ \
  rmw_request_id_t * info = lua_newuserdata(L, sizeof(rmw_request_id_t)); \
  *info = header; \
  lua_rawseti(L, -3, ACT_SRV_OUT_HEADER); \
  /* save callback function */ \
  lua_rawgeti(L, -1, CB_REQ_ID); \
  lua_rawseti(L, -3, ACT_SRV_OUT_CALLBACK); \
  lua_pop(L, 1); \
  return 1;

/**
 * Take goal request.
 *
 * Table: ActionServer
 * Method: take_goal_request
 *
 * Arguments:
 * - action server object
 *
 * Return:
 * - table {request, callback, header}
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_action_server_goal_request(lua_State * L)
{
  TAKE_SERVICE_REQUEST(goal, "SendGoal", ACT_SRV_REG_GOAL_CB)
}

/**
 * Take result request.
 *
 * Table: ActionServer
 * Method: take_result_request
 *
 * Arguments:
 * - action server object
 *
 * Return:
 * - table {request, callback, header}
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_action_server_result_request(lua_State * L)
{
  TAKE_SERVICE_REQUEST(result, "GetResult", -1)
}

/**
 * Take cancel request.
 *
 * Table: ActionServer
 * Method: take_cancel_request
 *
 * Arguments:
 * - action server object
 *
 * Return:
 * - table {request, callback, header}
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_action_server_cancel_request(lua_State * L)
{
  TAKE_SERVICE_REQUEST(cancel, "CancelGoal", ACT_SRV_REG_CANCEL_CB)
}

/**
 * Template for sending response.
 * \param Type Service type.
 */
#define SEND_SERVICE_RESPONSE(Type) \
  /* arg1 - action server */ \
  rcl_action_server_t * srv = luaL_checkudata(L, 1, MT_ACTION_SERVER); \
  /* arg2 - response */ \
  idl_lua_msg_t * resp = lua_touserdata(L, 2); \
  if (NULL == resp) {luaL_argerror(L, 2, "response is expected");} \
  /* arg3 - header */ \
  rmw_request_id_t * header = lua_touserdata(L, 3); \
  if (NULL == header) {luaL_argerror(L, 3, "header is expected");} \
  /* send response */ \
  rcl_ret_t ret = rcl_action_send_ ## Type ## _response(srv, header, ROSIDL_LUA_GET_MSG(resp)); \
  switch (ret) { \
    case RCL_RET_OK: break; \
    case RCL_RET_TIMEOUT: \
      lua_pushboolean(L, false); \
      lua_pushfstring(L, "failed to send response (timeout): %s", rcl_get_error_string().str); \
      rcl_reset_error(); \
      return 2; \
    default: \
      luaL_error(L, "failed to send " #Type " response"); \
  } \
  lua_pushboolean(L, true); \
  return 1; \

/**
 * Send goal response.
 *
 * Table: ActionServer
 * Method: send_goal_response
 *
 * Arguments:
 * - action server object
 * - response message
 * - header object
 *
 * Return:
 * - true in case of success
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_action_server_goal_response(lua_State * L)
{
  SEND_SERVICE_RESPONSE(goal)
}

/**
 * Send result response.
 *
 * Table: ActionServer
 * Method: send_result_response
 *
 * Arguments:
 * - action server object
 * - response message
 * - header object
 *
 * Return:
 * - true in case of success
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_action_server_result_response(lua_State * L)
{
  SEND_SERVICE_RESPONSE(result)
}

/**
 * Send cancel response.
 *
 * Table: ActionServer
 * Method: send_cancel_response
 *
 * Arguments:
 * - action server object
 * - response message
 * - header object
 *
 * Return:
 * - true in case of success
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_action_server_cancel_response(lua_State * L)
{
  SEND_SERVICE_RESPONSE(cancel)
}

/**
 * Get registered action process.
 *
 * Table: ActionServer
 * Method: get_executable
 *
 * Arguments:
 * - action server object
 *
 * Return:
 * - function
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_action_server_get_exec(lua_State * L)
{
  /* arg1 - action server */
  rcl_action_server_t * srv = luaL_checkudata(L, 1, MT_ACTION_SERVER);

  lua_rawgetp(L, LUA_REGISTRYINDEX, srv);  // push table

  lua_rawgeti(L, -1, ACT_SRV_REG_EXEC_CB);  // push method for execution
  return 1;
}

/**
 * Get method that may do additional configuration and run the main process.
 * The method takes goal handle and run execution.
 *
 * Table: ActionServer
 * Method: get_handle_preprocessing
 *
 * Arguments:
 * - action server object
 *
 * Return:
 * - function
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_action_server_get_handle_check(lua_State * L)
{
  /* arg1 - action server */
  rcl_action_server_t * srv = luaL_checkudata(L, 1, MT_ACTION_SERVER);

  lua_rawgetp(L, LUA_REGISTRYINDEX, srv);     // push table
  lua_rawgeti(L, -1, ACT_SRV_REG_HANDLE_CB);  // push method for goal handle run

  return 1;
}

/**
 * Send feedback message.
 *
 * Table: ActionServer
 * Method: publish_feedback
 *
 * Arguments:
 * - action server
 * - feedback message
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_action_server_publish_feedback(lua_State * L)
{
  /* arg1 - action server */
  rcl_action_server_t * srv = luaL_checkudata(L, 1, MT_ACTION_SERVER);

  /* arg2 - message object */
  lua_rawgetp(L, LUA_REGISTRYINDEX, srv);  // push table
  lua_rawgeti(L, -1, ACT_SRV_REG_FEEDBACK_MT);  // push metatable name
  const char * mt = lua_tostring(L, -1);
  idl_lua_msg_t *msg = luaL_checkudata(L, 2, mt);

  /* send */
  rcl_ret_t ret = rcl_action_publish_feedback(srv, ROSIDL_LUA_GET_MSG(msg));
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to publish feedback");
  }

  return 0;
}

/**
 * Send action server status.
 *
 * Table: ActionServer
 * Method: publish_status
 *
 * Arguments:
 * - action server
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_action_server_publish_status(lua_State * L)
{
  /* arg1 - action server */
  rcl_action_server_t * srv = luaL_checkudata(L, 1, MT_ACTION_SERVER);

  rcl_action_goal_status_array_t status_message =
    rcl_action_get_zero_initialized_goal_status_array();
  rcl_ret_t ret = rcl_action_get_goal_status_array(srv, &status_message);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to get goal status array");
  }

  ret = rcl_action_publish_status(srv, &status_message);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to publish goal status array");
  }

  ret = rcl_action_goal_status_array_fini(&status_message);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to finalize goal status array");
  }

  return 0;
}

/**
 * Notify server about finished task.
 *
 * Table: ActionServer
 * Method: notify_goal_done
 *
 * Arguments:
 * - action server
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_action_server_notify_goal_done(lua_State * L)
{
  /* arg1 - action server */
  rcl_action_server_t * srv = luaL_checkudata(L, 1, MT_ACTION_SERVER);

  rcl_ret_t ret = rcl_action_notify_goal_done(srv);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to notify action server of goal done");
  }

  return 0;
}

/**
 * Get number of entries to add to wait set.
 *
 * Table: ActionServer
 * Method: get_num_entities
 *
 * Arguments:
 * - actoin server
 *
 * Return:
 * - subscription number
 * - guard number
 * - timer number
 * - client number
 * - service number
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_action_server_num_entities(lua_State * L)
{
  /* arg1 - action server */
  rcl_action_server_t * srv = lua_touserdata(L, 1);

  /* subscriptions, guards, timers, clients, services */
  size_t count[5] = {0, 0, 0, 0, 0};
  rcl_ret_t ret = rcl_action_server_wait_set_get_num_entities(
    srv, count, count + 1, count + 2, count + 3, count + 4);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to get number of entities");
  }

  for(size_t i = 0; i < 5; i++) {
    lua_pushinteger(L, count[i]);
  }
  return 5;
}

/**
 * Check ready entries.
 *
 * Table: ActionServer
 * Method: is_ready
 *
 * Arguments:
 * - action server
 * - wait set object
 *
 * Return:
 * - goal request flag
 * - cancel request flag
 * - result request flag
 * - goal expired flag
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_action_server_is_ready(lua_State * L)
{
  /* arg1 - action server */
  rcl_action_server_t * srv = luaL_checkudata(L, 1, MT_ACTION_SERVER);
  /* arg2 - WaitSet */
  rcl_wait_set_t * wait_set = luaL_checkudata(L, 2, MT_WAIT_SET);

  /* goal_req, cancel_req, result_req, goal_expired */
  bool status[4] = {false, false, false, false};
  rcl_ret_t ret = rcl_action_server_wait_set_get_entities_ready(
    wait_set, srv, status, status + 1, status + 2, status + 3);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to get ready action server entries");
  }

  for(size_t i = 0; i < 4; i++) {
    lua_pushboolean(L, status[i]);
  }
  return 4;
}

/**
 * Add action server to wait set.
 *
 * Table: ActionServer
 * Method: add_to_waitset
 *
 * Arguments:
 * - action server
 * - wait set object
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_action_server_add_waitset(lua_State * L)
{
  /* arg1 - action server */
  rcl_action_server_t * srv = luaL_checkudata(L, 1, MT_ACTION_SERVER);
  /* arg2 - WaitSet */
  rcl_wait_set_t * wait_set = luaL_checkudata(L, 2, MT_WAIT_SET);

  rcl_ret_t ret = rcl_action_wait_set_add_action_server(wait_set, srv, NULL);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to add action server to wait set");
  }

  return 0;
}

/**
 * Process cancel request, make response.
 *
 * Table: ActionServer
 * Method: process_cancel_request
 *
 * Arguments:
 * - action server
 * - cancel request object
 *
 * Return:
 * - response message
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_action_server_proc_cancel_request(lua_State * L)
{
  /* arg1 - action server */
  rcl_action_server_t * srv = luaL_checkudata(L, 1, MT_ACTION_SERVER);
  /* arg2 - cancel request */
  idl_lua_msg_t * req = lua_touserdata(L, 1);
  luaL_argcheck(L, NULL != req, 2, "CancelGoal request is expected");

  /* produce response */
  rcl_action_cancel_response_t rcl_resp = rcl_action_get_zero_initialized_cancel_response();

  rcl_ret_t ret = rcl_action_process_cancel_request(srv, ROSIDL_LUA_GET_MSG(req), &rcl_resp);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to process cancel request");
  }

  /* fill message */
  lua_rawgetp(L, LUA_REGISTRYINDEX, srv);
  lua_rawgeti(L, -1, ACT_SRV_REG_INTERFACE);
  lua_getfield(L, -1, "CancelGoal");
  lua_getfield(L, -1, "Response");
  rosidl_luacommon_struct_to_msg(L, -1, &rcl_resp.msg);  // push message

  /* free */
  ret = rcl_action_cancel_response_fini(&rcl_resp);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to finalize cancel response");
  }

  return 1;
}

/**
 * Get list of expire goals.
 *
 * Table: ActionServer
 * Method: expired_goals
 *
 * Arguments:
 * - action server
 * - total number of goals
 *
 * Return:
 * - table of goal ID's (as string)
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_action_server_expire_goals(lua_State * L)
{
  /* arg1 - action server */
  rcl_action_server_t * srv = luaL_checkudata(L, 1, MT_ACTION_SERVER);
  /* arg2 - max number */
  int max_goals = luaL_checkinteger(L, 2);
  luaL_argcheck(L, max_goals > 0, 2, "expected positive number");

  rcl_action_goal_info_t * expired_goals =
    lua_newuserdata(L, max_goals * sizeof(rcl_action_goal_info_t));
  size_t num_expired = 0;
  rcl_ret_t ret = rcl_action_expire_goals(srv, expired_goals, max_goals, &num_expired);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to expire goals");
  }

  /* UUID as strings */
  lua_createtable(L, num_expired, 0);    // push table
  for (size_t i = 0; i < num_expired; i++) {
    const char *str = (char *) expired_goals[i].goal_id.uuid;
    lua_pushlstring(L, str, 16);         // push string
    lua_rawseti(L, -2, i + 1);             // pop string
  }

  return 1;
}

/**
 * Create action goal handle object.
 *
 * Table: rclbind
 * Method: new_action_goal_handle
 *
 * Arguments:
 * - action server
 * - GoalInfo message
 *
 * Return:
 * - goal handle object
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_action_goal_handle_init(lua_State * L)
{
  /* arg1 - action server */
  rcl_action_server_t * srv = luaL_checkudata(L, 1, MT_ACTION_SERVER);
  /* arg2 - GoalInfo message */
  luaL_argcheck(L, lua_isuserdata(L, 2), 2, "expected GoalInfo message");
  idl_lua_msg_t * msg = lua_touserdata(L, 2);

  /* new goal */
  rcl_action_goal_info_t * goal_info_ptr = (rcl_action_goal_info_t *) ROSIDL_LUA_GET_MSG(msg);
  rcl_action_goal_handle_t * rcl_handle = rcl_action_accept_new_goal(srv, goal_info_ptr);
  if (!goal_info_ptr) {
    luaL_error(L, "failed to accept new goal");
  }

  /* make object */
  rcl_action_goal_handle_t * action_goal_handle =
    lua_newuserdata(L, sizeof(rcl_action_goal_handle_t));  // push object
  *action_goal_handle = *rcl_handle;

  /* set metatable */
  luaL_getmetatable(L, MT_ACTION_GOAL_HANDLE);  // push
  lua_setmetatable(L, -2);                      // pop

  return 1;
}

/**
 * Goal handle destructor.
 *
 * Arguments:
 * - goal handle
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_action_goal_handle_free(lua_State * L)
{
  /* arg1 - action goal handle */
  rcl_action_goal_handle_t * handle = lua_touserdata(L, 1);

  rcl_ret_t ret = rcl_action_goal_handle_fini(handle);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to fini action goal handle");
  }

  return 0;
}

/**
 * Get goal status.
 *
 * Table: ActionGoalHandle
 * Method: get_status
 *
 * Arguments:
 * - goal handle
 *
 * Return:
 * - goal status (integer)
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_action_goal_handle_get_status(lua_State * L)
{
  /* arg1 - action goal handle */
  rcl_action_goal_handle_t * handle = luaL_checkudata(L, 1, MT_ACTION_GOAL_HANDLE);

  rcl_action_goal_state_t status;
  rcl_ret_t ret = rcl_action_goal_handle_get_status(handle, &status);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to get goal status");
  }

  lua_pushinteger(L, status);
  return 1;
}

/**
 * Set new goal status.
 *
 * Table: ActionGoalHandle
 * Method: update_goal_state
 *
 * Arguments:
 * - goal handle
 * - event (integer)
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_action_goal_handle_set_status(lua_State * L)
{
  /* arg1 - action goal handle */
  rcl_action_goal_handle_t * handle = luaL_checkudata(L, 1, MT_ACTION_GOAL_HANDLE);
  /* arg2 - event */
  lua_Integer ev = luaL_checkinteger(L, 2);

  rcl_action_goal_event_t event = ev;
  rcl_ret_t ret = rcl_action_update_goal_state(handle, event);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to update goal status");
  }

  return 0;
}

/**
 * Check if the goal is active.
 *
 * Table: ActionGoalHandle
 * Method: is_active
 *
 * Arguments:
 * - goal handle
 *
 * Return:
 * - true if the goal is active
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_action_goal_handle_is_active(lua_State * L)
{
  /* arg1 - action goal handle */
  rcl_action_goal_handle_t * handle = luaL_checkudata(L, 1, MT_ACTION_GOAL_HANDLE);

  lua_pushboolean(L, rcl_action_goal_handle_is_active(handle));
  return 1;
}

/** List of action server methods. */
static const struct luaL_Reg act_srv_methods[] = {
  {"__gc", rcl_lua_action_server_free},
  {"take_goal_request", rcl_lua_action_server_goal_request},
  {"send_goal_response", rcl_lua_action_server_goal_response},
  {"take_result_request", rcl_lua_action_server_result_request},
  {"send_result_response", rcl_lua_action_server_result_response},
  {"take_cancel_request", rcl_lua_action_server_cancel_request},
  {"send_cancel_response", rcl_lua_action_server_cancel_response},
  {"publish_feedback", rcl_lua_action_server_publish_feedback},
  {"publish_status", rcl_lua_action_server_publish_status},
  {"notify_goal_done", rcl_lua_action_server_notify_goal_done},
  {"get_num_entities", rcl_lua_action_server_num_entities},
  {"is_ready", rcl_lua_action_server_is_ready},
  {"add_to_waitset", rcl_lua_action_server_add_waitset},
  {"get_interface", rcl_lua_action_server_get_interface},
  {"get_executable", rcl_lua_action_server_get_exec},
  {"get_handle_preprocessing", rcl_lua_action_server_get_handle_check},
  {"expire_goals", rcl_lua_action_server_expire_goals},
  {"process_cancel_request", rcl_lua_action_server_proc_cancel_request},
  {NULL, NULL}
};

/** List of action goal handle methods. */
static const struct luaL_Reg act_srv_handle_methods[] = {
  {"__gc", rcl_lua_action_goal_handle_free},
  {"get_status", rcl_lua_action_goal_handle_get_status},
  {"update_goal_state", rcl_lua_action_goal_handle_set_status},
  {"is_active", rcl_lua_action_goal_handle_is_active},
  {NULL, NULL}
};

/** List of event types. */
static const rcl_lua_enum enum_event_types[] = {
  {"EXECUTE", GOAL_EVENT_EXECUTE},
  {"CANCEL_GOAL", GOAL_EVENT_CANCEL_GOAL},
  {"SUCCEED", GOAL_EVENT_SUCCEED},
  {"ABORT", GOAL_EVENT_ABORT},
  {"CANCELED", GOAL_EVENT_CANCELED},
  {NULL, -1}
};

/* Add actoin server and goal handle to library. */
void rcl_lua_add_action_server_methods(lua_State * L)
{
  /* constructor */
  lua_pushcfunction(L, rcl_lua_action_server_init);  // push function
  lua_setfield(L, -2, "new_action_server");          // pop, lib['new_action_server'] = function

  /* metamethods */
  rcl_lua_utils_add_mt(L, MT_ACTION_SERVER, act_srv_methods);

  /* goal handle constructor */
  lua_pushcfunction(L, rcl_lua_action_goal_handle_init);
  lua_setfield(L, -2, "new_action_goal_handle");

  /* goal handle metamethods */
  rcl_lua_utils_add_mt(L, MT_ACTION_GOAL_HANDLE, act_srv_handle_methods);

  /* events */
  rcl_lua_utils_add_enum(L, "GoalEvent", enum_event_types);
}
