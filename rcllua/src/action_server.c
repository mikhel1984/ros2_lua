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
#include <rcl/error_handling.h>

#include <rosidl_luacommon/definition.h>

#include "rcllua/action_server.h"
#include "rcllua/node.h"
#include "rcllua/clock.h"
#include "rcllua/wait_set.h"
#include "rcllua/qos.h"
#include "rcllua/utils.h"


/** Indices of action server binding in register. */
enum ActSrvReg {
  /** node reference */
  ACT_SRV_REG_NODE = 1,

  ACT_SRV_REG_INTERFACE,

  ACT_SRV_REG_EXEC_CB,

  ACT_SRV_REG_GOAL_CB,
  ACT_SRV_REG_ACCEPT_CB,
  ACT_SRV_REG_CANCEL_CB,

  ACT_SRV_REG_FEEDBACK_MT,

  /** number of elements + 1 */
  ACT_SRV_REG_NUMBER
};

/** List of output elements. */
enum ActSrvOut {
  /** request message */
  ACT_SRV_OUT_REQUEST = 1,
  /** callback funciton */
  ACT_SRV_OUT_CALLBACK,
  /** header */
  ACT_SRV_OUT_HEADER,
  /** number of elements + 1 */
  ACT_SRV_OUT_NUMBER
};


const char* MT_ACTION_SERVER = "ROS2.ActionServer";
const char* MT_ACTION_GOAL_HANDLE = "ROS2.ActionGoalHandle";

static int rcl_lua_action_server_init (lua_State* L)
{
  /* arg1 - node */
  rcl_node_t* node = luaL_checkudata(L, 1, MT_NODE);

  /* arg2 - clock */
  rcl_clock_t* clock = luaL_checkudata(L, 2, MT_CLOCK);

  /* arg3 - action type */
  rosidl_action_type_support_t* ts = NULL;
  /* check table */
  if (lua_istable(L, 3)) {
    lua_getfield(L, 3, "_type_support");   // push pointer
    if (lua_islightuserdata(L, -1)) {
      ts = lua_touserdata(L, -1);
      lua_pop(L, 1);                       // pop pointer
    }
  }
  if (NULL == ts) {
    luaL_argerror(L, 3, "expected action type");
  }

  /* arg4 - action service name */
  const char* srv_name = luaL_checkstring(L, 4);

  /* arg5 - parameters table */
  rcl_action_server_options_t action_server_ops = rcl_action_server_get_default_options();
  if (lua_istable(L, 5)) {
    /* QoS */
    if (lua_getfield(L, 5, "goal_service_qos") != LUA_TNIL) {
      action_server_ops.goal_service_qos = *((rmw_qos_profile_t*) luaL_checkudata(L, -1, MT_QOS));
    }
    lua_pop(L, 1);
    if (lua_getfield(L, 5, "result_service_qos") != LUA_TNIL) {
      action_server_ops.result_service_qos = *((rmw_qos_profile_t*) luaL_checkudata(L, -1, MT_QOS));
    }
    lua_pop(L, 1);
    if (lua_getfield(L, 5, "cancel_service_qos") != LUA_TNIL) {
      action_server_ops.cancel_service_qos = *((rmw_qos_profile_t*) luaL_checkudata(L, -1, MT_QOS));
    }
    lua_pop(L, 1);
    if (lua_getfield(L, 5, "feedback_topic_qos") != LUA_TNIL) {
      action_server_ops.feedback_topic_qos = *((rmw_qos_profile_t*) luaL_checkudata(L, -1, MT_QOS));
    }
    lua_pop(L, 1);
    if (lua_getfield(L, 5, "status_topic_qos") != LUA_TNIL) {
      action_server_ops.status_topic_qos = *((rmw_qos_profile_t*) luaL_checkudata(L, -1, MT_QOS));
    }
    lua_pop(L, 1);
    /* timeout */
    if (lua_getfield(L, 5, "result_timeout") != LUA_TNIL) {
      action_server_ops.result_timeout.nanoseconds = (rcl_duration_value_t) RCL_S_TO_NS(luaL_checknumber(L, -1));
    } else {
      action_server_ops.result_timeout.nanoseconds = (rcl_duration_value_t) 900;  // seconds
    }
    lua_pop(L, 1);
  }

  /* arg6 - execute callback */
  luaL_argcheck(L, lua_isfunction(L, 6), 6, "function is expected");

  /* new action server */
  rcl_action_server_t *srv = lua_newuserdata(L, sizeof(rcl_action_server_t));
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

  /* save reference objects */
  lua_createtable(L, 0, ACT_SRV_REG_NUMBER-1);  // push table a

  lua_pushvalue(L, 1);                   // push node
  lua_rawseti(L, -2, ACT_SRV_REG_NODE);  // pop node

  /* callbacks */
  if (lua_istable(L, 5)) {
    if (lua_getfield(L, 5, "goal_callback") != LUA_TNIL) {  // push
      lua_rawseti(L, -2, ACT_SRV_REG_GOAL_CB);    // pop
    }
    if (lua_getfield(L, 5, "handle_accepted_callback") != LUA_TNIL) {  // push
      lua_rawseti(L, -2, ACT_SRV_REG_ACCEPT_CB);  // pop
    }
    if (lua_getfield(L, 5, "cancel_callback") != LUA_TNIL) {  // push
      lua_rawseti(L, -2, ACT_SRV_REG_CANCEL_CB);  // pop
    }
  }

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
  lua_rawseti(L, -2, ACT_SRV_REG_INTERFACE);  // pop interface

  lua_getfield(L, 2, "FeedbackMessage");
  lua_getfield(L, -1, "_metatable");
  lua_rawseti(L, -3, ACT_SRV_REG_FEEDBACK_MT);
  lua_pop(L, 1);

  lua_rawsetp(L, LUA_REGISTRYINDEX, srv);  // pop table, save to registry

  return 1;
}

static int rcl_lua_action_server_free (lua_State* L)
{
  /* arg1 - action server */
  rcl_action_server_t* srv = lua_touserdata(L, 1);

  /* get node */
  lua_rawgetp(L, LUA_REGISTRYINDEX, srv);  // push table
  lua_rawgeti(L, -1, ACT_SRV_REG_NODE);        // push node
  rcl_node_t* node = lua_touserdata(L, -1);

  /* finalize */
  rcl_ret_t ret = rcl_action_server_fini(srv, node);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to fini server: %s", rcl_get_error_string().str);
  }

  /* free dependencies */
  lua_pushnil(L);
  lua_rawsetp(L, LUA_REGISTRYINDEX, srv);

  return 0;
}

/**
 * Get message constructor for the specific action structure.
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
static int rcl_lua_action_server_get_interface (lua_State* L)
{
  /* arg1 - action client */
  rcl_action_server_t* srv = luaL_checkudata(L, 1, MT_ACTION_SERVER);
  /* arg2 - interface name */
  const char* type = luaL_checkstring(L, 2);

  lua_rawgetp(L, LUA_REGISTRYINDEX, srv);
  lua_rawgeti(L, -1, ACT_SRV_REG_INTERFACE);

  lua_getfield(L, -1, type);
  return 1;
}


#define TAKE_SERVICE_REQUEST(Type, TBL_NAME, CB_REQ_ID) \
  /* arg1 - action server */ \
  rcl_action_server_t* srv = lua_touserdata(L, 1); \
  lua_createtable(L, ACT_SRV_OUT_NUMBER-1, 0); \
  /* prepare request message */ \
  lua_rawgetp(L, LUA_REGISTRYINDEX, srv); \
  lua_rawgeti(L, -1, ACT_SRV_REG_INTERFACE); \
  lua_getfield(L, -1, TBL_NAME); \
  lua_getfield(L, -1, "Request"); \
  lua_getfield(L, -1, "_new"); \
  lua_rotate(L, -3, 1); lua_pop(L, 2); \
  lua_call(L, 0, 1); \
  idl_lua_msg_t *msg = lua_touserdata(L, -1); \
  /* get request */ \
  rmw_request_id_t header; \
  rcl_ret_t ret = rcl_action_take_ ## Type ## _request(srv, &header, msg->obj); \
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
  rmw_request_id_t* info = lua_newuserdata(L, sizeof(rmw_request_id_t)); \
  *info = header; \
  lua_rawseti(L, -3, ACT_SRV_OUT_HEADER); \
  /* save callback function */ \
  lua_rawgeti(L, -1, CB_REQ_ID); \
  lua_rawseti(L, -3, ACT_SRV_OUT_CALLBACK); \
  lua_pop(L, 1); \
  return 1;
 
static int rcl_lua_action_server_goal_request (lua_State* L)
{
  TAKE_SERVICE_REQUEST(goal, "SendGoal", ACT_SRV_REG_GOAL_CB)
}

static int rcl_lua_action_server_result_request (lua_State* L)
{
  TAKE_SERVICE_REQUEST(result, "GetResult", -1)
}

static int rcl_lua_action_server_cancel_request (lua_State* L)
{
  TAKE_SERVICE_REQUEST(cancel, "CancelGoal", ACT_SRV_REG_CANCEL_CB)
}

#define SEND_SERVICE_RESPONSE(Type) \
  /* arg1 - action server */ \
  rcl_action_server_t* srv = lua_touserdata(L, 1); \
  /* arg2 - response */ \
  idl_lua_msg_t* resp = lua_touserdata(L, 2); \
  /* arg3 - header */ \
  rmw_request_id_t* header = lua_touserdata(L, 3); \
  /* send response */ \
  rcl_ret_t ret = rcl_action_send_ ## Type ## _response(srv, header, resp->obj); \
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

static int rcl_lua_action_server_goal_response (lua_State* L)
{
  SEND_SERVICE_RESPONSE(goal)
}

static int rcl_lua_action_server_result_response (lua_State* L)
{
  SEND_SERVICE_RESPONSE(result)
}

static int rcl_lua_action_server_cancel_response (lua_State* L)
{
  SEND_SERVICE_RESPONSE(cancel)
}

static int rcl_lua_action_server_get_exec (lua_State* L)
{
  /* arg1 - action server */
  rcl_action_server_t* srv = lua_touserdata(L, 1);

  lua_rawgetp(L, LUA_REGISTRYINDEX, srv);  // push table
  lua_rawgeti(L, -1, ACT_SRV_REG_EXEC_CB);  // push method for execution

  return 1;
}

static int rcl_lua_action_server_publish_feedback (lua_State* L)
{
  /* arg1 - action server */
  rcl_action_server_t* srv = luaL_checkudata(L, 1, MT_ACTION_SERVER);

  /* arg2 - message object */
  lua_rawgetp(L, LUA_REGISTRYINDEX, srv);  // push table
  lua_rawgeti(L, -1, ACT_SRV_REG_FEEDBACK_MT);  // push metatable name
  const char* mt = lua_tostring(L, -1);
  idl_lua_msg_t *msg = luaL_checkudata(L, 2, mt);

  /* send */
  rcl_ret_t ret = rcl_action_publish_feedback(srv, msg->obj);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to publish feedback");
  }

  return 0;
}

static int rcl_lua_action_server_publish_status (lua_State* L)
{
  /* arg1 - action server */
  rcl_action_server_t* srv = lua_touserdata(L, 1);

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
    luaL_error(L, "failed to finalize goal status array: %s", rcl_get_error_string().str);
  }

  return 0;
}

static int rcl_lua_action_server_notify_goal_done (lua_State* L)
{
  /* arg1 - action server */
  rcl_action_server_t* srv = lua_touserdata(L, 1);

  rcl_ret_t ret = rcl_action_notify_goal_done(srv);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to notify action server of goal done");
  }

  return 0;
}

static int rcl_lua_action_server_num_entities (lua_State* L)
{
  /* arg1 - action server */
  rcl_action_server_t* srv = lua_touserdata(L, 1);

  /* subscriptions, guards, timers, clients, services */
  size_t count[5] = {0, 0, 0, 0, 0};
  rcl_ret_t ret = rcl_action_server_wait_set_get_num_entities(
    srv, count, count+1, count+2, count+3, count+4);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to get number of entities");
  }
  
  for(size_t i = 0; i < 5; i++) {
    lua_pushinteger(L, count[i]);
  }
  return 5;
}

static int rcl_lua_action_server_is_ready (lua_State* L)
{
  /* arg1 - action server */
  rcl_action_server_t* srv = luaL_checkudata(L, 1, MT_ACTION_SERVER);
  /* arg2 - WaitSet */
  rcl_wait_set_t* wait_set = luaL_checkudata(L, 2, MT_WAIT_SET);

  /* goal_req, cancel_req, result_req, goal_expired */
  bool status[4] = {false, false, false, false};
  rcl_ret_t ret = rcl_action_server_wait_set_get_entities_ready(
    wait_set, srv, status, status+1, status+2, status+3);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to get ready action server entries");
  }

  for(size_t i = 0; i < 4; i++) {
    lua_pushboolean(L, status[i]);
  }
  return 4;
}

static int rcl_lua_action_server_add_waitset (lua_State* L)
{
  /* arg1 - action server */
  rcl_action_server_t* srv = luaL_checkudata(L, 1, MT_ACTION_SERVER);
  /* arg2 - WaitSet */
  rcl_wait_set_t* wait_set = luaL_checkudata(L, 2, MT_WAIT_SET);

  rcl_ret_t ret = rcl_action_wait_set_add_action_server(wait_set, srv, NULL);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to add action server to wait set");
  }

  return 0;
}

//static int rcl_lua_action_server_proc_cancel_request (lua_State* L)
//{
//  /* arg1 - action server */
//  rcl_action_server_t* srv = luaL_checkudata(L, 1, MT_ACTION_SERVER);
//  /* arg2 - cancel request */
//  idl_lua_msg_t* req = lua_touserdata(L, 1);
//
//  /* make response message */
//  lua_rawgetp(L, LUA_REGISTRYINDEX, srv);
//  lua_rawgeti(L, -1, ACT_SRV_REG_INTERFACE);
//  lua_getfield(L, -1, "CancelGoal");
//  lua_getfield(L, -1, "Response");
//  lua_getfield(L, -1, "_new");
//  lua_rotate(L, -3, 1); lua_pop(L, 2);
//  lua_call(L, 0, 1);
//  idl_lua_msg_t* resp = lua_touserdata(L, -1);
//
//  rcl_action_cancel_response_t rcl_resp = rcl_action_get_zero_initialized_cancel_response();
//
//  rcl_ret_t ret = rcl_action_process_cancel_request(srv, req->obj, resp->obj);
//  if (RCL_RET_OK != ret) {
//    luaL_error(L, "Failed to process cancel request");
//  }
//
//  return 1;
//}

//static int rcl_lua_actoin_server_expire_goals (lua_State* L)
//{
//  /* arg1 - action server */
//  rcl_action_server_t* srv = luaL_checkudata(L, 1, MT_ACTION_SERVER);
//  /* arg2 - max number */
//  int max_goals = luaL_checkinteger(L, 2);
//  luaL_argcheck(L, max_goals > 0, 2, "expected positive number");
//
//  rcl_action_goal_info_t* expired_goals = 
//    lua_newuserdata(L, max_goals*sizeof(rcl_action_goal_info_t));
//  size_t num_expired = 0;
//  rcl_ret_t ret = rcl_action_expire_goals(srv, expired_goals, max_goals, &num_expired);
//  if (RCL_RET_OK != ret) {
//    luaL_error(L, "failed to expire goals");
//  }
//}

static int rcl_lua_action_goal_handle_init (lua_State* L)
{
  /* arg1 - action server */
  rcl_action_server_t* srv = luaL_checkudata(L, 1, MT_ACTION_SERVER);
  /* arg2 - GoalInfo message */
  luaL_argcheck(L, lua_isuserdata(L, 2), 2, "expected GoalInfo message");
  idl_lua_msg_t* msg = lua_touserdata(L, 2);

  rcl_action_goal_info_t* goal_info_ptr = (rcl_action_goal_info_t*) msg->obj;
  rcl_action_goal_handle_t* rcl_handle = rcl_action_accept_new_goal(srv, goal_info_ptr);
  if (!goal_info_ptr) {
    luaL_error(L, "failed to accept new goal");
  }

  rcl_action_goal_handle_t* action_goal_handle = lua_newuserdata(L, sizeof(rcl_action_goal_handle_t));
  *action_goal_handle = *rcl_handle;

  /* set metatable */
  luaL_getmetatable(L, MT_ACTION_GOAL_HANDLE);
  lua_setmetatable(L, -2);

  return 1;
}

static int rcl_lua_action_goal_handle_free (lua_State* L)
{
  /* arg1 - action goal handle */
  rcl_action_goal_handle_t* handle = lua_touserdata(L, 1);

  rcl_ret_t ret = rcl_action_goal_handle_fini(handle);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to fini action goal handle");
  }

  return 0;
}

static int rcl_lua_action_goal_handle_get_status (lua_State* L)
{
  /* arg1 - action goal handle */
  rcl_action_goal_handle_t* handle = lua_touserdata(L, 1);

  rcl_action_goal_state_t status;
  rcl_ret_t ret = rcl_action_goal_handle_get_status(handle, &status);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to get goal status");
  }

  lua_pushinteger(L, status);
  return 1;
}

static int rcl_lua_action_goal_handle_set_status (lua_State* L)
{
  /* arg1 - action goal handle */
  rcl_action_goal_handle_t* handle = luaL_checkudata(L, 1, MT_ACTION_GOAL_HANDLE);
  /* arg2 - event */
  lua_Integer ev = luaL_checkinteger(L, 2);

  rcl_action_goal_event_t event = ev;
  rcl_ret_t ret = rcl_action_update_goal_state(handle, event);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to update goal status");
  }

  return 0;
}

static int rcl_lua_action_goal_handle_is_active (lua_State* L)
{
  /* arg1 - action goal handle */
  rcl_action_goal_handle_t* handle = lua_touserdata(L, 1);

  lua_pushboolean(L, rcl_action_goal_handle_is_active(handle));
  return 1;
}


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
  {NULL, NULL}
};

static const struct luaL_Reg act_srv_handle_methods[] = {
  {"__gc", rcl_lua_action_goal_handle_free},
  {"get_status", rcl_lua_action_goal_handle_get_status},
  {"update_status", rcl_lua_action_goal_handle_set_status},
  {"is_active", rcl_lua_action_goal_handle_is_active},
  {NULL, NULL}
};


void rcl_lua_add_action_server_methods (lua_State* L)
{
  /* constructor */
  lua_pushcfunction(L, rcl_lua_action_server_init);  // push function
  lua_setfield(L, -2, "new_action_server");          // pop, lib['new_action_server'] = function

  /* metamethods */
  rcl_lua_utils_add_mt(L, MT_ACTION_SERVER, act_srv_methods);

  lua_pushcfunction(L, rcl_lua_action_goal_handle_init);
  lua_setfield(L, -2, "new_action_goal_handle");

  /* metamethods */
  rcl_lua_utils_add_mt(L, MT_ACTION_GOAL_HANDLE, act_srv_handle_methods);
}
