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

#include "rcllua/node.h"
#include "rcllua/clock.h"
#include "rcllua/wait_set.h"
#include "rcllua/utils.h"

/** Indices of action server binding in register. */
enum ActSrvReg {
  /** node reference */
  ACT_SRV_REG_NODE = 1,
  ACT_SRV_REG_GOAL_NEW_REQ,
  ACT_SRV_REG_GOAL_NEW_RESP,
  ACT_SRV_REG_GOAL_CB,
  ACT_SRV_REG_RESULT_NEW_REQ,
  ACT_SRV_REG_RESULT_NEW_RESP,
  ACT_SRV_REG_RESULT_CB,
  ACT_SRV_REG_CANCEL_NEW_REQ,
  ACT_SRV_REG_CANCEL_NEW_RESP,
  ACT_SRV_REG_CANCEL_CB,
  ACT_SRV_REG_FEEDBACK_MT,
  /** number of elements + 1 */
  ACT_SRV_REG_NUMBER
}

/** List of output elements. */
enum ActSrvOut {
  /** request message */
  ACT_SRV_OUT_REQUEST = 1,
  /** response message */
  ACT_SRV_OUT_RESPONSE,
  /** callback funciton */
  ACT_SRV_OUT_CALLBACK,
  /** header */
  ACT_SRV_OUT_HEADER,
  /** light userdata */
  ACT_SRV_OUT_REF,
  /** number of elements + 1 */
  ACT_SRV_OUT_NUMBER
};


const char* MT_ACTION_SERVER = "ROS2.ActionServer";

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

  /* arg5 - QoS table */
  rcl_action_server_options_t action_server_ops = rcl_action_server_get_default_options();
  if (lua_istable(L, 5)) {
    lua_getfield(L, 5, "goal_service_qos");
    if (!lua_isnil(L, -1)) {
      action_server_ops.goal_service_qos = *((rmw_qos_profile_t*) luaL_checkudata(L, -1, MT_QOS));
    }
    lua_pop(L, 1);
    lua_getfield(L, 5, "result_service_qos");
    if (!lua_isnil(L, -1)) {
      action_server_ops.result_service_qos = *((rmw_qos_profile_t*) luaL_checkudata(L, -1, MT_QOS));
    }
    lua_pop(L, 1);
    lua_getfield(L, 5, "cancel_service_qos");
    if (!lua_isnil(L, -1)) {
      action_server_ops.cancel_service_qos = *((rmw_qos_profile_t*) luaL_checkudata(L, -1, MT_QOS));
    }
    lua_pop(L, 1);
    lua_getfield(L, 5, "feedback_topic_qos");
    if (!lua_isnil(L, -1)) {
      action_server_ops.feedback_topic_qos = *((rmw_qos_profile_t*) luaL_checkudata(L, -1, MT_QOS));
    }
    lua_pop(L, 1);
    lua_getfield(L, 5, "status_topic_qos");
    if (!lua_isnil(L, -1)) {
      action_server_ops.status_topic_qos = *((rmw_qos_profile_t*) luaL_checkudata(L, -1, MT_QOS));
    }
    lua_pop(L, 1);
  }

  /* arg6 - result timeout, sec */
  if (lua_isnumber(L, 6)) {
    action_server_ops.result_timeout.nanoseconds = (rcl_duration_value_t) RCL_S_TO_NS(lua_tonumberx(L, 6));
  }

  /* new action server */
  rcl_action_server_t *srv = lua_newuserdata(L, sizeof(rcl_action_server_t));
  *srv = rcl_action_get_zero_initialized_server();

  rcl_ret_t ret = rcl_action_server_init(src, node, clock, ts, srv_name, &action_server_ops);
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

  lua_rawsetp(L, LUA_REGISTRYINDEX, srv);  // pop table, save to registry

  return 1;
}

static int rcl_lua_action_server_free (lua_State* L)
{
  /* arg1 - action server */
  rcl_action_server_t* srv = lua_touserdata(L, 1);

  /* get node */
  lua_rawgetp(L, LUA_REGISTRYINDEX, cli);  // push table
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

#define TAKE_SERVICE_REQUEST(Type, NEW_REQ_ID, NEW_RESP_ID, CB_ID) \
  lua_createtable(L, ACT_SRV_OUT_NUMBER-1, 0); \
  /* save pointer */ \
  lua_pushlightuserdata(L, (void*) srv); \
  lua_rawseti(L, -2, ACT_SRV_OUT_REF); \
  /* prepare request message */ \
  lua_rawgetp(L, LUA_REGISTRYINDEX, srv); \
  if (lua_isnil(L, -1)) { \
    luaL_error(L, "action service binginds not found"); \
  } \
  lua_rawgeti(L, -1, NEW_REQ_ID); \
  lua_call(L, 0, 1); \
  idl_lua_msg_t *msg = lua_touserdata(L, -1); \
  /* get request */ \
  rmw_request_id_t header; \
  rcl_ret_t ret = rcl_action_take_ ## Type ## _request(srv, &header, msg->obj); \
  switch (ret) { \
    case RCL_RET_OK: break; \
    case RCL_RET_ACTION_CLIENT_TAKE_FAILED: \
    case RCL_RET_ACTION_SERVICE_TAKE_FAILED: \
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
  /* add response */ \
  lua_rawgeti(L, -1, NEW_RESP_ID); \
  lua_call(L, 0, 1); \
  lua_rawseti(L, -3, ACT_SRV_OUT_RESPONSE); \
  /* save callback function */ \
  lua_rawgeti(L, -1, CB_ID); \
  lua_rawseti(L, -3, ACT_SRV_OUT_CALLBACK); \
  lua_pop(L, 1);  
 
static int rcl_lua_action_server_goal_request (lua_State* L)
{
  TAKE_SERVICE_REQUEST(goal, ACT_SRV_REG_GOAL_NEW_REQ, ACT_SRV_REG_GOAL_NEW_RESP, ACT_SRV_REG_GOAL_CB)
}

static int rcl_lua_action_server_result_request (lua_State* L)
{
  TAKE_SERVICE_REQUEST(result, ACT_SRV_REG_RESULT_NEW_REQ, ACT_SRV_REG_RESULT_NEW_RESP, ACT_SRV_REG_RESULT_CB)
}

static int rcl_lua_action_server_cancel_request (lua_State* L)
{
  TAKE_SERVICE_REQUEST(cancel, ACT_SRV_REG_CANCEL_NEW_REQ, ACT_SRV_REG_CANCEL_NEW_RESP, ACT_SRV_REG_CANCEL_CB)
}

#define SEND_SERVICE_RESPONSE(Type) \
  luaL_argcheck( \
    L, LUA_TTABLE == lua_type(L, 1) && lua_rawlen(L, 1) == (ACT_SRV_OUT_NUMBER-1), 1, \
    "expected table from action service request"); \
  /* get required elements */ \
  lua_rawgeti(L, 1, ACT_SRV_OUT_REF); \
  rcl_action_server_t* srv = luaL_checkudata(L, 1, MT_ACTION_SERVER); \
  lua_rawgeti(L, 1, ACT_SRV_OUT_RESPONSE); \
  idl_lua_msg_t *resp = lua_touserdata(L, -1); \
  lua_rawgeti(L, 1, ACT_SRV_OUT_HEADER); \
  rmw_request_id_t* header = lua_touserdata(L, -1); \
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

  return 0;
}

static int rcl_lua_action_server_num_entities (lua_State* L)
{
  /* arg1 - action server */
  rcl_action_server_t* srv = lua_touserdata(L, 1);

  /* subscriptions, guards, timers, clients, services */
  size_t count[] = {0, 0, 0, 0, 0, 0};
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
  bool status[] = {false, false, false, false};
  rcl_ret_t ret = rcl_action_server_wait_set_get_entities_ready(
    wait_set, srv, status, status+1, status+2, status+3, status+4);
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

static int rcl_lua_action_server_proc_cancel_request (lua_State* L)
{
  return 0;
}

static int rcl_lua_actoin_server_expire_goals (lua_State* L)
{
  /* arg1 - action server */
  rcl_action_server_t* srv = luaL_checkudata(L, 1, MT_ACTION_SERVER);
  /* arg2 - max number */
  int max_goals = luaL_checkinteger(L, 2);
  luaL_argcheck(L, max_goals > 0, 2, "expected positive number");

  rcl_action_goal_info_t* expired_goals = 
    lua_newuserdata(L, max_goals*sizeof(rcl_action_goal_info_t));
  size_t num_expired = 0;
  rcl_ret_t ret = rcl_action_expire_goals(srv, expired_goals, max_goals, &num_expired);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to expire goals");
  }
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
  {"get_num_entities", rcl_lua_action_server_num_entities},
  {"is_ready", rcl_lua_action_server_is_ready},
  {"add_to_waitset", rcl_lua_action_server_add_waitset},
  {NULL, NULL}
};

void rcl_lua_add_action_server_methods (lua_State* L)
{
  /* constructor */
  lua_pushcfunction(L, rcl_lua_action_server_init);  // push function
  lua_setfield(L, -2, "new_action_server");          // pop, lib['new_action_server'] = function

  /* metamethods */
  rcl_lua_utils_add_mt(L, MT_ACTION_SERVER, act_srv_methods);
}
