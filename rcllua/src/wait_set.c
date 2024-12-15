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

#include <rcl/wait.h>
#include <rcl/allocator.h>
#include <rcl/error_handling.h>

#include "wait_set.h"
#include "context.h"
#include "timer.h"
#include "subscriber.h"
#include "service.h"
#include "client.h"
#include "utils.h"

/** WaitSet object metatable name. */
const char* MT_WAIT_SET = "ROS2.WaitSet";

/**
 * Init WaitSet object.
 *
 * Arguments:
 * - subscriptions number
 * - guard conditiona number
 * - timers number
 * - clients number
 * - services number
 * - events number
 *
 * Return:
 * - WaitSet object.
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_wait_set_init (lua_State* L)
{
  /* arg1 - #subsctiptions */
  int num_sub = luaL_checkinteger(L, 1);
  /* arg2 - #guard_conditions */
  int num_guard = luaL_checkinteger(L, 2);
  /* arg3 - #timers */
  int num_timers = luaL_checkinteger(L, 3);
  /* arg4 - #clients */
  int num_cli = luaL_checkinteger(L, 4);
  /* arg5 - #services */
  int num_srv = luaL_checkinteger(L, 5);
  /* arg6 - #events */
  int num_ev = luaL_checkinteger(L, 6);

  if (num_sub < 0 || num_guard < 0 || num_timers < 0 || num_cli < 0
   || num_srv < 0 || num_ev < 0
  ) {
    luaL_error(L, "negative number of items");
  }

  /* make */
  rcl_wait_set_t* wait_set = lua_newuserdata(L, sizeof(rcl_wait_set_t));  // push object
  *wait_set = rcl_get_zero_initialized_wait_set();
  rcl_ret_t ret = rcl_wait_set_init(
    wait_set,
    (size_t) num_sub,
    (size_t) num_guard,
    (size_t) num_timers,
    (size_t) num_cli,
    (size_t) num_srv,
    (size_t) num_ev,
    rcl_lua_context_ref(),
    rcl_get_default_allocator());
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to initialize wait set");
  }

  /* metamethods */
  luaL_getmetatable(L, MT_WAIT_SET);  // push metatable
  lua_setmetatable(L, -2);            // pop metatable

  return 1;
}

/**
 * WaitSet destructor.
 *
 * Arguments:
 * - WaitSet object
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_wait_set_free (lua_State* L)
{
  /* arg1 - waitset object */
  rcl_wait_set_t* ws = lua_touserdata(L, 1);

  rcl_ret_t ret = rcl_wait_set_fini(ws);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to fini wait set: %s", rcl_get_error_string().str);
    rcl_reset_error();
  }

  return 0;
}

/**
 * Clear WaitSet object.
 *
 * Arguments:
 * - WaitSet object
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_wait_set_clear (lua_State* L)
{
  /* arg1 - waitset object */
  rcl_wait_set_t* ws = luaL_checkudata(L, 1, MT_WAIT_SET);

  rcl_ret_t ret = rcl_wait_set_clear(ws);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to clear wait set");
  }

  return 0;
}

/**
 * Add timer for waiting.
 *
 * Arguments:
 * - WaitSet object
 * - timer object
 *
 * Return:
 * - index of added timer
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_wait_set_add_timer (lua_State* L)
{
  /* arg1 - wait set */
  rcl_wait_set_t* ws = luaL_checkudata(L, 1, MT_WAIT_SET);
  /* arg2 - timer */
  rcl_timer_t* timer = luaL_checkudata(L, 2, MT_TIMER);

  /* add */
  size_t index = 0;
  rcl_ret_t ret = rcl_wait_set_add_timer(ws, timer, &index);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to add timer");
  }

  lua_pushinteger(L, (lua_Integer) index);
  return 1;
}

/**
 * Add subscription for waiting.
 *
 * Arguments:
 * - WaitSet object
 * - subscription object
 *
 * Return:
 * - index of added subscription
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_wait_set_add_subscription (lua_State* L)
{
  /* arg1 - wait set */
  rcl_wait_set_t* ws = luaL_checkudata(L, 1, MT_WAIT_SET);
  /* arg2 - subsctiption */
  rcl_subscription_t* sub = luaL_checkudata(L, 2, MT_SUBSCRIPTION);

  /* add */
  size_t index = 0;
  rcl_ret_t ret = rcl_wait_set_add_subscription(ws, sub, &index);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to add subscription");
  }

  lua_pushinteger(L, (lua_Integer) index);
  return 1;
}

/**
 * Add service for waiting.
 *
 * Arguments:
 * - WaitSet object
 * - service object
 *
 * Return:
 * - index of added service
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_wait_set_add_service (lua_State* L)
{
  /* arg1 - wait set */
  rcl_wait_set_t* ws = luaL_checkudata(L, 1, MT_WAIT_SET);
  /* arg2 - service */
  rcl_service_t* srv = luaL_checkudata(L, 2, MT_SERVICE);
  
  /* add */
  size_t index = 0;
  rcl_ret_t ret = rcl_wait_set_add_service(ws, srv, &index);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to add service");
  }

  lua_pushinteger(L, (lua_Integer) index);
  return 1;
}

/**
 * Add client for waiting.
 *
 * Arguments:
 * - WaitSet object
 * - client object
 *
 * Return:
 * - index of added client
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_wait_set_add_client (lua_State* L)
{
  /* arg1 - wait set */
  rcl_wait_set_t* ws = luaL_checkudata(L, 1, MT_WAIT_SET);
  /* arg2 - client */
  rcl_client_t* cli = luaL_checkudata(L, 2, MT_CLIENT);
  
  /* add */
  size_t index = 0;
  rcl_ret_t ret = rcl_wait_set_add_client(ws, cli, &index);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to add client");
  }

  lua_pushinteger(L, (lua_Integer) index);
  return 1;
}

/**
 * Waiting for the next ready object.
 *
 * Arguments:
 * - WaitSet object
 * - timeout  (??)
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_wait_set_wait (lua_State* L)
{
  /* arg1 - wait set */
  rcl_wait_set_t* ws = luaL_checkudata(L, 1, MT_WAIT_SET);
  /* arg2 - timeout */
  lua_Integer timeout = luaL_checkinteger(L, 2);

  /* waiting */
  rcl_ret_t ret = rcl_wait(ws, timeout);
  if (RCL_RET_OK != ret && RCL_RET_TIMEOUT != ret) {
    luaL_error(L, "failed to wait on wait set");
  }

  return 0;
}

/**
 * Collect ready timers.
 *
 * Arguments:
 * - WaitSet object
 *
 * Return:
 * - table of callback functions
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_wait_set_ready_timers (lua_State* L)
{
  /* arg1 - wait set */
  rcl_wait_set_t* ws = luaL_checkudata(L, 1, MT_WAIT_SET);

  /* collect functions */
  lua_createtable(L, ws->size_of_timers, 0);           // push table
  for (size_t i = 0; i < ws->size_of_timers; i++) {
    lua_rawgetp(L, LUA_REGISTRYINDEX, ws->timers[i]);  // push function
    lua_rawseti(L, -2, i+1);                           // pop function
  }

  return 1;
}

/**
 * Collect ready subscriptions.
 *
 * Arguments:
 * - WaitSet object
 *
 * Return:
 * - table of tuples (message, callback)
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_wait_set_ready_subscription (lua_State* L)
{
  /* arg1 - wait set */
  rcl_wait_set_t* ws = luaL_checkudata(L, 1, MT_WAIT_SET);

  /* collect functions */
  lua_createtable(L, ws->size_of_subscriptions, 0);  // push table a
  for (size_t i = 0; i < ws->size_of_subscriptions; i++) {
    /* table {message, function} */
    rcl_lua_subscription_push_callback(L, ws->subscriptions[i]);  // push table b
    lua_rawseti(L, -2, i+1);                         // pop table b
  }

  return 1;
}

/**
 * Collect ready client response.
 *
 * Arguments:
 * - WaitSet object
 *
 * Return:
 * - table of tuples (response, callback)
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_wait_set_ready_clients (lua_State* L)
{
  /* arg1 - wait set */
  rcl_wait_set_t* ws = luaL_checkudata(L, 1, MT_WAIT_SET);

  /* collect services */
  lua_createtable(L, ws->size_of_clients, 0);  // push table a
  for (size_t i = 0; i < ws->size_of_clients; i++) {
    /* add table */
    rcl_lua_client_push_response(L, ws->clients[i]);  // push table b
    lua_rawseti(L, -2, i+1);                   // pop table b
  }

  return 1;
}

/**
 * Collect ready service requests.
 *
 * Arguments:
 * - WaitSet object
 *
 * Return:
 * - table of tuples (response, response, ...)
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_wait_set_ready_services (lua_State* L)
{
  /* arg1 - wait set */
  rcl_wait_set_t* ws = luaL_checkudata(L, 1, MT_WAIT_SET);

  /* collect services */
  lua_createtable(L, ws->size_of_services, 0);  // push table a
  for (size_t i = 0; i < ws->size_of_services; i++) {
    /* add table */
    rcl_lua_service_push_callback(L, ws->services[i]);  // push table b
    lua_rawseti(L, -2, i+1);                   // pop table b
  }

  return 1;
}

/** List of WaitSet methods */
static const struct luaL_Reg wait_set_methods[] = {
  {"add_timer", rcl_lua_wait_set_add_timer},
  {"add_subscription", rcl_lua_wait_set_add_subscription},
  {"add_service", rcl_lua_wait_set_add_service},
  {"add_client", rcl_lua_wait_set_add_client},
  {"ready_timers", rcl_lua_wait_set_ready_timers},
  {"ready_subscriptions", rcl_lua_wait_set_ready_subscription},
  {"ready_services", rcl_lua_wait_set_ready_services},
  {"ready_clients", rcl_lua_wait_set_ready_clients},
  {"clear", rcl_lua_wait_set_clear},
  {"wait", rcl_lua_wait_set_wait},
  {"__gc", rcl_lua_wait_set_free},
  {NULL, NULL}
};

/* Add to library */
void rcl_lua_add_wait_set_methods (lua_State* L)
{
  /* wait set constructor*/
  lua_pushcfunction(L, rcl_lua_wait_set_init);  // push function
  lua_setfield(L, -2, "new_wait_set");          // pop, lib['new_wait_set'] = function

  /* metamethods */
  rcl_lua_utils_add_mt(L, MT_WAIT_SET, wait_set_methods);
}
