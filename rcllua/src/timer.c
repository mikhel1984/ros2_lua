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

#include <rcl/timer.h>
#include <rcl/error_handling.h>

#include "rcllua/timer.h"
#include "rcllua/clock.h"
#include "rcllua/context.h"
#include "rcllua/utils.h"

/* Check timer status. */
static int rcl_lua_timer_push_ready (lua_State* L, const rcl_timer_t* timer);

/** Indices of output elements. */
enum TmOut {
  /** callback function */
  TM_OUT_CALLBACK = 1,
  /** light userdata */
  TM_OUT_REF,
  /** number of elements + 1 */
  TM_OUT_NUMBER
};

/** Timer object metatable name. */
const char* MT_TIMER = "ROS2.Timer";

/**
 * Create new timer.
 *
 * Arguments:
 * - clock object.
 * - period, sec (float)
 * - callback function fn(nil) -> nil
 *
 * Return:
 * - timer object.
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_timer_init (lua_State* L)
{
  /* arg1 - clock */
  rcl_clock_t* clock = luaL_checkudata(L, 1, MT_CLOCK);

  /* arg2 - period */
  lua_Number sec = luaL_checknumber(L, 2);
  luaL_argcheck(L, sec >= 0, 2, "negative period");
  rcl_time_point_value_t nsec = sec * 1E9;

  /* arg3 - callback */
  luaL_checktype(L, 3, LUA_TFUNCTION);

  /* init timer */
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_context_t* context = rcl_lua_context_ref();
  rcl_timer_t* timer = lua_newuserdata(L, sizeof(rcl_timer_t));  // push timer
  *timer = rcl_get_zero_initialized_timer();

  rcl_ret_t ret = rcl_timer_init(
    timer, clock, context, nsec, NULL, allocator);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to create timer");
  }

  /* set metamethods */
  luaL_getmetatable(L, MT_TIMER);  // push metatable
  lua_setmetatable(L, -2);         // pop metatable

  /* save callback */
  lua_pushvalue(L, 3);             // push callback function
  lua_rawsetp(L, LUA_REGISTRYINDEX, timer);  // reg[timer] = callback function

  return 1;
}

/**
 * Timer destructor.
 *
 * Arguments:
 * - timer object.
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_timer_free (lua_State* L)
{
  /* arg1 - timer object */
  rcl_timer_t *timer = lua_touserdata(L, 1);

  rcl_ret_t ret = rcl_timer_fini(timer);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to fini timer: %s", rcl_get_error_string().str);
    rcl_reset_error();
  }

  return 0;
}

/**
 * Check if the timer is ready.
 *
 * Arguments:
 * - timer object.
 *
 * Return:
 * - true when ready.
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_timer_is_ready (lua_State* L)
{
  /* arg1 - timer object */
  rcl_timer_t* timer = luaL_checkudata(L, 1, MT_TIMER);

  return rcl_lua_timer_push_ready(L, timer);
}

/**
 * Check if the timer is ready using pointer.
 *
 * Arguments:
 * - timer pointer (light userdata)
 *
 * Return:
 * - true when ready.
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_timer_is_ready_ptr (lua_State* L)
{
  /* arg1 - light userdata */
  const rcl_timer_t* timer = lua_topointer(L, 1);

  return rcl_lua_timer_push_ready(L, timer);
}

/**
 * Call timer.
 *
 * Arguments:
 * - timer object
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_timer_call (lua_State* L)
{
  /* arg1 - timer object */
  rcl_timer_t* timer = luaL_checkudata(L, 1, MT_TIMER);

  rcl_ret_t ret = rcl_timer_call(timer);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to call timer");
  }

  return 0;
}

/**
 * Get time until the next call, in seconds.
 *
 * Arguments:
 * - timer object
 *
 * Return:
 * - rest time (seconds, float)
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_timer_time_until_next_call (lua_State* L)
{
  /* arg1 - timer object */
  rcl_timer_t* timer = luaL_checkudata(L, 1, MT_TIMER);

  /* get rest */
  int64_t nsec = 0;
  rcl_ret_t ret = rcl_timer_get_time_until_next_call(timer, &nsec);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to get time");
  }

  lua_pushnumber(L, nsec * 1E-9);  // seconds
  return 1;
}

/**
 * Get time since last call, in seconds.
 *
 * Arguments:
 * - timer object
 *
 * Return:
 * - current time (seconds, float)
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_timer_time_since_last_call (lua_State* L)
{
  /* arg1 - timer object */
  rcl_timer_t* timer = luaL_checkudata(L, 1, MT_TIMER);

  /* get time */
  int64_t nsec = 0;
  rcl_ret_t ret = rcl_timer_get_time_since_last_call(timer, &nsec);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to get time");
  }

  lua_pushnumber(L, nsec * 1E-9);  // seconds
  return 1;
}

/**
 * Get timer period, in seconds.
 *
 * Arguments:
 * - timer object
 *
 * Return:
 * - period (seconds, float)
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_timer_get_period (lua_State* L)
{
  /* arg1 - timer object */
  rcl_timer_t* timer = luaL_checkudata(L, 1, MT_TIMER);

  /* get period */
  int64_t nsec = 0;
  rcl_ret_t ret = rcl_timer_get_period(timer, &nsec);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to get period");
  }

  lua_pushnumber(L, nsec * 1E-9);  // seconds
  return 1;
}

/**
 * Change timer period.
 *
 * Arguments:
 * - timer object
 * - new period, seconds
 *
 * Return:
 * - old period, seconds
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_timer_change_period (lua_State* L)
{
  /* arg1 - timer */
  rcl_timer_t* timer = luaL_checkudata(L, 1, MT_TIMER);
  /* arg2 - new period */
  lua_Number sec = luaL_checknumber(L, 2);

  int64_t old_period;
  rcl_ret_t ret = rcl_timer_exchange_period(timer, (int64_t) (sec * 1E9), &old_period);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to exchange period");
  }

  lua_pushnumber(L, old_period * 1E-9);
  return 1;
}

/**
 * Reset timer state.
 *
 * Arguments:
 * - timer object
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_timer_reset (lua_State* L)
{
  /* arg1 - timer object */
  rcl_timer_t* timer = luaL_checkudata(L, 1, MT_TIMER);

  /* reset state */
  rcl_ret_t ret = rcl_timer_reset(timer);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to reset timer");
  }

  return 0;
}

/**
 * Cancel timer.
 *
 * Arguments:
 * - timer object
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_timer_cancel (lua_State* L)
{
  /* arg1 - timer object */
  rcl_timer_t* timer = luaL_checkudata(L, 1, MT_TIMER);

  /* cancel */
  rcl_ret_t ret = rcl_timer_cancel(timer);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to cancel timer");
  }

  return 0;
}

/**
 * Check timer status.
 *
 * Arguments:
 * - timer object
 *
 * Return:
 * - true if the timer is canceled
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_timer_is_canceled (lua_State* L)
{
  /* arg1 - timer object */
  rcl_timer_t* timer = luaL_checkudata(L, 1, MT_TIMER);

  /* check status */
  bool is_canceled = false;
  rcl_ret_t ret = rcl_timer_is_canceled(timer, &is_canceled);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to check if timer is canceled");
  }

  lua_pushboolean(L, is_canceled);
  return 1;
}

/** List of timer methods */
static const struct luaL_Reg timer_methods[] = {
  {"is_ready", rcl_lua_timer_is_ready},
  {"call", rcl_lua_timer_call},
  {"time_until_next_call", rcl_lua_timer_time_until_next_call},
  {"time_since_last_call", rcl_lua_timer_time_since_last_call},
  {"period", rcl_lua_timer_get_period},
  {"set_period", rcl_lua_timer_change_period},
  {"reset", rcl_lua_timer_reset},
  {"cancel", rcl_lua_timer_cancel},
  {"is_canceled", rcl_lua_timer_is_canceled},
  {"__gc", rcl_lua_timer_free},
  {NULL, NULL}
};

/* Add to library */
void rcl_lua_add_timer_methods (lua_State* L)
{
  /* timer constructor */
  lua_pushcfunction(L, rcl_lua_timer_init);  // push function
  lua_setfield(L, -2, "new_timer");          // pop, lib['new_timer'] = function

  lua_pushcfunction(L, rcl_lua_timer_is_ready_ptr);  // push function
  lua_setfield(L, -2, "is_timer_ready");     // pop, lib['is_timer_ready'] = fn

  /* metamethods */
  rcl_lua_utils_add_mt(L, MT_TIMER, timer_methods);
}

/**
 * Check if the timer is ready, common part. Push result to the stack.
 *
 * \param[inout] L Lua stack.
 * \param[in] timer Timer object.
 * \return number of outputs.
 */
static int rcl_lua_timer_push_ready (lua_State* L, const rcl_timer_t* timer)
{
  /* check status */
  bool ready = false;
  rcl_ret_t ret = rcl_timer_is_ready(timer, &ready);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to check timer ready");
  }

  lua_pushboolean(L, ready);
  return 1;
}

/* Return table {callback, ref}. */
void rcl_lua_timer_push_callback (lua_State* L, const rcl_timer_t* timer)
{
  /* save result into table */
  lua_createtable(L, TM_OUT_NUMBER-1, 0);  // push table a

  lua_rawgetp(L, LUA_REGISTRYINDEX, timer);  // push function
  if (lua_isnil(L, -1)) {
    luaL_error(L, "timer bindings not found");
  }
  lua_rawseti(L, -2, TM_OUT_CALLBACK);     // pop, a[.] = callback

  lua_pushlightuserdata(L, (void*) timer);         // push reference
  lua_rawseti(L, -2, TM_OUT_REF);          // pop, a[.] = reference
  /* keep table 'a' on stack */
}
