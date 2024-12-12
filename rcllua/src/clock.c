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

#include <rcl/time.h>
#include <rcl/allocator.h>
#include <rcl/error_handling.h>

#include "clock.h"
#include "time.h"
#include "utils.h"

/** Clock object metatable name. */
const char* MT_CLOCK = "ROS2.Clock";

/**
 * Create clock object.
 *
 * Arguments:
 * - clock tipe (int, optional)
 *
 * Return:
 * - clock object.
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_clock_init (lua_State* L)
{
  /* arg1 - clock type */
  int tp = luaL_optinteger(L, 1, RCL_SYSTEM_TIME);
  luaL_argcheck(
    L, RCL_CLOCK_UNINITIALIZED <= tp && tp <= RCL_STEADY_TIME, 1, "wrong clock type");

  /* make */
  rcl_clock_t* clock = lua_newuserdata(L, sizeof(rcl_clock_t));  // push object
  rcl_allocator_t allocator = rcl_get_default_allocator();

  rcl_ret_t ret = rcl_clock_init(tp, clock, &allocator);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to initialize clock");
  }

  /* set metamethods */
  luaL_getmetatable(L, MT_CLOCK);  // push metatable
  lua_setmetatable(L, -2);         // pop metatable

  return 1;
}

/**
 * Clock destructor.
 *
 * Arguments:
 * - clock object.
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_clock_free (lua_State* L)
{
  /* arg1 - clock object */
  rcl_clock_t* clock = lua_touserdata(L, 1);

  rcl_ret_t ret = rcl_clock_fini(clock);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to fini clock: %s", rcl_get_error_string().str);
    rcl_reset_error();
  }

  return 0;
}

/**
 * Get current time.
 *
 * Arguments:
 * - clock object.
 *
 * Return:
 * - time object.
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_clock_get_now (lua_State* L)
{
  /* arg1 - clock */
  rcl_clock_t* clock = luaL_checkudata(L, 1, MT_CLOCK);

  /* get time */
  rcl_time_point_value_t time_ns;
  rcl_ret_t ret = rcl_clock_get_now(clock, &time_ns);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to get clock value");
  }

  rcl_lua_time_push_time(L, time_ns, clock->type);
  return 1;
}

/**
 * Get clock type.
 *
 * Arguments:
 * - clock object
 *
 * Return:
 * - type value (int)
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_clock_get_type (lua_State* L)
{
  /* arg1 - clock */
  rcl_clock_t* clock = luaL_checkudata(L, 1, MT_CLOCK);

  lua_pushinteger(L, clock->type);
  return 1;
}

/** List of clock types. */
static const rcl_lua_enum enum_clock_types[] = {
  {"UNINITIALIZED", RCL_CLOCK_UNINITIALIZED},
  {"ROS_TIME", RCL_ROS_TIME},
  {"SYSTEM_TIME", RCL_SYSTEM_TIME},
  {"STEADY_TIME", RCL_STEADY_TIME},
  {NULL, -1}
};

/** List of clock methods. */
static const struct luaL_Reg clock_methods[] = {
  {"now", rcl_lua_clock_get_now},
  {"clock_type", rcl_lua_clock_get_type},
  {"__gc", rcl_lua_clock_free},
  {NULL, NULL}
};

/* Add to library */
void rcl_lua_add_clock_methods (lua_State* L)
{
  /* clock constructor */
  lua_pushcfunction(L, rcl_lua_clock_init);   // push function
  lua_setfield(L, -2, "new_clock");           // pop, lib['new_clock'] = function

  /* metamethods */
  rcl_lua_utils_add_mt(L, MT_CLOCK, clock_methods);

  /* clock types */
  rcl_lua_utils_add_enum(L, "ClockType", enum_clock_types);  // lib['ClockType'] = {...}
}
