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

#include <stdint.h>
#include <lauxlib.h>

#include <rcl/time.h>

#include "time.h"
#include "utils.h"

/** Time object metatable name. */
const char* MT_TIME = "ROS2.Time";
/** Duration object metatable name. */
const char* MT_DURATION = "ROS2.Duration";

/**
 * Create time object.
 *
 * Arguments:
 * - seconds (int, optional)
 * - nanoseconds (int, optional)
 * - clock tipe (int, optional)
 *
 * Return:
 * - time object
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_time_init (lua_State* L)
{
  /* arg1 - seconds */
  lua_Integer sec = luaL_optinteger(L, 1, 0);
  /* arg2 - nanoseconds */
  lua_Integer nsec = luaL_optinteger(L, 2, 0);
  /* arg3 - clock type */
  int tp = luaL_optinteger(L, 3, RCL_SYSTEM_TIME);

  /* check arguments and overflow */
  luaL_argcheck(L, sec >= 0, 1, "negative value");
  luaL_argcheck(L, nsec >= 0, 2, "negative value");
  luaL_argcheck(
    L, RCL_CLOCK_UNINITIALIZED <= tp && tp <= RCL_STEADY_TIME, 3, "wrong clock type");
  if ((sec*1E9 + (double)nsec) > UINT64_MAX) {
    luaL_error(L, "too large value of nanoseconds");
  }

  /* init time object */
  rcl_time_point_value_t val = nsec;
  val += sec * NSEC_IN_SEC;
  rcl_time_point_t* time = lua_newuserdata(L, sizeof(rcl_time_point_t));  // push object
  time->nanoseconds = val;
  time->clock_type = tp;

  /* set metamethods */
  luaL_getmetatable(L, MT_TIME);  // push metatable
  lua_setmetatable(L, -2);        // pop metatable

  return 1;
}

/**
 * Get pair (seconds, nanoseconds).
 *
 * Arguments:
 * - time object
 *
 * Return:
 * - seconds (int)
 * - nanoseconds (int)
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_time_get (lua_State* L)
{
  /* arg1 - time object */
  rcl_time_point_t* time = luaL_checkudata(L, 1, MT_TIME);

  /* save */
  lua_pushinteger(L, time->nanoseconds / NSEC_IN_SEC);  // push seconds
  lua_pushinteger(L, time->nanoseconds % NSEC_IN_SEC);  // push nanoseconds

  return 2;
}

/**
 * Get time in seconds.
 *
 * Arguments:
 * - time object
 *
 * Return:
 * - seconds (float)
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_time_seconds (lua_State* L)
{
  /* arg1 - time object */
  rcl_time_point_t* time = luaL_checkudata(L, 1, MT_TIME);
  lua_pushnumber(L, time->nanoseconds * 1E-9);  // push seconds

  return 1;
}

/** List of time methods */
static const struct luaL_Reg time_methods[] = {
  {"get", rcl_lua_time_get},
  {"seconds", rcl_lua_time_seconds},
  {NULL, NULL}
};

/* Add to library */
void rcl_lua_add_time_methods (lua_State* L)
{
  /* time constructor */
  lua_pushcfunction(L, rcl_lua_time_init);  // push function
  lua_setfield(L, -2, "new_time");          // pop, lib['new_time'] = function

  /* metamethods */
  rcl_lua_utils_add_mt(L, MT_TIME, time_methods);
}
