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

#include <string.h>
#include <stdint.h>
#include <lauxlib.h>

#include <rcl/time.h>

#include "rcllua/time.h"
#include "rcllua/utils.h"

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
  rcl_lua_time_push_time(L, val, tp);

  return 1;
}

/**
 * Create duration object.
 *
 * Arguments:
 * - seconds (int, optional)
 * - nanoseconds (int, optional)
 *
 * Return:
 * - duration object
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_time_init_dur (lua_State* L)
{
  /* arg1 - seconds */
  lua_Integer sec = luaL_optinteger(L, 1, 0);
  /* arg2 - nanoseconds */
  lua_Integer nsec = luaL_optinteger(L, 2, 0);

  /* check overflow */
  double sum = sec*1E9 + nsec;
  if (sum > INT64_MAX || sum < INT64_MIN) {
    luaL_error(L, "out of range");
  }

  rcl_duration_value_t val = nsec;
  val += sec*NSEC_IN_SEC;
  rcl_lua_time_push_duration(L, val);

  return 1;
}

/**
 * Get time field value (sec, nsec, clock).
 *
 * Arguments:
 * - time object
 * - field name
 *
 * Return:
 * - field value
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_time_index (lua_State* L)
{
  /* arg1 - time object */
  rcl_time_point_t* time = luaL_checkudata(L, 1, MT_TIME);
  /* arg2 - field name */
  const char* field = luaL_checkstring(L, 2);

  /* get */
  if (0 == strcmp(field, "sec")) {
    lua_pushinteger(L, time->nanoseconds / NSEC_IN_SEC);
  } else if (0 == strcmp(field, "nsec")) {
    lua_pushinteger(L, time->nanoseconds % NSEC_IN_SEC);
  } else if (0 == strcmp(field, "clock_type")) {
    lua_pushinteger(L, time->clock_type);
  } else if (LUA_TNIL == luaL_getmetafield(L, 1, field)) {
    luaL_error(L, "unknown field '%s'", field);
  }

  return 1;
}

/**
 * Get duration field value (sec, nsec).
 *
 * Arguments:
 * - duration object
 * - field name
 *
 * Return:
 * - field value
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_time_index_dur (lua_State* L)
{
  /* arg1 - time object */
  rcl_duration_t* dur = luaL_checkudata(L, 1, MT_DURATION);
  /* arg2 - field name */
  const char* field = luaL_checkstring(L, 2);

  /* get */
  if (0 == strcmp(field, "sec")) {
    lua_pushinteger(L, dur->nanoseconds / NSEC_IN_SEC);
  } else if (0 == strcmp(field, "nsec")) {
    lua_pushinteger(L, dur->nanoseconds % NSEC_IN_SEC);
  } else if (LUA_TNIL == luaL_getmetafield(L, 1, field)) {
    luaL_error(L, "unknown field '%s'", field);
  }

  return 1;
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

/**
 * Get duration in seconds.
 *
 * Arguments:
 * - duration object
 *
 * Return:
 * - seconds (float)
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_time_seconds_dur (lua_State* L)
{
  /* arg1 - time object */
  rcl_duration_t* dur = luaL_checkudata(L, 1, MT_DURATION);

  lua_pushnumber(L, dur->nanoseconds * 1E-9);  // push seconds
  return 1;
}

/**
 * Check t1 == t2. Clock type must be equal.
 *
 * Arguments:
 * - time 1
 * - time 2
 *
 * Return:
 * - t1 == t2
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_time_eq (lua_State* L)
{
  /* arg1 - first time */
  rcl_time_point_t* t1 = luaL_checkudata(L, 1, MT_TIME);
  /* arg2 - second time */
  rcl_time_point_t* t2 = luaL_checkudata(L, 2, MT_TIME);

  if (t1->clock_type != t2->clock_type) {
    luaL_error(L, "different clock type");
  }

  lua_pushboolean(L, t1->nanoseconds == t2->nanoseconds);
  return 1;
}

/**
 * Check t1 < t2. Clock type must be equal.
 *
 * Arguments:
 * - time 1
 * - time 2
 *
 * Return:
 * - t1 < t2
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_time_lt (lua_State* L)
{
  /* arg1 - first time */
  rcl_time_point_t* t1 = luaL_checkudata(L, 1, MT_TIME);
  /* arg2 - second time */
  rcl_time_point_t* t2 = luaL_checkudata(L, 2, MT_TIME);

  if (t1->clock_type != t2->clock_type) {
    luaL_error(L, "different clock type");
  }

  lua_pushboolean(L, t1->nanoseconds < t2->nanoseconds);
  return 1;
}

/**
 * Check t1 <= t2. Clock type must be equal.
 *
 * Arguments:
 * - time 1
 * - time 2
 *
 * Return:
 * - t1 <= t2
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_time_le (lua_State* L)
{
  /* arg1 - first time */
  rcl_time_point_t* t1 = luaL_checkudata(L, 1, MT_TIME);
  /* arg2 - second time */
  rcl_time_point_t* t2 = luaL_checkudata(L, 2, MT_TIME);

  if (t1->clock_type != t2->clock_type) {
    luaL_error(L, "different clock type");
  }

  lua_pushboolean(L, t1->nanoseconds <= t2->nanoseconds);
  return 1;
}

/**
 * Check d1 == d2.
 *
 * Arguments:
 * - duration 1
 * - duration 2
 *
 * Return:
 * - d1 == d2
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_time_eq_dur (lua_State* L)
{
  /* arg1 - first time */
  rcl_duration_t* d1 = luaL_checkudata(L, 1, MT_DURATION);
  /* arg2 - second time */
  rcl_duration_t* d2 = luaL_checkudata(L, 2, MT_DURATION);

  lua_pushboolean(L, d1->nanoseconds == d2->nanoseconds);
  return 1;
}

/**
 * Check d1 < d2.
 *
 * Arguments:
 * - duration 1
 * - duration 2
 *
 * Return:
 * - d1 < d2
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_time_lt_dur (lua_State* L)
{
  /* arg1 - first time */
  rcl_duration_t* d1 = luaL_checkudata(L, 1, MT_DURATION);
  /* arg2 - second time */
  rcl_duration_t* d2 = luaL_checkudata(L, 2, MT_DURATION);

  lua_pushboolean(L, d1->nanoseconds < d2->nanoseconds);
  return 1;
}

/**
 * Check d1 <= d2.
 *
 * Arguments:
 * - duration 1
 * - duration 2
 *
 * Return:
 * - d1 <= d2
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_time_le_dur (lua_State* L)
{
  /* arg1 - first time */
  rcl_duration_t* d1 = luaL_checkudata(L, 1, MT_DURATION);
  /* arg2 - second time */
  rcl_duration_t* d2 = luaL_checkudata(L, 2, MT_DURATION);

  lua_pushboolean(L, d1->nanoseconds <= d2->nanoseconds);
  return 1;
}

/**
 * Increase time to specific duration value.
 *
 * Arguments:
 * - time object
 * - duration object
 *
 * Return:
 * - new time object
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_time_add (lua_State* L)
{
  /* arg1 - time object */
  rcl_time_point_t* t = luaL_checkudata(L, 1, MT_TIME);
  /* arg2 - duration object */
  rcl_duration_t* d = luaL_checkudata(L, 2, MT_DURATION);

  /* check result */
  double sum = t->nanoseconds + (double) d->nanoseconds;
  if (sum < 0 || sum > 1E64) {
    luaL_error(L, "sum is out of bounds");
  }

  /* init */
  rcl_lua_time_push_time( L, t->nanoseconds + d->nanoseconds, t->clock_type);
  return 1;
}

/**
 * Decrease time to specific duration value.
 *
 * Arguments:
 * - first time
 * - second time
 *
 * Return:
 * - new duration object
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_time_sub (lua_State* L)
{
  /* arg1 - time object */
  rcl_time_point_t* t1 = luaL_checkudata(L, 1, MT_TIME);
  /* arg2 - time object */
  rcl_time_point_t* t2 = luaL_checkudata(L, 2, MT_TIME);

  /* init */
  rcl_lua_time_push_duration(L, t1->nanoseconds - t2->nanoseconds);
  return 1;
}

static int rcl_lua_time_unm_dur (lua_State* L)
{
  /* arg1 - duration object */
  rcl_duration_t* d = lua_touserdata(L, 1);

  /* init */
  rcl_lua_time_push_duration(L, - d->nanoseconds);
  return 1;
}

/** List of time methods */
static const struct luaL_Reg time_methods[] = {
  {"__index", rcl_lua_time_index},
  {"__eq", rcl_lua_time_eq},
  {"__lt", rcl_lua_time_lt},
  {"__le", rcl_lua_time_le},
  {"__add", rcl_lua_time_add},
  {"__sub", rcl_lua_time_sub},
  {"seconds", rcl_lua_time_seconds},
  {NULL, NULL}
};

/** List of duration methods. */
static const struct luaL_Reg duration_methods[] = {
  {"__index", rcl_lua_time_index_dur},
  {"__eq", rcl_lua_time_eq_dur},
  {"__lt", rcl_lua_time_lt_dur},
  {"__le", rcl_lua_time_le_dur},
  {"__unm", rcl_lua_time_unm_dur},
  {"seconds", rcl_lua_time_seconds_dur},
  {NULL, NULL}
};

/* Add to library */
void rcl_lua_add_time_methods (lua_State* L)
{
  /* time constructor */
  lua_pushcfunction(L, rcl_lua_time_init);  // push function
  lua_setfield(L, -2, "new_time");          // pop, lib['new_time'] = function

  /* duration constructor */
  lua_pushcfunction(L, rcl_lua_time_init_dur);  // push function
  lua_setfield(L, -2, "new_duration");          // pop, lib['new_duration'] = function

  /* time metamethods */
  rcl_lua_utils_add_mt(L, MT_TIME, time_methods);

  /* duration metamethods */
  rcl_lua_utils_add_mt(L, MT_DURATION, duration_methods);
}

/* Create time object, init and push to the stack. */
void rcl_lua_time_push_time (lua_State* L, int64_t ns, int clock_type)
{
  rcl_time_point_t* time = lua_newuserdata(L, sizeof(rcl_time_point_t));  // push object
  time->nanoseconds = ns;
  time->clock_type = clock_type;

  /* set metamethods */
  luaL_getmetatable(L, MT_TIME);  // push metatable
  lua_setmetatable(L, -2);        // pop metatable
}

/* Create duration object, init and push to the stack. */
void rcl_lua_time_push_duration (lua_State* L, int64_t ns)
{
  rcl_duration_t* dur = lua_newuserdata(L, sizeof(rcl_duration_t));  // push object
  dur->nanoseconds = ns;

  /* set metamethods */
  luaL_getmetatable(L, MT_DURATION);  // push metatable
  lua_setmetatable(L, -2);        // pop metatable
}
