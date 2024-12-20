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
#include <lauxlib.h>

#include <rmw/types.h>
#include <rmw/qos_profiles.h>
#include <rcl/time.h>

#include "rcllua/time.h"
#include "rcllua/qos.h"
#include "rcllua/utils.h"

/** Collect structure fields for access. */
enum QoSFields {
  QOS_F_HISTORY,
  QOS_F_DEPTH,
  QOS_F_RELIABILITY,
  QOS_F_DURABILITY,
  QOS_F_LIVELINESS,
  QOS_F_AVOID_ROS,
  QOS_F_LIFESPAN,
  QOS_F_DEADLINE,
  QOS_F_LIVELINESS_LEASE,
  QOS_F_NUMBER
};

/** QoS object metatable name. */
const char* MT_QOS = "ROS2.QoS";

/**
 * Convert rcl duration into rmw datatype.
 *
 * \param[in] dur rcl duration
 * \param[out] tm rmw duration
 */
static void duration_to_rmw_time (const rcl_duration_t* dur, rmw_time_t* tm)
{
  tm->sec = RCL_NS_TO_S(dur->nanoseconds);
  tm->nsec = dur->nanoseconds % NSEC_IN_SEC;
}

/**
 * Create QoS object.
 *
 * Arguments:
 * - QoS tipe (string, optional)
 *
 * Return:
 * - QoS object.
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_qos_init (lua_State* L)
{
  /* arg1 - type name */
  const char* profile = luaL_optstring(L, 1, "qos_profile_default");
  rmw_qos_profile_t *qos = lua_newuserdata(L, sizeof(rmw_qos_profile_t));  // push object

  /* choose settings */
  if (0 == strcmp(profile, "qos_profile_default")) {
    *qos = rmw_qos_profile_default;
  } else if (0 == strcmp(profile, "qos_profile_sensor_data")) {
    *qos = rmw_qos_profile_sensor_data;
  } else if (0 == strcmp(profile, "qos_profile_system_default")) {
    *qos = rmw_qos_profile_system_default;
  } else if (0 == strcmp(profile, "qos_profile_service_default")) {
    *qos = rmw_qos_profile_services_default;
  } else if (0 == strcmp(profile, "qos_profile_unknown")) {
    *qos = rmw_qos_profile_unknown;
  } else if (0 == strcmp(profile, "qos_profile_parameters")) {
    *qos = rmw_qos_profile_parameters;
  } else if (0 == strcmp(profile, "qos_profile_parameter_events")) {
    *qos = rmw_qos_profile_parameter_events;
  } else {
    luaL_error(L, "unexpected QoS profile %s", profile);
  }

  /* add metamethods */
  luaL_getmetatable(L, MT_QOS);  // push metatable
  lua_setmetatable(L, -2);       // pop metatable

  return 1;
}

/**
 * Prepare hash table for quick access to QoS fields.
 *
 * \param[inout] L Lua stack.
 */
static void rcl_lua_qos_set_types (lua_State* L)
{
  /* set to metatable */
  luaL_getmetatable(L, MT_QOS);            // push metatable

  /* add container and fill it */
  lua_createtable(L, 0, QOS_F_NUMBER);     // push table a
  lua_pushinteger(L, QOS_F_HISTORY);       // push
  lua_setfield(L, -2, "history");          // pop and set
  lua_pushinteger(L, QOS_F_DEPTH);         // push
  lua_setfield(L, -2, "depth");            // pop and set
  lua_pushinteger(L, QOS_F_RELIABILITY);   // push
  lua_setfield(L, -2, "reliability");      // pop and set
  lua_pushinteger(L, QOS_F_DURABILITY);    // push
  lua_setfield(L, -2, "durability");       // pop and set
  lua_pushinteger(L, QOS_F_LIVELINESS);    // push
  lua_setfield(L, -2, "liveliness");       // pop and set
  lua_pushinteger(L, QOS_F_AVOID_ROS);     // push
  lua_setfield(L, -2, "avoid_ros_namespace_conventions");  // pop and set
  lua_pushinteger(L, QOS_F_LIFESPAN);      // push
  lua_setfield(L, -2, "lifespan");         // pop and set
  lua_pushinteger(L, QOS_F_DEADLINE);      // push
  lua_setfield(L, -2, "deadline");         // pop and set
  lua_pushinteger(L, QOS_F_LIVELINESS_LEASE);        // push
  lua_setfield(L, -2, "liveliness_lease_duration");  // pop and set

  /* save and clear */
  lua_setfield(L, -2, "_fields");          // pop, metatalbe['_fields'] = a
  lua_pop(L, 1);                           // pop metatable
}

/**
 * Translate value to ROS duration, put to stack.
 *
 * \param[inout] L Lua stack.
 * \param[in] tm rmw duration.
 */
static void rcl_lua_qos_push_duration (lua_State* L, const rmw_time_t* tm)
{
  rcl_duration_t* dur = lua_newuserdata(L, sizeof(rcl_duration_t));
  dur->nanoseconds = tm->nsec;
  dur->nanoseconds += tm->sec * NSEC_IN_SEC;

  luaL_getmetatable(L, MT_DURATION);
  lua_setmetatable(L, -2);
}

/**
 * Read QoS field value by name.
 *
 * Arguments:
 * - QoS object
 * - field name
 *
 * Return:
 * - field value
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_qos_index (lua_State* L)
{
  /* arg1 - qos object, find index */
  luaL_getmetafield(L, 1, "_fields");                // push _fields
  /* arg2 - field name */
  const char* field = luaL_checkstring(L, 2);
  if (LUA_TNUMBER != lua_getfield(L, -1, field)) {   // push int
    luaL_error(L, "%s not found", field);
  }
  int id = lua_tointeger(L, -1);
  lua_pop(L, 2);                                     // pop int and _fields

  /* get value */
  rmw_qos_profile_t* qos = lua_touserdata(L, 1);
  switch (id) {
    case QOS_F_HISTORY:
      lua_pushinteger(L, qos->history);
      break;
    case QOS_F_DEPTH:
      lua_pushinteger(L, qos->depth);
      break;
    case QOS_F_RELIABILITY:
      lua_pushinteger(L, qos->reliability);
      break;
    case QOS_F_DURABILITY:
      lua_pushinteger(L, qos->durability);
      break;
    case QOS_F_LIVELINESS:
      lua_pushinteger(L, qos->liveliness);
      break;
    case QOS_F_AVOID_ROS:
      lua_pushboolean(L, qos->avoid_ros_namespace_conventions);
      break;
    case QOS_F_LIFESPAN:
      rcl_lua_qos_push_duration(L, &(qos->lifespan));
      break;
    case QOS_F_DEADLINE:
      rcl_lua_qos_push_duration(L, &(qos->deadline));
      break;
    case QOS_F_LIVELINESS_LEASE:
      rcl_lua_qos_push_duration(L, &(qos->liveliness_lease_duration));
      break;
    default:
      luaL_error(L, "unexpected error");
  }

  return 1;
}

/**
 * Write QoS field value by name.
 *
 * Arguments:
 * - QoS object
 * - field name
 * - field value
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_qos_newindex (lua_State* L)
{
  /* arg1 - qos object, find index */
  luaL_getmetafield(L, 1, "_fields");                // push _fields
  /* arg2 - field name */
  const char* field = luaL_checkstring(L, 2);
  if (LUA_TNUMBER != lua_getfield(L, -1, field)) {   // push int
    luaL_error(L, "%s not found", field);
  }
  int id = lua_tointeger(L, -1);
  lua_pop(L, 2);                                     // pop int and _fields

  /* arg3 - value, set */
  rmw_qos_profile_t* qos = lua_touserdata(L, 1);
  rcl_duration_t* dur = NULL;
  switch (id) {
    case QOS_F_HISTORY:
      qos->history = (rmw_qos_history_policy_t) luaL_checkinteger(L, 3);
      break;
    case QOS_F_DEPTH:
      qos->depth = (int) luaL_checkinteger(L, 3);
      break;
    case QOS_F_RELIABILITY:
      qos->reliability = (rmw_qos_reliability_policy_t) luaL_checkinteger(L, 3);
      break;
    case QOS_F_DURABILITY:
      qos->durability = (rmw_qos_durability_policy_t) luaL_checkinteger(L, 3);
      break;
    case QOS_F_LIVELINESS:
      qos->liveliness = (rmw_qos_liveliness_policy_t) luaL_checkinteger(L, 3);
      break;
    case QOS_F_AVOID_ROS:
      qos->avoid_ros_namespace_conventions = lua_toboolean(L, 3);
      break;
    case QOS_F_LIFESPAN:
      dur = luaL_checkudata(L, 3, MT_DURATION);
      duration_to_rmw_time(dur, &(qos->lifespan));
      break;
    case QOS_F_DEADLINE:
      dur = luaL_checkudata(L, 3, MT_DURATION);
      duration_to_rmw_time(dur, &(qos->deadline));
      break;
    case QOS_F_LIVELINESS_LEASE:
      dur = luaL_checkudata(L, -1, MT_DURATION);
      duration_to_rmw_time(dur, &(qos->liveliness_lease_duration));
      break;
    default:
      luaL_error(L, "unexpected error");
  }

  return 1;
}

/** List of QoS methods */
static const struct luaL_Reg qos_methods[] = {
  {"__index", rcl_lua_qos_index},
  {"__newindex", rcl_lua_qos_newindex},
  {NULL, NULL}
};

/* Add to library */
void rcl_lua_add_qos_methods (lua_State* L)
{
  /* constructor */
  lua_pushcfunction(L, rcl_lua_qos_init);   // push function
  lua_setfield(L, -2, "new_qos");           // pop, lib['new_qos'] = function

  /* metamethods */
  rcl_lua_utils_add_mt(L, MT_QOS, qos_methods);

  /* fill _fields table */
  rcl_lua_qos_set_types(L);
}

/* Make copy of the gimen QoS object. */
void rcl_lua_qos_push_copy (lua_State* L, const rmw_qos_profile_t* src)
{
  if (NULL == src) {
    luaL_error(L, "no QoS");
  }

  /* make copy */
  rmw_qos_profile_t *qos = lua_newuserdata(L, sizeof(rmw_qos_profile_t));  // push object
  *qos = *src;

  /* add metamethods */
  luaL_getmetatable(L, MT_QOS);  // push metatable
  lua_setmetatable(L, -2);       // pop metatable
}
