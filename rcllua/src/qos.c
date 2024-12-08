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

#include "time.h"
#include "qos.h"
#include "utils.h"

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

static int rcl_lua_qos_update (lua_State* L)
{
  rmw_qos_profile_t *qos = luaL_checkudata(L, 1, MT_QOS);
  luaL_argcheck(L, lua_istable(L, 2), 2, "table expected");

  if (LUA_TNIL != lua_getfield(L, 2, "history")) {
    qos->history = (rmw_qos_history_policy_t) luaL_checkinteger(L, -1);
  }
  lua_pop(L, 1);

  if (LUA_TNIL != lua_getfield(L, 2, "depth")) {
    qos->depth = (int) luaL_checkinteger(L, -1);
  }
  lua_pop(L, 1);

  if (LUA_TNIL != lua_getfield(L, 2, "reliability")) {
    qos->reliability = (rmw_qos_reliability_policy_t) luaL_checkinteger(L, -1);
  }
  lua_pop(L, 1);

  if (LUA_TNIL != lua_getfield(L, 2, "durability")) {
    qos->durability = (rmw_qos_durability_policy_t) luaL_checkinteger(L, -1);
  }
  lua_pop(L, 1);

  if (LUA_TNIL != lua_getfield(L, 2, "liveliness")) {
    qos->liveliness = (rmw_qos_liveliness_policy_t) luaL_checkinteger(L, -1);
  }
  lua_pop(L, 1);

  if (LUA_TNIL != lua_getfield(L, 2, "avoid_ros_namespace_conventions")) {
    if (lua_isboolean(L, -1)) {
      qos->avoid_ros_namespace_conventions = lua_toboolean(L, -1);
    } else {
      luaL_error(L, "avoid_ros_namespace_conventions is boolean");
    }
  }
  lua_pop(L, 1);

  if (LUA_TNIL != lua_getfield(L, 2, "lifespan")) {
    rcl_duration_t* dur = luaL_checkudata(L, -1, MT_DURATION);
    duration_to_rmw_time(dur, &(qos->lifespan));
  }
  lua_pop(L, 1);

  if (LUA_TNIL != lua_getfield(L, 2, "deadline")) {
    rcl_duration_t* dur = luaL_checkudata(L, -1, MT_DURATION);
    duration_to_rmw_time(dur, &(qos->deadline));
  }
  lua_pop(L, 1);

  if (LUA_TNIL != lua_getfield(L, 2, "liveliness_lease_duration")) {
    rcl_duration_t* dur = luaL_checkudata(L, -1, MT_DURATION);
    duration_to_rmw_time(dur, &(qos->liveliness_lease_duration));
  }
  lua_pop(L, 1);

  return 0;
}

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

static void rcl_lua_qos_push_duration (lua_State* L, const rmw_time_t* tm)
{
  rcl_duration_t* dur = lua_newuserdata(L, sizeof(rcl_duration_t));
  dur->nanoseconds = tm->nsec;
  dur->nanoseconds += tm->sec * NSEC_IN_SEC;

  luaL_getmetatable(L, MT_DURATION);
  lua_setmetatable(L, -2);
}

static int rcl_lua_qos_index (lua_State* L)
{
  /* find index */
  luaL_getmetafield(L, 1, "_fields");                // push _fields
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

// TODO __index, __newindex, remove __call

/** List of QoS methods */
static const struct luaL_Reg qos_methods[] = {
  {"__call", rcl_lua_qos_update},
  {NULL, NULL}
};

/* Add to library */
void rcl_lua_add_qos_methods (lua_State* L)
{
  /* constructor */
  lua_pushcfunction(L, rcl_lua_qos_init);
  lua_setfield(L, -2, "new_qos");

  /* metamethods */
  rcl_lua_utils_add_mt(L, MT_QOS, qos_methods);
}

