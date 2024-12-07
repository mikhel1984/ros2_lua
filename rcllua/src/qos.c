
#include <string.h>

#include <lauxlib.h>

#include <rmw/types.h>
#include <rmw/qos_profiles.h>
#include <rcl/time.h>

#include "time.h"
#include "qos.h"
#include "utils.h"

const char* MT_QOS = "ROS2.QoS";

static void duration_to_rmw_time (const rcl_duration_t* dur, rmw_time_t* tm)
{
  tm->sec = RCL_NS_TO_S(dur->nanoseconds);
  tm->nsec = dur->nanoseconds % NSEC_IN_SEC;
}

static int rcl_lua_qos_init (lua_State* L)
{
  const char* profile = luaL_optstring(L, 1, "qos_profile_default");
  rmw_qos_profile_t *qos = lua_newuserdata(L, sizeof(rmw_qos_profile_t));

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

  luaL_getmetatable(L, MT_QOS);
  lua_setmetatable(L, -2);

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

// TODO __index, __newindex, remove __call

static const struct luaL_Reg qos_methods[] = {
  {"__call", rcl_lua_qos_update},
  {NULL, NULL}
};

void rcl_lua_add_qos_methods (lua_State* L)
{
  /* constructor */
  lua_pushcfunction(L, rcl_lua_qos_init);
  lua_setfield(L, -2, "qos_init");

  /* metamethods */
  rcl_lua_utils_add_mt(L, MT_QOS, qos_methods);
}

