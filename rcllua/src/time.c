
#include <stdint.h>

#include <lauxlib.h>

#include <rcl/time.h>

#include "utils.h"

#define NSEC_IN_SEC 1000000000

const char* MT_TIME = "ROS2.Time";

static int rcl_lua_time_init (lua_State* L)
{
  // first check overflow
  lua_Integer sec = luaL_optinteger(L, 1, 0);
  lua_Integer nsec = luaL_optinteger(L, 2, 0);
  int tp = luaL_optinteger(L, 3, RCL_SYSTEM_TIME);

  luaL_argcheck(L, sec >= 0, 1, "negative second value");
  luaL_argcheck(L, nsec >= 0, 2, "negative nanosecond value");
  luaL_argcheck(
    L, RCL_CLOCK_UNINITIALIZED <= tp && tp <= RCL_STEADY_TIME, 3, "wrong clock type");
  if ((sec*1E9 + (double)nsec) > UINT64_MAX) {
    luaL_error(L, "too large value of nanoseconds");
  }

  rcl_time_point_value_t val = nsec;
  val += sec * NSEC_IN_SEC;

  rcl_time_point_t* time = lua_newuserdata(L, sizeof(rcl_time_point_t));
  time->nanoseconds = val;
  time->clock_type = tp;

  // set metatable
  luaL_getmetatable(L, MT_TIME);
  lua_setmetatable(L, -2);

  return 1;
}

static int rcl_lua_time_get (lua_State* L)
{
  rcl_time_point_t* time = luaL_checkudata(L, 1, MT_TIME);  
  // sec
  lua_pushinteger(L, time->nanoseconds / NSEC_IN_SEC);
  // nsec
  lua_pushinteger(L, time->nanoseconds % NSEC_IN_SEC);

  return 2;
}

static int rcl_lua_time_seconds (lua_State* L)
{
  rcl_time_point_t* time = luaL_checkudata(L, 1, MT_TIME);
  lua_pushnumber(L, time->nanoseconds * 1E-9);

  return 1;
}

static const struct luaL_Reg time_methods[] = {
  {"get", rcl_lua_time_get},
  {"seconds", rcl_lua_time_seconds},
  {NULL, NULL}
};

void rcl_lua_add_time_methods (lua_State* L)
{
  // make time
  lua_pushcfunction(L, rcl_lua_time_init);
  lua_setfield(L, -2, "new_time");

  // metamethods
  rcl_lua_utils_add_mt(L, MT_TIME, time_methods);
}
