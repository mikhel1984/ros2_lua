
#include <lauxlib.h>

#include <rcl/time.h>
#include <rcl/timer.h>
#include <rcl/allocator.h>
#include <rcl/error_handling.h>

#include "clock.h"
#include "time.h"
#include "timer.h"
#include "context.h"
#include "utils.h"


const char* MT_CLOCK = ".ROS2.Clock";

static int rcl_lua_clock_init (lua_State* L)
{
  int tp = luaL_optinteger(L, 1, RCL_SYSTEM_TIME);
  luaL_argcheck(
    L, RCL_CLOCK_UNINITIALIZED <= tp && tp <= RCL_STEADY_TIME, 1, "wrong clock type");

  rcl_clock_t *clock = lua_newuserdata(L, sizeof(rcl_clock_t));

  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rcl_clock_init(tp, clock, &allocator);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to initialize clock");
  }

  // set metatable
  luaL_getmetatable(L, MT_CLOCK);
  lua_setmetatable(L, -2);

  return 1;
}

static int rcl_lua_clock_free (lua_State* L)
{
  rcl_clock_t* clock = lua_touserdata(L, 1);
  rcl_ret_t ret = rcl_clock_fini(clock);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to fini clock: %s", rcl_get_error_string().str);
    rcl_reset_error();
  }

  return 0;
}

static int rcl_lua_clock_get_now (lua_State* L)
{
  rcl_clock_t* clock = luaL_checkudata(L, 1, MT_CLOCK);

  rcl_time_point_value_t time_ns;
  rcl_ret_t ret = rcl_clock_get_now(clock, &time_ns);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to get clock value");
  }

  rcl_time_point_t* ptr = lua_newuserdata(L, sizeof(rcl_time_point_t));
  ptr->clock_type = clock->type;
  ptr->nanoseconds = time_ns;

  luaL_getmetatable(L, MT_TIME);
  lua_setmetatable(L, -2);

  return 1;
}

static int rcl_lua_clock_new_timer (lua_State* L)
{
  /* arg1 - clock */
  rcl_clock_t* clock = luaL_checkudata(L, 1, MT_CLOCK);

  /* arg2 - period */
  lua_Number sec = luaL_checknumber(L, 2);
  luaL_argcheck(L, sec > 0, 2, "negative period");
  rcl_time_point_value_t nsec = sec * 1E9;

  /* arg3 - callback */
  luaL_argcheck(L, lua_type(L, 3) == LUA_TFUNCTION, 3, "wrong timer callback");

  /* init */
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_context_t* context = rcl_lua_context_ref();

  rcl_timer_t* timer = lua_newuserdata(L, sizeof(rcl_timer_t));
  *timer = rcl_get_zero_initialized_timer();

  rcl_ret_t ret = rcl_timer_init(
    timer, clock, context, nsec, NULL, allocator);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to create timer");
  }

  /* set metatable */
  luaL_getmetatable(L, MT_TIMER);
  lua_setmetatable(L, -2);

  /* save callback */
  lua_pushvalue(L, 3);  // duplicate callback function
  lua_rawsetp(L, LUA_REGISTRYINDEX, timer);  // reg[timer] = fn

  return 1;
}

static const rcl_lua_enum enum_clock_types[] = {
  {"UNINITIALIZED", RCL_CLOCK_UNINITIALIZED},
  {"ROS_TIME", RCL_ROS_TIME},
  {"SYSTEM_TIME", RCL_SYSTEM_TIME},
  {"STEADY_TIME", RCL_STEADY_TIME},
  {NULL, -1}
};

static const struct luaL_Reg clock_methods[] = {
  {"now", rcl_lua_clock_get_now},
  {"new_timer", rcl_lua_clock_new_timer},
  {"__gc", rcl_lua_clock_free},
  {NULL, NULL}
};

void rcl_lua_add_clock_methods (lua_State* L)
{
  /* make clock */
  lua_pushcfunction(L, rcl_lua_clock_init);
  lua_setfield(L, -2, "new_clock");

  /* metamethods */
  rcl_lua_utils_add_mt(L, MT_CLOCK, clock_methods);

  /* enum types */
  rcl_lua_utils_add_enum(L, "ClockType", enum_clock_types);
}
