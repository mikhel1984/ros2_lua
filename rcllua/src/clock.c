
#include <lauxlib.h>

#include <rcl/time.h>
#include <rcl/allocator.h>
#include <rcl/error_handling.h>

#include "clock.h"
#include "time.h"
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
    luaL_error(L, "failed to fini clocl: %s", rcl_get_error_string().str);
    rcl_reset_error();
  }

  return 0;
}

static int rcl_lua_clock_get_now (lua_State* L)
{
  rcl_clock_t* clock = luaL_checkudata(L, 1, MT_CLOCK);

  rcl_time_point_value_t time_ns;
  rcl_ret_t = ret = rcl_clock_get_now(clock, &time_ns);
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
