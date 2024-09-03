
#include <lauxlib.h>

#include <rcl/time.h>
#include <rcl/allocator.h>
#include <rcl/error_handling.h>

#include "clock.h"

const char* MT_CLOCK = ".ROS2.Clock";

static int rcl_lua_clock_init (lua_State* L)
{
  rcl_clock_t *clock = lua_newuserdata(L, sizeof(rcl_clock_t));
  rcl_clock_type_t clock_type; // ??

  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rcl_clock_init(clock_type, clock, &allocator);
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
}
