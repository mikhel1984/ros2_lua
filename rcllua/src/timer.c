
#include <lauxlib.h>

#include <rcl/timer.h>
#include <rcl/error_handling.h>


const char* MT_TIMER = ".ROS2.Timer";

static int rcl_lua_timer_free (lua_State* L)
{
  rcl_timer_t *timer = lua_touserdata(L, 1);

  rcl_ret_t ret = rcl_timer_fini(timer);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to fini timer: %s", rcl_get_error_string().str);
    rcl_reset_error();
  }

  return 0;
}

static const struct luaL_Reg timer_methods[] = {
  {"__gc", rcl_lua_timer_free},
  {NULL, NULL}
};

void rcl_lua_add_timer_methods (lua_State* L)
{
  // metamethods
  rcl_lua_utils_add_mt(L, MT_TIMER, timer_methods);
}
