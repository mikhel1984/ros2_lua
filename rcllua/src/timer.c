
#include <lauxlib.h>

#include <rcl/timer.h>
#include <rcl/error_handling.h>

#include "utils.h"

const char* MT_TIMER = "ROS2.Timer";

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

static int rcl_lua_timer_is_ready (lua_State* L)
{
  rcl_timer_t* timer = luaL_checkudata(L, 1, MT_TIMER);

  bool ready = false;
  rcl_ret_t ret = rcl_timer_is_ready(timer, &ready);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to check timer ready");
  }
  lua_pushboolean(L, ready);

  return 1;
}

static int rcl_lua_timer_call (lua_State* L)
{
  rcl_timer_t* timer = luaL_checkudata(L, 1, MT_TIMER);

  rcl_ret_t ret = rcl_timer_call(timer);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to call timer");
  }

  return 0;
}

static int rcl_lua_timer_time_until_next_call (lua_State* L)
{
  rcl_timer_t* timer = luaL_checkudata(L, 1, MT_TIMER);

  int64_t nsec = 0;
  rcl_ret_t ret = rcl_timer_get_time_until_next_call(timer, &nsec);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to get time");
  }
  lua_pushnumber(L, nsec * 1E-9);  // seconds

  return 1;
}

static int rcl_lua_timer_time_since_last_call (lua_State* L)
{
  rcl_timer_t* timer = luaL_checkudata(L, 1, MT_TIMER);

  int64_t nsec = 0;
  rcl_ret_t ret = rcl_timer_get_time_since_last_call(timer, &nsec);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to get time");
  }
  lua_pushnumber(L, nsec * 1E-9);  // seconds

  return 1;
}

static int rcl_lua_timer_get_period (lua_State* L)
{
  rcl_timer_t* timer = luaL_checkudata(L, 1, MT_TIMER);

  int64_t nsec = 0;
  rcl_ret_t ret = rcl_timer_get_period(timer, &nsec);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to get period");
  }
  lua_pushnumber(L, nsec * 1E-9);  // seconds

  return 1;
}

static int rcl_lua_timer_change_period (lua_State* L)
{
  /* arg1 - timer */
  rcl_timer_t* timer = luaL_checkudata(L, 1, MT_TIMER);
  /* arg2 - new period */
  lua_Number sec = luaL_checknumber(L, 2);

  int64_t old_period;
  rcl_ret_t ret = rcl_timer_exchange_period(timer, (int64_t) (sec * 1E9), &old_period);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to exchange period");
  }
  
  return 0;
}

static int rcl_lua_timer_reset (lua_State* L)
{
  rcl_timer_t* timer = luaL_checkudata(L, 1, MT_TIMER);

  rcl_ret_t ret = rcl_timer_reset(timer);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to reset timer");
  }

  return 0;
}

static int rcl_lua_timer_cancel (lua_State* L)
{
  rcl_timer_t* timer = luaL_checkudata(L, 1, MT_TIMER);

  rcl_ret_t ret = rcl_timer_cancel(timer);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to cancel timer");
  }

  return 0;
}

static int rcl_lua_timer_is_canceled (lua_State* L)
{
  rcl_timer_t* timer = luaL_checkudata(L, 1, MT_TIMER);

  bool is_canceled = false;
  rcl_ret_t ret = rcl_timer_is_canceled(timer, &is_canceled);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to check if timer is canceled");
  }
  lua_pushboolean(L, is_canceled);

  return 1;
}

static const struct luaL_Reg timer_methods[] = {
  {"is_ready", rcl_lua_timer_is_ready},
  {"call", rcl_lua_timer_call},
  {"time_until_next_call", rcl_lua_timer_time_until_next_call},
  {"time_since_last_call", rcl_lua_timer_time_since_last_call},
  {"period", rcl_lua_timer_get_period},
  {"set_period", rcl_lua_timer_change_period},
  {"reset", rcl_lua_timer_reset},
  {"cancel", rcl_lua_timer_cancel},
  {"is_canceled", rcl_lua_timer_is_canceled},
  {"__gc", rcl_lua_timer_free},
  {NULL, NULL}
};

void rcl_lua_add_timer_methods (lua_State* L)
{
  // metamethods
  rcl_lua_utils_add_mt(L, MT_TIMER, timer_methods);
}
