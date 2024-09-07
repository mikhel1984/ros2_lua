
#include <lauxlib.h>

#include <rcl/wait.h>
#include <rcl/allocator.h>
#include <rcl/error_handling.h>

#include "wait_set.h"
#include "context.h"
#include "timer.h"


const char* MT_WAIT_SET = ".ROS2.WaitSet";

static int rcl_lua_wait_set_init (lua_State* L)
{
  /* arg1 - #subsctiptions */
  int num_sub = luaL_checkinteger(L, 1);
  /* arg2 - #guard_conditions */
  int num_guard = luaL_checkinteger(L, 2);
  /* arg3 - #timers */
  int num_timers = luaL_checkinteger(L, 3);
  /* arg4 - #clients */
  int num_cli = luaL_checkinteger(L, 4);
  /* arg5 - #services */
  int num_srv = luaL_checkinteger(L, 5);
  /* arg6 - #events */
  int num_ev = luaL_checkinteger(L, 6);

  if (num_sub < 0 || num_guard < 0 || num_timers < 0 || num_cli < 0 
   || num_srv < 0 || num_ev < 0
  ) {
    luaL_error(L, "wrong number of items");
  }

  /* make */
  rcl_wait_set_t* wait_set = lua_newuserdata(L, sizeof(rcl_wait_set_t));
  *wait_set = rcl_get_zero_initialized_wait_set();

  rcl_ret_t ret = rcl_wait_set_init(
    wait_set,
    (size_t) num_sub,
    (size_t) num_guard,
    (size_t) num_timers,
    (size_t) num_cli,
    (size_t) num_srv,
    (size_t) num_ev,
    rcl_lua_context_ref(),
    rcl_get_default_allocator());
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to initialize wait set");
  }

  /* set metatable */
  luaL_getmetatable(L, MT_WAIT_SET);
  lua_setmetatable(L, -2);

  return 1;
}

static int rcl_lua_wait_set_free (lua_State* L)
{
  rcl_wait_set_t* ws = lua_touserdata(L, 1);
  rcl_ret_t ret = rcl_wait_set_fini(ws);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to fini wait set: %s", rcl_get_error_string().str);
    rcl_reset_error();
  }

  return 0;
}

static int rcl_lua_wait_set_clear (lua_State* L)
{
  rcl_wait_set_t* ws = luaL_checkudata(L, 1, MT_WAIT_SET);
  rcl_ret_t ret = rcl_wait_set_clear(ws);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to clear wait set");
  }

  return 0;
}

static int rcl_lua_wait_set_add_timer (lua_State* L)
{
  /* arg1 - wait set */
  rcl_wait_set_t* ws = luaL_checkudata(L, 1, MT_WAIT_SET);
  /* arg2 - timer */
  rcl_timer_t* timer = luaL_checkudata(L, 2, MT_TIMER);

  size_t index = 0;
  rcl_ret_t ret = rcl_wait_set_add_timer(ws, timer, &index);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to add timer");
  }

  lua_pushinteger(L, (lua_Integer) index);

  return 1;
}

static int rcl_lua_wait_set_wait (lua_State* L)
{
  /* arg1 - wait set */
  rcl_wait_set_t* ws = luaL_checkudata(L, 1, MT_WAIT_SET);
  /* arg2 - timeout */
  lua_Integer timeout = luaL_checkinteger(L, 2);

  rcl_ret_t ret = rcl_wait(ws, timeout);
  if (RCL_RET_OK != ret && RCL_RET_TIMEOUT != ret) {
    luaL_error(L, "failed to wait on wait set");
  }

  return 0;
}

static int rcl_lua_wait_set_ready_timers (lua_State* L)
{
  rcl_wait_set_t* ws = luaL_checkudata(L, 1, MT_WAIT_SET);

  /* collect functions */
  lua_createtable(L, ws->size_of_timers, 0);
  for (size_t i = 0; i < ws->size_of_timers; i++) {
    lua_rawgetp(L, LUA_REGISTRYINDEX, ws->timers[i]);
    lua_rawseti(L, -2, i+1);
  }

  return 1;
}

static const struct luaL_Reg wait_set_methods[] = {
  {"add_timer", rcl_lua_wait_set_add_timer},
  {"ready_timers", rcl_lua_wait_set_ready_timers},
  {"clear", rcl_lua_wait_set_clear},
  {"__gc", rcl_lua_wait_set_free},
  {NULL, NULL}
};

void rcl_lua_add_wait_set_methods (lua_State* L)
{  
  /* make wait set */
  lua_pushcfunction(L, rcl_lua_wait_set_init);
  lua_setfield(L, -2, "new_wait_set");

  /* metamethods */
  rcl_lua_utils_add_mt(L, MT_WAIT_SET, wait_set_methods);
}
