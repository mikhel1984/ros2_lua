
//#include <stdlib.h>
#include <lauxlib.h>

#include <rcl/allocator.h>
#include <rcl/init.h>
#include <rcl/init_options.h>

#include "context.h"

static rcl_context_t context_;
static bool context_init_ = false;

static int rcl_lua_context_init (lua_State* L)
{
  if (!lua_istable(L, 1)) {
    luaL_argerror(L, 1, "expected table of arguments");
  }
  if (context_init_) {
    // TODO(Mikhel) print message
    return 0;
  }
  context_ = rcl_get_zero_initialized_context();

  // prepare options
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_ret_t ret = rcl_init_options_init(&init_options, allocator);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to initialize options");
  }

  // read command line arguments
  size_t n = lua_rawlen(L, 1) + 1;  // table size + 1 position for zero
  const char** argv = lua_newuserdata(L, sizeof(char*) * n);
  for (size_t i = 0; i < n; ++i) {
    lua_rawgeti(L, 1, i);
    argv[i] = luaL_checkstring(L, -1);
    lua_pop(L, 1);  // string
  }

  // set domain id ?
  // init context
  ret = rcl_init((int) n, argv, &init_options, &context_);
  lua_pop(L, 1);  // free argv
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to initialize rcl");
  }
  
  context_init_ = true;
  return 0;
}

static int rcl_lua_context_ok (lua_State* L)
{
  lua_pushboolean(L, rcl_context_is_valid(&context_));
  return 1;
}

static int rcl_lua_context_shutdown (lua_State* L)
{
  rcl_ret_t ret = rcl_shutdown(&context_);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to shutdown");
  }
  return 0;
}

static int rcl_lua_context_free (lua_State* L)
{
  rcl_ret_t ret;
  if (rcl_context_is_valid(&context_)) {
    ret = rcl_shutdown(&context_);
    if (RCL_RET_OK != ret) {
      luaL_error(L, "failed to shutdown");
    }
  }
  ret = rcl_context_fini(&context_);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to fini rcl_context_t");
  }
  return 0;
}

static const struct luaL_Reg context_lib [] =
{
  {"context_init", rcl_lua_context_init},
  {"context_ok", rcl_lua_context_ok},
  {"context_shutdown", rcl_lua_context_shutdown},
  {"context_free", rcl_lua_context_free},  // TODO(Mikhel) __gc
  {NULL, NULL}
};

void rcl_lua_add_context_methods (lua_State* L)
{
  luaL_setfuncs(L, context_lib, 0);
}

rcl_context_t* rcl_lua_context_ref()
{
  return context_init_ ? &context_ : NULL;
}
