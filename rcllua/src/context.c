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

#include <lauxlib.h>

#include <rcl/allocator.h>
#include <rcl/init.h>
#include <rcl/init_options.h>
#include <rcl/logging.h>

#include "rcllua/context.h"

/** Keep context state. */
static rcl_context_t context_;

/** Initialization flag */
static bool context_init_ = false;

/**
 * Initialize context state.
 *
 * Arguments:
 * - command line arguments (table)
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */

static int rcl_lua_context_init (lua_State* L)
{
  /* arg1 - command line arguments */
  luaL_argcheck(L, lua_istable(L, 1), 1, "expected table of arguments");
  // TODO read options

  if (context_init_) {
    return 0;
  }
  context_ = rcl_get_zero_initialized_context();

  /* prepare options */
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_ret_t ret = rcl_init_options_init(&init_options, allocator);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to initialize options");
  }

  /* get strings from command line table */
  size_t n = lua_rawlen(L, 1) + 1;  // table size and 1 position for zero
  const char** argv = lua_newuserdata(L, sizeof(char*) * n);  // push argv
  for (size_t i = 0; i < n; ++i) {
    lua_rawgeti(L, 1, i);           // push value
    argv[i] = luaL_checkstring(L, -1);
    lua_pop(L, 1);                  // pop value
  }

  /* init context */
  ret = rcl_init((int) n, argv, &init_options, &context_);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to initialize rcl");
  }

  /* init logger */
  rcutils_ret_t rcret = rcutils_logging_initialize();
  if (RCUTILS_RET_OK != rcret) {
    rcutils_reset_error();
    luaL_error(L, "failed to initialize logging system");
  }

  lua_pop(L, 1);                   // pop argv
  context_init_ = true;

  return 0;
}

/**
 * Check context status.
 *
 * Return:
 * - true if the context is valid.
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_context_ok (lua_State* L)
{
  lua_pushboolean(L, rcl_context_is_valid(&context_));  // push flag

  return 1;
}

/**
 * Finalize execution, free common objects.
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_context_shutdown (lua_State* L)
{
  rcl_ret_t ret = rcl_shutdown(&context_);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to shutdown");
  }

  rcutils_ret_t rcret = rcutils_logging_shutdown();
  if (RCUTILS_RET_OK != rcret) {
    rcutils_reset_error();
    luaL_error(L, "failed to shutdown logging system");
  }

  return 0;
}

/**
 * Context destructor.
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_context_free (lua_State* L)
{
  if (rcl_context_is_valid(&context_)) {
    rcl_lua_context_shutdown(L);
  }

  rcl_ret_t ret = rcl_context_fini(&context_);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to fini rcl_context_t");
  }
  return 0;
}

/** Context aware methods */
static const struct luaL_Reg context_lib [] =
{
  {"context_init", rcl_lua_context_init},
  {"context_ok", rcl_lua_context_ok},
  {"context_shutdown", rcl_lua_context_shutdown},
  {"context_free", rcl_lua_context_free},  // TODO(Mikhel) __gc
  {NULL, NULL}
};

/* Add methods to library */
void rcl_lua_add_context_methods (lua_State* L)
{
  luaL_setfuncs(L, context_lib, 0);
}

/* Get pointer to the context object */
rcl_context_t* rcl_lua_context_ref()
{
  return context_init_ ? &context_ : NULL;
}
