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

#include <rcl/guard_condition.h>

#include "rcllua/guard_condition.h"
#include "rcllua/context.h"
#include "rcllua/utils.h"

/** Guard condition metatable name. */
const char* MT_GUARD_CONDITION = "ROS2.GuardCondition";

/**
 * Create guard condition object.
 * 
 * Arguments:
 * - callback (=nil)
 *
 * Return:
 * - new guard condition.
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_guard_condition_init (lua_State* L)
{
  /* make object */
  rcl_guard_condition_t* guard = lua_newuserdata(L, sizeof(rcl_guard_condition_t));
  *guard = rcl_get_zero_initialized_guard_condition();
  rcl_guard_condition_options_t opt = rcl_guard_condition_get_default_options();

  /* init */
  rcl_ret_t ret = rcl_guard_condition_init(guard, rcl_lua_context_ref(), opt);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to create guard condition");
  }

  /* set metatable */
  luaL_getmetatable(L, MT_GUARD_CONDITION);
  lua_setmetatable(L, -2);

  if (lua_isfunction(L, 1)) {
    lua_pushvalue(L, 1);
    lua_rawsetp(L, LUA_REGISTRYINDEX, guard);
  }

  return 1;
}

/**
 * Guard condition destructor.
 *
 * Arguments:
 * - guard condition object
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_guard_condition_free (lua_State* L)
{
  /* arg1 - guard condition object */
  rcl_guard_condition_t* guard = lua_touserdata(L, 1);

  rcl_ret_t ret = rcl_guard_condition_fini(guard);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to fini guard condition");
  }

  return 0;
}

/**
 * Trigger a general purpose guard condition.
 *
 * Arguments:
 * - guard condition object
 * 
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_guard_condition_trigger (lua_State* L)
{
  /* arg1 - guard condition object */
  rcl_guard_condition_t* guard = lua_touserdata(L, 1);
  luaL_argcheck(L, NULL != guard, 1, "guard condition is expected");

  rcl_ret_t ret = rcl_trigger_guard_condition(guard);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to trigger guard condition");
  }
  
  return 0;
}

/** List of guard condition methods */
static const struct luaL_Reg guard_methods[] = {
  {"trigger", rcl_lua_guard_condition_trigger},
  {"__gc", rcl_lua_guard_condition_free},
  {NULL, NULL}
};

/* Add guard condition to library */
void rcl_lua_add_guard_condition_methods (lua_State* L)
{
  /* constructor */
  lua_pushcfunction(L, rcl_lua_guard_condition_init);  // push function
  lua_setfield(L, -2, "new_guard_condition");          // pop, lib['new_guard_condition'] = fn

  /* metamethods */
  rcl_lua_utils_add_mt(L, MT_GUARD_CONDITION, guard_methods);
}

bool rcl_lua_guard_condition_push_callback (lua_State* L, const rcl_guard_condition_t* guard)
{
  return lua_rawgetp(L, LUA_REGISTRYINDEX, guard) != LUA_TNIL;
}
