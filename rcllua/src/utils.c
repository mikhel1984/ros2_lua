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

#include <threads.h>
#include <lauxlib.h>

#include "rcllua/utils.h"

/* Save metatable */
void rcl_lua_utils_add_mt (lua_State* L, const char* name, const luaL_Reg* fn)
{
  /* table */
  luaL_newmetatable(L, name);      // push metatable
  lua_pushvalue(L, -1);            // push metatable ref
  lua_setfield(L, -2, "__index");  // pop metatable ref, set field
  /* add methods */
  luaL_setfuncs(L, fn, 0);
  /* clear */
  lua_pop(L, 1);                   // pop metatable
}

/* Save 'enum' */
void rcl_lua_utils_add_enum (lua_State* L, const char* name, const rcl_lua_enum* ps)
{
  /* find length */
  int n = 0;
  for (n = 0; ps[n].name; ++n) {}
  /* fill table */
  lua_createtable(L, 0, n);           // push table a
  for (int i = 0; i < n; ++i) {
    lua_pushinteger(L, ps[i].value);  // push value
    lua_setfield(L, -2, ps[i].name);  // pop, a.nm = value
  }
  /* save table */
  lua_setfield(L, -2, name);          // pop, lib['name'] = a
}

/**
 * Stop execution for some time.
 *
 * Arguments:
 * - sleep duration, seconds
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_utils_sleep_thread (lua_State* L)
{
  /* arg1 - time value */
  double sec = luaL_checknumber(L, 1);
  luaL_argcheck(L, sec >= 1E-9, 1, "duration >= 1ns is expected");
  long full = (long) sec;
  long part = (long) ((sec - full)*1E9);

  /* sleep */
  struct timespec time;
  time.tv_sec = full;
  time.tv_nsec = part;
  
  thrd_sleep(&time, NULL);

  return 0;
}

/* Add to library */
void rcl_lua_add_util_methods (lua_State* L)
{
  /* sleep some time */
  lua_pushcfunction(L, rcl_lua_utils_sleep_thread);  // push function
  lua_setfield(L, -2, "sleep_thread");               // pop, lib['sleep_thread'] = fn
}
