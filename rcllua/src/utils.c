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

#include <time.h>
#include <stdlib.h>
#include <stdbool.h>
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

/**
 * Generate UUID value.
 * 
 * Return:
 * - table form uint8[16]
 * - string form
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_utils_get_uuid (lua_State* L)
{
  /* init */
  static bool gen_init = false;
  static uint64_t uuid_lsb = 0;
  static uint64_t uuid_msb = 0;

  if (!gen_init) {
    srand(time(NULL));
    uuid_lsb = (uint64_t) rand();
    uuid_msb = (uint64_t) rand();
    gen_init = true;
  }

  /* update */
  if (0 == ++uuid_lsb) {
    ++uuid_msb;
  }

  char uuid[16];
  *(uint64_t*)(&uuid[0]) = uuid_msb;
  *(uint64_t*)(&uuid[8]) = uuid_lsb;

  /* as table */
  lua_createtable(L, 16, 0);
  uint8_t *seq = (uint8_t*) uuid;
  for (int i = 0; i < 16; i++) {
    lua_pushinteger(L, seq[i]);
    lua_rawseti(L, -2, i+1);
  }
  
  /* as string */
  lua_pushlstring(L, uuid, 16);
  return 2;
}

/* Convert UUID to string, push result to stack. */
void rcl_lua_utils_push_uuid_str (lua_State* L, int pos)
{
  char uuid[16];
  for (int i = 0; i < 16; i++) {
    lua_geti(L, pos, i);
    uuid[i] = lua_tointeger(L, -1);
    lua_pop(L, 1);
  }

  lua_pushlstring(L, uuid, 16);
}

/**
 * String representation for UUID.
 *
 * Arguments:
 * - message field or table with 16 uint.
 * 
 * Return:
 * - uuid as string.
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_utils_uuid_to_str (lua_State* L)
{
  /* arg1 - table or userdata with 16 integers */
  luaL_argcheck(L, lua_istable(L, 1) || lua_isuserdata(L, 1), 1, "expected message field or table");

  rcl_lua_utils_push_uuid_str(L, 1);
  return 1;
}

/**
 * Check message type.
 *
 * Arguments:
 * - message to check
 * - interface to check
 *
 * Result:
 * - true when the message has the given type.
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_utils_is_instance (lua_State* L)
{
  bool equal = false;
  /* arg1 - message object */
  /* arg2 - message table */
  if (lua_istable(L, 2)) {
    lua_getfield(L, 2, "_metatable");    // push name
    const char* mt = lua_tostring(L, -1);
    equal = (mt != NULL) && (luaL_testudata(L, 1, mt) != NULL);
  }

  lua_pushboolean(L, equal);
  return 1;
}

/* Add to library */
void rcl_lua_add_util_methods (lua_State* L)
{
  /* sleep some time */
  lua_pushcfunction(L, rcl_lua_utils_sleep_thread);  // push function
  lua_setfield(L, -2, "sleep_thread");               // pop, lib['sleep_thread'] = fn

  lua_createtable(L, 0, 2);                          // push table
  /* generate uuid */
  lua_pushcfunction(L, rcl_lua_utils_get_uuid);      // push function
  lua_setfield(L, -2, "new");                        // pop, lib.uuid.new = fn
  /* make string key for UUID */
  lua_pushcfunction(L, rcl_lua_utils_uuid_to_str);   // push function
  lua_setfield(L, -2, "str");                        // pop, lib.uuid.str = fn
  lua_setfield(L, -2, "uuid");                       // pop, lib['uuid'] = tbl

  /* check message type */
  lua_pushcfunction(L, rcl_lua_utils_is_instance);   // push function
  lua_setfield(L, -2, "is_instance");                // pop, lib['is_instance'] = fn
}
