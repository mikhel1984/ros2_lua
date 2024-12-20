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
