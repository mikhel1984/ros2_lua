
#include <lauxlib.h>

#include "utils.h"

void rcl_lua_utils_add_mt (lua_State* L, const char* name, const luaL_Reg* fn)
{
  // metatable
  luaL_newmetatable(L, name);
  lua_pushvalue(L, -1);
  lua_setfield(L, -2, "__index");

  // methods
  luaL_setfuncs(L, fn, 0);

  // remove methatable from stack
  lua_pop(L, 1);
}

void rcl_lua_utils_add_enum (lua_State* L, const char* name, const rcl_lua_enum* ps)
{
  int n = 0;
  for (n = 0; ps[n].name; ++n) {}  // count

  // fill table
  lua_createtable(L, 0, n);
  for (int i = 0; i < n; ++i) {
    lua_pushinteger(L, ps[i].value);
    lua_setfield(L, -2, ps[i].name);
  }

  // save table
  lua_setfield(L, -2, name);
}
