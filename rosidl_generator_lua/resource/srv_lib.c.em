
#include <lua.h>
#include <lauxlib.h>

@{
function_list = ['aabbbcc']
}@

// prototypes

@[for function in function_list]@
void @(function[0]) (lua_State* L);
@[end for]@

// library

int luaopen_msg (lua_State* L)
{
  lua_createtable(L, 0, 0);

@[for function in function_list]@
  @(function[0])(L);
@[end for]@

  return 1;
}

