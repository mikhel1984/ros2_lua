
#include <lua.h>

#include "context.h"

int luaopen_rclbind (lua_State* L)
{
  lua_createtable(L, 0, 10);

  rcl_lua_add_context_methods(L);

  return 1;
}
