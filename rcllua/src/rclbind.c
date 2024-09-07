
#include <lua.h>

#include "context.h"
#include "logger.h"
#include "node.h"
#include "time.h"

int luaopen_rclbind (lua_State* L)
{
  lua_createtable(L, 0, 10); // TODO(set number)

  rcl_lua_add_context_methods(L);
  rcl_lua_add_logger_methods(L);

  rcl_lua_add_time_methods(L);
  rcl_lua_add_timer_methods(L);
  rcl_lua_add_clock_methods(L);

  rcl_lua_add_node_methods(L);

  return 1;
}
