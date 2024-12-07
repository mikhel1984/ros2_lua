
#include <lua.h>

#include "context.h"
#include "logger.h"
#include "node.h"
#include "publisher.h"
#include "time.h"
#include "timer.h"
#include "clock.h"
#include "qos.h"
#include "wait_set.h"

int luaopen_rcllua_rclbind (lua_State* L)
{
  lua_createtable(L, 0, 20); // TODO set number

  rcl_lua_add_context_methods(L);
  rcl_lua_add_logger_methods(L);

  rcl_lua_add_time_methods(L);
  rcl_lua_add_timer_methods(L);
  rcl_lua_add_clock_methods(L);
  rcl_lua_add_qos_methods(L);

  rcl_lua_add_node_methods(L);
  rcl_lua_add_publisher_methods(L);
  rcl_lua_add_wait_set_methods(L);

  return 1;
}
