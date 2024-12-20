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

#include "rcllua/context.h"
#include "rcllua/logger.h"
#include "rcllua/node.h"
#include "rcllua/publisher.h"
#include "rcllua/subscriber.h"
#include "rcllua/service.h"
#include "rcllua/client.h"
#include "rcllua/time.h"
#include "rcllua/timer.h"
#include "rcllua/clock.h"
#include "rcllua/qos.h"
#include "rcllua/wait_set.h"

/**
 * Create rclbind library.
 *
 * Pass Lua table to all them method in other to fill it with
 * requiired constructors, metatables and enums.
 *
 * \param[inout] L Lua stack.
 */

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
  rcl_lua_add_subscription_methods(L);
  rcl_lua_add_service_methods(L);
  rcl_lua_add_client_methods(L);
  rcl_lua_add_wait_set_methods(L);

  return 1;
}
