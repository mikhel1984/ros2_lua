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

#ifndef RCL_LUA_TIMER_H
#define RCL_LUA_TIMER_H

#include <lua.h>

struct rcl_timer_s;

extern const char* MT_TIMER;

/**
 * Create timer metatable, add constructor to library.
 *
 * \param[inout] L Lua stack.
 */
void rcl_lua_add_timer_methods (lua_State* L);

/**
 * Push to the stack timer callback and reference (light userdata).
 *
 * \param[inout] L Lua stack.
 * \param[in] timer pointer to timer.
 */
void rcl_lua_timer_push_callback (lua_State* L, const struct rcl_timer_s* timer);

#endif  // RCL_LUA_TIMER_H
