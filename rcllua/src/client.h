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

#ifndef RCL_LUA_CLIENT_H
#define RCL_LUA_CLIENT_H

#include <lua.h>

struct rcl_client_s;

/**
 * Create client metatable, add constructor to library.
 *
 * \param[inout] L Lua stack.
 */
void rcl_lua_add_client_methods (lua_State* L);

/**
 * Take response message, translate to Lua object.
 * Push to the stack table {resp, callback}.
 *
 * \param[inout] L Lua stack.
 * \param[in] cli pointer to client.
 */
void rcl_lua_client_push_response (lua_State* L, const struct rcl_client_s* cli);

#endif  // RCL_LUA_CLIENT_H
