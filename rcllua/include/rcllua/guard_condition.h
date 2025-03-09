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

#ifndef RCLLUA__GUARD_CONDITION_H_
#define RCLLUA__GUARD_CONDITION_H_

#include <lua.h>

struct rcl_guard_condition_s;

extern const char* MT_GUARD_CONDITION;

/**
 * Create guard condition metatable, add constructor to library.
 *
 * \param[inout] L Lua stack.
 */
void rcl_lua_add_guard_condition_methods (lua_State* L);

/**
 * Push to the stack callback if any.
 *
 * \param[inout] L Lua stack.
 * \param[in] guard Guard condition pointer.
 * \return true when callback is found.
 */
bool rcl_lua_guard_condition_push_callback (
  lua_State* L, 
  const struct rcl_guard_condition_s* guard);

#endif  // RCLLUA__GUARD_CONDITION_H_
