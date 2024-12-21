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

#ifndef RCLLUA__UTILS_H_
#define RCLLUA__UTILS_H_

#include <lua.h>

struct luaL_Reg;

/** Keep pairs 'name - value' to fill 'enum' in Lua. */
typedef struct rcl_lua_enum
{
  /** field name */
  const char* name;
  /** integer */
  int value;
} rcl_lua_enum;

/**
 * Save methods to metatable.
 *
 * \param[inout] L Lua stack.
 * \param[in] name metatable name.
 * \param[in] fn function list.
 */
void rcl_lua_utils_add_mt (lua_State* L, const char* name, const struct luaL_Reg* fn);

/**
 * Combine list of pairs to 'enum', add to library.
 *
 * \param[inout] L Lua stack.
 * \param[in] name desired table (enum) name.
 * \param[in] ps list of pairs 'name - value'.
 */
void rcl_lua_utils_add_enum (lua_State* L, const char* name, const rcl_lua_enum* ps);

/**
 * Add useful functions to library.
 *
 * \param[inout] L Lua stack.
 */
void rcl_lua_add_util_methods (lua_State* L);

#endif  // RCLLUA__UTILS_H_
