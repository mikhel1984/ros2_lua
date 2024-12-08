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

#ifndef RCL_LUA_TIME_H
#define RCL_LUA_TIME_H

#include <lua.h>

/** Number of nanoseconds in one second */
#define NSEC_IN_SEC 1000000000

extern const char* MT_TIME;
extern const char* MT_DURATION;

/**
 * Create time metatable, add constructor to library.
 *
 * \param[inout] L Lua stack.
 */
void rcl_lua_add_time_methods (lua_State* L);

#endif  // RCL_LUA_TIME_H
