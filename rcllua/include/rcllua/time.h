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

#ifndef RCLLUA__TIME_H_
#define RCLLUA__TIME_H_

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

/**
 * Create time object, init and push to the stack.
 *
 * \param[inout] L Lua stack.
 * \param[in] ns time in nanoseconds.
 * \param[in] clock_type type of clock.
 */
void rcl_lua_time_push_time (lua_State* L, int64_t ns, int clock_type);

/**
 * Create duration object, init and push to the stack.
 *
 * \param[inout] L Lua stack.
 * \param[in] ns time in nanoseconds.
 */
void rcl_lua_time_push_duration (lua_State* L, int64_t ns);

#endif  // RCLLUA__TIME_H_
