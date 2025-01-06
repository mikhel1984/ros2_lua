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

#ifndef ROSIDL_LUACOMMON__UTILITY_H_
#define ROSIDL_LUACOMMON__UTILITY_H_

#include <lua.h>

#include "rosidl_luacommon/definition.h"

int rosidl_luacommon_push_length (lua_State* L);

int rosidl_luacommon_push_msg_string (lua_State* L, const char* prefix);

#endif  // ROSIDL_LUACOMMON__UTILITY_H_

