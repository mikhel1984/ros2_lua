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

#include "rosidl_luacommon/definition.h"

/* Prototipes */

void rosidl_luacommon_add_float (lua_State* L);
void rosidl_luacommon_add_double (lua_State* L);
void rosidl_luacommon_add_long_double (lua_State* L);

void rosidl_luacommon_add_int8 (lua_State* L);
void rosidl_luacommon_add_uint8 (lua_State* L);
void rosidl_luacommon_add_int16 (lua_State* L);
void rosidl_luacommon_add_uint16 (lua_State* L);
void rosidl_luacommon_add_int32 (lua_State* L);
void rosidl_luacommon_add_uint32 (lua_State* L);
void rosidl_luacommon_add_int64 (lua_State* L);
void rosidl_luacommon_add_uint64 (lua_State* L);

void rosidl_luacommon_add_boolean (lua_State* L);
void rosidl_luacommon_add_String (lua_State* L);

/**
 * Make Lua library with sequences of primitive types.
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
int luaopen_rosidl_luacommon_sequence (lua_State* L)
{
  lua_createtable(L, 0, 0);  // TODO save metatables 
  
  /* prepare metatables */
  rosidl_luacommon_add_float(L);
  rosidl_luacommon_add_double(L);
  rosidl_luacommon_add_long_double(L);
  
  rosidl_luacommon_add_int8(L);
  rosidl_luacommon_add_uint8(L);
  rosidl_luacommon_add_int16(L);
  rosidl_luacommon_add_uint16(L);
  rosidl_luacommon_add_int32(L);
  rosidl_luacommon_add_uint32(L);
  rosidl_luacommon_add_int64(L);
  rosidl_luacommon_add_uint64(L);  
  
  rosidl_luacommon_add_boolean(L);
  rosidl_luacommon_add_String(L);
  
  return 1;  
}
