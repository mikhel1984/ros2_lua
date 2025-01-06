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

#include <lauxlib.h>

#include <rosidl_runtime_c/primitives_sequence.h>

#include "rosidl_luacommon/utility.h"

int rosidl_luacommon_push_length (lua_State* L)
{
  idl_lua_msg_t* ptr = lua_touserdata(L, 1);
  if (ptr->value > IDL_LUA_SEQ) {
    /* array */
    lua_pushinteger(L, ptr->value);
  } else if (ptr->value == IDL_LUA_SEQ) {
    /* list, assume message structure is the same for all types */
    rosidl_runtime_c__boolean__Sequence* seq = ptr->obj;
    lua_pushinteger(L, seq->size);
  } else {
    /* scalar value */
    lua_pushnil(L);
  }

  return 1;
}

int rosidl_luacommon_push_msg_string (lua_State* L, const char* prefix)
{
  idl_lua_msg_t* ptr = lua_touserdata(L, 1);
  if (ptr->value > IDL_LUA_SEQ) {
    lua_pushfstring(L, "%s array of size %d", prefix, ptr->value);
  } else if (ptr->value == IDL_LUA_SEQ) {
    /* list, assume message structure is the same for all types */
    rosidl_runtime_c__boolean__Sequence* seq = ptr->obj;
    lua_pushfstring(L, "%s sequence of size %d", prefix, seq->size);
  } else {
    lua_pushfstring(L, "%s", prefix);
  }

  return 1;
}
