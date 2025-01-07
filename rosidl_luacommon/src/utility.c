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

// It is assumed that all Sequence objects has the same structure.
// So the boolean Sequence is used as a template.

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
    /* list */
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
    /* list */
    rosidl_runtime_c__boolean__Sequence* seq = ptr->obj;
    lua_pushfstring(L, "%s sequence of size %d", prefix, seq->size);
  } else {
    lua_pushfstring(L, "%s", prefix);
  }

  return 1;
}

int rosidl_luacommon_push_realloc (lua_State* L, msg_mem_realloc fn)
{
  idl_lua_msg_t* ptr = lua_touserdata(L, 1);
  if (ptr->value != IDL_LUA_SEQ) {
    /* only list can be resized */
    lua_pushboolean(L, false);
    return 1;
  }
    
  /* new length */
  lua_Integer len = luaL_checkinteger(L, 2);
  luaL_argcheck(L, len >= 0, 2, "wrong length");
  
  /* list */
  rosidl_runtime_c__boolean__Sequence* seq = ptr->obj;
  bool done = true;  
  if (seq->capacity >= (size_t) len) {
    /* memory is enough */
    seq->size = (size_t) len;
  } else if (seq->capacity == 0 || seq->size == 0) {
    /* empty object, make new */
    done = fn(ptr, (size_t) len, false);
  } else {
    /* resize and copy data */
    done = fn(ptr, (size_t) len, true);    
  }

  lua_pushboolean(L, done);
  return 1;
}

void* rosidl_luacommon_array_check_ind (idl_lua_msg_t* msg, int ind)
{
  if (msg->value > IDL_LUA_SEQ) {
    if (0 < ind && ind <= msg->value) {
      return msg->obj;
    }
  } else {
    /* list */
    rosidl_runtime_c__boolean__Sequence *seq = msg->obj;
    if (0 < ind && ((size_t) ind) <= seq->size) {
      return (void*) seq->data;
    }
  }
  return NULL;
}

void rosidl_luacommon_field_apply (lua_State* L, const char* table, int top)
{
  /* nested object, other metatable, get by name */
  if (luaL_getmetafield(L, 1, table) != LUA_TTABLE) {  // push table
    luaL_error(L, "wrong metatable");
  }
  lua_pushvalue(L, 2);                      // push key (duplicate)
  lua_gettable(L, -2);                      // pop key, push function
  lua_CFunction fn = lua_tocfunction(L, -1);
  if (NULL == fn) {
    luaL_error(L, "unknown field '%s'", lua_tostring(L, 2));
  }
  lua_settop(L, top);                       // stack [object, key, value]
  fn(L);                                    // execute, push result if need
}
