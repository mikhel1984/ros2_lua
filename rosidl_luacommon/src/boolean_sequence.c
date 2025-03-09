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

#include <string.h>

#include <rosidl_runtime_c/primitives_sequence.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>

#include "rosidl_luacommon/definition.h"
#include "rosidl_luacommon/sequence_macro.h"
#include "rosidl_luacommon/utility.h"

/* Boolean sequence metatable name. */
const char* MT_SEQ_BOOLEAN = "primitives_sequence__msg__boolean__mt";

/**
 * Set boolean value by index.
 *
 * Arguments:
 * - message wrapper
 * - index
 * - value
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int boolean_seq_set (lua_State* L)
{
  /* arg1 - message wrapper */
  idl_lua_msg_t* ptr = lua_touserdata(L, 1);
  /* arg2 - index */
  lua_Integer ind = luaL_checkinteger(L, 2);

  /* arg3 - value */
  if (!lua_isboolean(L, 3)) {
    luaL_error(L, "boolean expected");
  }
  bool val = lua_toboolean(L, 3);

  /* get pointer */
  bool* lst = rosidl_luacommon_array_check_ind(ptr, ind);

  /* set */
  if (lst) {
    lst[ind-1] = val;
  }

  return 0;
}

/**
 * Get boolean value by index.
 *
 * Arguments:
 * - message wrapper
 * - index
 *
 * Return:
 * - value or nil
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int boolean_seq_get (lua_State* L)
{
  /* arg1 - message wrapper */
  idl_lua_msg_t* ptr = lua_touserdata(L, 1);
  /* arg2 - index */
  lua_Integer ind = luaL_checkinteger(L, 2);

  /* get pointer */
  bool* lst = rosidl_luacommon_array_check_ind(ptr, ind);

  /* get */
  if (lst) {
    lua_pushboolean(L, lst[ind-1]);
  } else {
    lua_pushnil(L);
  }

  return 1;
}

OBJ_SEQ_DO_RESIZE(boolean)
OBJ_SEQ_RESIZE(boolean)

OBJ_SEQ_COPY (boolean, bool, MT_SEQ_BOOLEAN)

/**
 * Call message as function.
 *
 * Can be used to copy other message, set fields, resize sequence.
 *
 * Arguments:
 * - message wrapper
 * - other wrapper | table | new size
 *
 * Return:
 * - true in the case of success
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int boolean_seq_call (lua_State* L)
{
  bool done = false;
  int tp = lua_type(L, 2);

  if (LUA_TSTRING == tp) {
    /* arg2 - command */
    const char* cmd = lua_tostring(L, 2);

    if (strcmp(cmd, "resize") == 0) {
      /* arg3 - new size */
      lua_remove(L, 2);
      return boolean_seq_resize(L);

    } else if (strcmp(cmd, "copy") == 0) {
      idl_lua_msg_t* msg = lua_touserdata(L, 1);
      size_t arr_len = 0, arr_cap = 0;
      bool* lst = rosidl_luacommon_list_info(msg, &arr_len, &arr_cap);
      lua_createtable(L, arr_len, 0);   // push table for data array
      for (size_t i = 0; i < arr_len; i++) {
        lua_pushboolean(L, lst[i]);   // push value
        lua_rawseti(L, -2, i+1);      // pop value
      }
      return 1;
    }
 
  } else if (LUA_TUSERDATA == tp) {
    /* arg2 - other message */
    return boolean_seq_copy(L);

  } else if (LUA_TTABLE == tp) {
    /* arg2 - table */
    idl_lua_msg_t* msg = lua_touserdata(L, 1);
    int len = luaL_len(L, 2);

    if (len > 0 && msg->value >= IDL_LUA_SEQ) {
      /* check array */
      size_t arr_len = 0, arr_cap = 0;
      bool* lst = rosidl_luacommon_list_info(msg, &arr_len, &arr_cap);
      if (arr_len != (size_t) len) {
        if (IDL_LUA_SEQ == msg->value) {
          if ((size_t) len <= arr_cap) {
            ((rosidl_runtime_c__boolean__Sequence*)msg->obj)->size = (size_t) len;
          } else if (!boolean_do_resize(msg, (size_t) len, false)) {
            goto failed;
          }
          lst = ((rosidl_runtime_c__boolean__Sequence*)msg->obj)->data;
        } else {
          goto failed;
        }
      }
      /* copy */
      for (int i = 0; i < len; i++) {
        lua_pushinteger(L, i+1);   // push index
        lua_gettable(L, 2);        // pop index, push value
        if (LUA_TBOOLEAN != lua_type(L, -1)) {
          goto failed;
        }
        *lst++ = lua_toboolean(L, -1);
        lua_pop(L, 1);             // pop value
      }
      done = true;
    }
  }

failed:
  lua_pushboolean(L, done);
  return 1;
}

OBJ_SEQ_LEN (boolean)

OBJ_SEQ_EQ (boolean, MT_SEQ_BOOLEAN)
OBJ_SEQ_STR (boolean)
OBJ_SEQ_BNOT (boolean)

OBJ_METHODS(boolean, boolean_seq_len)
OBJ_ADD_TABLE (boolean, MT_SEQ_BOOLEAN)
