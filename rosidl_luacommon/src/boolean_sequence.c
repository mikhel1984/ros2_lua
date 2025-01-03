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

#include <rosidl_runtime_c/primitives_sequence.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>

#include "rosidl_luacommon/definition.h"
#include "rosidl_luacommon/sequence_macro.h"

/* Boolean sequence metatable name. */
const char* MT_SEQ_BOOLEAN = "ROS2.rosidl_sequence.boolean";

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
  bool* lst = NULL;
  if (ptr->value > 0) {
    /* array */
    if (0 < ind && ind <= ptr->value) {
      lst = ptr->obj;
    }
  } else if (ptr->value == IDL_LUA_SEQ) {
    /* sequence */
    rosidl_runtime_c__boolean__Sequence* seq = ptr->obj;
    if (0 < ind && ((size_t) ind) <= seq->size) {
      lst = seq->data;
    }
  } else {
    luaL_error(L, "not an array");
  }

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
  bool* lst = NULL;
  if (ptr->value > 0) {
    /* array */
    if (0 < ind && ind <= ptr->value) {
      lst = ptr->obj;
    }
  } else if (ptr->value == IDL_LUA_SEQ) {
    /* sequence */
    rosidl_runtime_c__boolean__Sequence* seq = ptr->obj;
    if (0 < ind && ((size_t) ind) <= seq->size) {
      lst = seq->data;
    }
  } else {
    luaL_error(L, "not an array");
  }

  /* get */
  if (lst) {
    lua_pushboolean(L, lst[ind-1]);
  } else {
    lua_pushnil(L);
  }

  return 1;
}

OBJ_SEQ_COPY (boolean, bool, MT_SEQ_BOOLEAN)
OBJ_SEQ_RESIZE (boolean, MT_SEQ_BOOLEAN)

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

  if (LUA_TUSERDATA == tp) {
    /* arg2 - other message */
    return boolean_seq_copy(L);

  } else if (LUA_TNUMBER == tp) {
    /* arg2 - size */
    return boolean_seq_resize(L);

  } else if (LUA_TTABLE == tp) {
    /* arg2 - table */
    lua_len(L, 2);                 // push len
    int len = luaL_checkinteger(L, -1);
    idl_lua_msg_t* msg = lua_touserdata(L, 1);

    if (len > 0 && (IDL_LUA_SEQ == msg->value || msg->value == len)) {
      bool* lst = msg->obj;
      if (IDL_LUA_SEQ == msg->value) {
        /* resize sequence */
        lua_insert(L, 2);          // set len before table
        boolean_seq_resize(L);
        if (!lua_toboolean(L, -1)) {
          /* resize failed */
          return 1;
        }
        rosidl_runtime_c__boolean__Sequence* seq = msg->obj;
        lst = seq->data;
        lua_remove(L, 2);          // pop len
      }
      lua_pop(L, 1);               // pop top (len or call result)
      /* copy */
      bool stop = false;
      for (int i = 0; i < len; i++) {
        lua_pushinteger(L, i+1);   // push index
        lua_gettable(L, 2);        // pop index, push value
        if (LUA_TBOOLEAN != lua_type(L, -1)) {
          stop = true;
          break;
        }
        lst[i] = lua_toboolean(L, -1);
        lua_pop(L, 1);             // pop value
      }
      done = !stop;
    }
  }

  lua_pushboolean(L, done);
  return 1;
}


OBJ_SEQ_EQ (boolean, MT_SEQ_BOOLEAN)
OBJ_SEQ_LEN (boolean)
OBJ_SEQ_STR (boolean)

OBJ_METHODS(boolean, boolean_seq_len)
OBJ_ADD_TABLE (boolean, MT_SEQ_BOOLEAN)
