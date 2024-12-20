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

#include <rosidl_runtime_c/string.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>

#include "rosidl_luacommon/definition.h"
#include "rosidl_luacommon/sequence_macro.h"

/** String sequence metatable name. */
const char* MT_SEQ_STRING = "ROS2.rosidl_sequence.String";

/**
 * Set string by index.
 *
 * Arguments:
 * - message wrapper
 * - index
 * - value
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int String_seq_set (lua_State* L)
{
  /* arg1 - message wrapper */
  idl_lua_msg_t* ptr = lua_touserdata(L, 1);
  /* arg2 - index */
  lua_Integer ind = luaL_checkinteger(L, 2);

  /* arg3 - string */
  if (!lua_isstring(L, 3)) {
    luaL_error(L, "string is expected");
  }
  size_t len = 0;
  const char* val = lua_tolstring(L, 3, &len);

  /* get pointer */
  rosidl_runtime_c__String* lst = NULL;
  if (ptr->value > 0) {
    /* array */
    if (0 < ind && ind <= ptr->value) {
      lst = ptr->obj;
    }
  } else if (ptr->value == IDL_LUA_SEQ) {
    /* sequence */
    rosidl_runtime_c__String__Sequence* seq = ptr->obj;
    if (0 < ind && ((size_t) ind) <= seq->size) {
      lst = seq->data;
    }
  } else {
    luaL_error(L, "unexpected object");
  }

  /* set */
  if (lst) {
    rosidl_runtime_c__String__assignn(lst + (ind-1), val, len);
  }

  return 0;
}

/**
 * Get string by index.
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
static int String_seq_get (lua_State* L)
{
  /* arg1 - message wrapper */
  idl_lua_msg_t* ptr = lua_touserdata(L, 1);
  /* arg2 - index */
  lua_Integer ind = luaL_checkinteger(L, 2);

  /* get pointer */
  rosidl_runtime_c__String* lst = NULL;
  if (ptr->value > 0) {
    /* array */
    if (0 < ind && ind <= ptr->value) {
      lst = ptr->obj;
    }
  } else if (ptr->value == IDL_LUA_SEQ) {
    /* sequence */
    rosidl_runtime_c__String__Sequence* seq = ptr->obj;
    if (0 < ind && ((size_t) ind) <= seq->size) {
      lst = seq->data;
    }
  } else {
    luaL_error(L, "unexpected object");
  }

  /* set */
  if (lst) {
    rosidl_runtime_c__String* s = lst + (ind-1);
    lua_pushlstring(L, s->data, s->size);
  } else {
    lua_pushnil(L);
  }

  return 1;
}

/**
 * Copy string sequence data.
 *
 * Arguments:
 * - destination sequence
 * - source sequence
 *
 * Return:
 * - true on success
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int String_seq_copy (lua_State* L)
{
  /* arg1 - dst sequence */
  idl_lua_msg_t* dst = luaL_checkudata(L, 1, MT_SEQ_STRING);
  /* arg2 - src sequence */
  idl_lua_msg_t* src = luaL_checkudata(L, 2, MT_SEQ_STRING);
  bool done = false;

  if (dst->value == IDL_LUA_SEQ) {
    /* sequence */
    if (src->value == IDL_LUA_SEQ) {
      /* to sequence */
      done = rosidl_runtime_c__String__Sequence__copy(src->obj, dst->obj);
    } else if (src->value > 0) {
      /* to array */
      rosidl_runtime_c__String__Sequence tmp;
      tmp.data = src->obj;
      tmp.size = tmp.capacity = (size_t) src->value;
      done = rosidl_runtime_c__String__Sequence__copy(&tmp, dst->obj);
    }
  } else if (dst->value > 0) {
    /* array */
    rosidl_runtime_c__String *a = dst->obj, *b = NULL;
    if (src->value > 0 && dst->value == src->value) {
      /* to array */
      b = src->obj;
    } else if (src->value == IDL_LUA_SEQ) {
      /* to sequence */
      rosidl_runtime_c__String__Sequence* seq = src->obj;
      if (seq->size == (size_t) dst->value) {
        b = seq->data;
      }
    }
    if (b != NULL) {
      for (int i = 0; i < dst->value; i++) {
        rosidl_runtime_c__String__copy(b + i, a + i);
      }
      done = true;
    }
  }

  lua_pushboolean(L, done);
  return 1;
}

OBJ_SEQ_RESIZE (String, MT_SEQ_STRING)

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
static int String_seq_call (lua_State* L)
{
  bool done = false;
  int tp = lua_type(L, 2);

  if (LUA_TUSERDATA == tp) {
    /* arg2 - other message */
    return String_seq_copy(L);

  } else if (LUA_TNUMBER == tp) {
    /* arg2 - size */
    return String_seq_resize(L);

  } else if (LUA_TTABLE == tp) {
    /* arg2 - table */
    lua_len(L, 2);                  // push len
    int len = luaL_checkinteger(L, -1);
    idl_lua_msg_t* msg = lua_touserdata(L, 1);
    if (len > 0 && (IDL_LUA_SEQ == msg->value || msg->value == len)) {
      rosidl_runtime_c__String* lst = msg->obj;
      if (IDL_LUA_SEQ == msg->value) {
        /* resize sequence */
        lua_insert(L, 2);           // set len before table
        String_seq_resize(L);
        if (!lua_toboolean(L, -1)) {
          /* resize failed */
          return 1;
        }
        rosidl_runtime_c__String__Sequence* seq = msg->obj;
        lst = seq->data;
        lua_remove(L, 2);           // pop len
      }
      lua_pop(L, 1);                // pop top (len or call result)
      /* copy */
      bool stop = false;
      for (int i = 0; i < len; i++) {
        lua_pushinteger(L, i+1);    // push index
        lua_gettable(L, 2);         // pop index, push value
        if (LUA_TSTRING != lua_type(L, -1)) {
          stop = true;
          break;
        }
        size_t str_len = 0;
        const char* str_data = lua_tolstring(L, -1, &str_len);
        if (!rosidl_runtime_c__String__assignn(lst+i, str_data, str_len)) {
          stop = true;
          break;
        }
        lua_pop(L, 1);              // pop value
      }
      done = !stop;
    }
  }
  lua_pushboolean(L, done);

  return 1;
}

OBJ_SEQ_EQ (String, MT_SEQ_STRING)
OBJ_SEQ_LEN (String)
OBJ_SEQ_STR (String)

OBJ_METHODS (String, String_seq_len)
OBJ_ADD_TABLE (String, MT_SEQ_STRING)
