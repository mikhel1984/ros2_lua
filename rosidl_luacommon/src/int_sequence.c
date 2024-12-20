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

#include <stdint.h>

#include <rosidl_runtime_c/primitives_sequence.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>

#include "rosidl_luacommon/definition.h"
#include "rosidl_luacommon/sequence_macro.h"

/**
 * Set value by index.
 *
 * \param STRUCT_NAME rosidl structure name
 * \param TYPE_NAME C data type name
 * \param V_MIN lower limit
 * \param V_MAX upper limit
 * \return setter function
 */
#define INT_SEQ_SET(STRUCT_NAME, TYPE_NAME, V_MIN, V_MAX) \
static int STRUCT_NAME ## _seq_set (lua_State* L) \
{ \
  idl_lua_msg_t* ptr = lua_touserdata(L, 1); \
  lua_Integer ind = luaL_checkinteger(L, 2); \
  lua_Integer val = luaL_checkinteger(L, 3); \
  luaL_argcheck(L, V_MIN <= val && ((TYPE_NAME) val) <= V_MAX, 3, "wrong value");  \
  TYPE_NAME* lst = NULL;  \
  if (ptr->value > 0) { \
    if (0 < ind && ind <= ptr->value) { \
      lst = ptr->obj; \
    } \
  } else if (ptr->value == IDL_LUA_SEQ) { \
    rosidl_runtime_c__ ## STRUCT_NAME ## __Sequence* seq = ptr->obj; \
    if (0 < ind && ((size_t) ind) <= seq->size) { \
      lst = seq->data; \
    } \
  } else { \
    luaL_error(L, "not an array"); \
  } \
  if (lst) { \
    lst[ind-1] = val; \
  } \
  return 0; \
}

/**
 * Get value by index.
 *
 * \param STRUCT_NAME rosidl structure name
 * \param TYPE_NAME C data type name
 * \return getter function
 */
#define INT_SEQ_GET(STRUCT_NAME, TYPE_NAME) \
static int STRUCT_NAME ## _seq_get (lua_State* L) \
{ \
  idl_lua_msg_t* ptr = lua_touserdata(L, 1); \
  lua_Integer ind = luaL_checkinteger(L, 2); \
  TYPE_NAME* lst = NULL; \
  if (ptr->value > 0) { \
    if (0 < ind && ind <= ptr->value) { \
      lst = ptr->obj; \
    } \
  } else if (ptr->value == IDL_LUA_SEQ) { \
    rosidl_runtime_c__ ## STRUCT_NAME ## __Sequence* seq = ptr->obj; \
    if (0 < ind && ((size_t) ind) <= seq->size) { \
      lst = seq->data; \
    } \
  } else { \
    luaL_error(L, "not an array"); \
  } \
  if (lst) { \
    lua_pushinteger(L, lst[ind-1]); \
  } else { \
    lua_pushnil(L); \
  } \
  return 1; \
}

/**
 * Message caller.
 *
 * \param STRUCT_NAME rosidl structure name
 * \param TYPE_NAME C data type name
 * \param V_MIN lower limit
 * \param V_MAX upper limit
 * \return caller function
 */
#define INT_SEQ_CALL(STRUCT_NAME, TYPE_NAME, V_MIN, V_MAX) \
static int STRUCT_NAME ## _seq_call (lua_State* L) \
{ \
  bool done = false; \
  int tp = lua_type(L, 2); \
  if (LUA_TUSERDATA == tp) { \
    return STRUCT_NAME ## _seq_copy(L); \
  } else if (LUA_TNUMBER == tp) { \
    return STRUCT_NAME ## _seq_resize(L); \
  } else if (LUA_TTABLE == tp) { \
    lua_len(L, 2); \
    int len = luaL_checkinteger(L, -1); \
    idl_lua_msg_t* msg = lua_touserdata(L, 1); \
    if (len > 0 && (IDL_LUA_SEQ == msg->value || msg->value == len)) { \
      TYPE_NAME * lst = msg->obj; \
      if (IDL_LUA_SEQ == msg->value) { \
        lua_insert(L, 2); \
        STRUCT_NAME ## _seq_resize(L); \
        if (!lua_toboolean(L, -1)) { \
          return 1; \
        } \
        rosidl_runtime_c__ ## STRUCT_NAME ## __Sequence* seq = msg->obj; \
        lst = seq->data; \
        lua_remove(L, 2); \
      } \
      lua_pop(L, 1); \
      bool stop = false; \
      for (int i = 0; i < len; i++) { \
        lua_pushinteger(L, i+1); \
        lua_gettable(L, 2); \
        lua_Integer val = luaL_checkinteger(L, -1); \
        if (!(V_MIN <= val && ((TYPE_NAME) val) <= V_MAX)) { \
          stop = true; \
          break; \
        } \
        lst[i] = (TYPE_NAME) val; \
        lua_pop(L, 1); \
      } \
      done = !stop; \
    } \
  } \
  lua_pushboolean(L, done); \
  return 1; \
}

/** int8 sequence metatable name. */
const char* MT_SEQ_INT8 = "ROS2.rosidl_sequence.int8";

INT_SEQ_SET (int8, int8_t, INT8_MIN, INT8_MAX)
INT_SEQ_GET (int8, int8_t)
OBJ_SEQ_EQ (int8, MT_SEQ_INT8)
OBJ_SEQ_LEN (int8)
OBJ_SEQ_STR (int8)
OBJ_SEQ_COPY (int8, int8_t, MT_SEQ_INT8)
OBJ_SEQ_RESIZE (int8, MT_SEQ_INT8)
INT_SEQ_CALL (int8, int8_t, INT8_MIN, INT8_MAX)

OBJ_METHODS (int8, int8_seq_len)
OBJ_ADD_TABLE (int8, MT_SEQ_INT8)

/** uint8 sequence metatable name. */
const char* MT_SEQ_UINT8 = "ROS2.rosidl_sequence.uint8";

INT_SEQ_SET (uint8, uint8_t, 0, UINT8_MAX)
INT_SEQ_GET (uint8, uint8_t)
OBJ_SEQ_EQ (uint8, MT_SEQ_UINT8)
OBJ_SEQ_STR (uint8)
OBJ_SEQ_COPY (uint8, uint8_t, MT_SEQ_UINT8)
OBJ_SEQ_RESIZE (uint8, MT_SEQ_UINT8)
INT_SEQ_CALL (uint8, uint8_t, 0, UINT8_MAX)

OBJ_METHODS (uint8, int8_seq_len)
OBJ_ADD_TABLE (uint8, MT_SEQ_UINT8)

/** int16 sequence metatable name. */
const char* MT_SEQ_INT16 = "ROS2.rosidl_sequence.int16";

INT_SEQ_SET (int16, int16_t, INT16_MIN, INT16_MAX)
INT_SEQ_GET (int16, int16_t)
OBJ_SEQ_EQ (int16, MT_SEQ_INT16)
OBJ_SEQ_STR (int16)
OBJ_SEQ_COPY (int16, int16_t, MT_SEQ_INT16)
OBJ_SEQ_RESIZE (int16, MT_SEQ_INT16)
INT_SEQ_CALL (int16, int16_t, INT16_MIN, INT16_MAX)

OBJ_METHODS (int16, int8_seq_len)
OBJ_ADD_TABLE (int16, MT_SEQ_INT16)

/* uint16 sequence metatable name. */
const char* MT_SEQ_UINT16 = "ROS2.rosidl_sequence.uint16";

INT_SEQ_SET (uint16, uint16_t, 0, UINT16_MAX)
INT_SEQ_GET (uint16, uint16_t)
OBJ_SEQ_EQ (uint16, MT_SEQ_UINT16)
OBJ_SEQ_STR (uint16)
OBJ_SEQ_COPY (uint16, uint16_t, MT_SEQ_UINT16)
OBJ_SEQ_RESIZE (uint16, MT_SEQ_UINT16)
INT_SEQ_CALL (uint16, uint16_t, 0, UINT16_MAX)

OBJ_METHODS (uint16, int8_seq_len)
OBJ_ADD_TABLE (uint16, MT_SEQ_UINT16)

/* int32 sequence metatable name. */
const char* MT_SEQ_INT32 = "ROS2.rosidl_sequence.int32";

INT_SEQ_SET (int32, int32_t, INT32_MIN, INT32_MAX)
INT_SEQ_GET (int32, int32_t)
OBJ_SEQ_EQ (int32, MT_SEQ_INT32)
OBJ_SEQ_STR (int32)
OBJ_SEQ_COPY (int32, int32_t, MT_SEQ_INT32)
OBJ_SEQ_RESIZE (int32, MT_SEQ_INT32)
INT_SEQ_CALL (int32, int32_t, INT32_MIN, INT32_MAX)

OBJ_METHODS (int32, int8_seq_len)
OBJ_ADD_TABLE (int32, MT_SEQ_INT32)

/* uint32 sequence metatable name. */
const char* MT_SEQ_UINT32 = "ROS2.rosidl_sequence.uint32";

INT_SEQ_SET (uint32, uint32_t, 0, UINT32_MAX)
INT_SEQ_GET (uint32, uint32_t)
OBJ_SEQ_EQ (uint32, MT_SEQ_UINT32)
OBJ_SEQ_STR (uint32)
OBJ_SEQ_COPY (uint32, uint32_t, MT_SEQ_UINT32)
OBJ_SEQ_RESIZE (uint32, MT_SEQ_UINT32)
INT_SEQ_CALL (uint32, uint32_t, 0, UINT32_MAX)

OBJ_METHODS (uint32, int8_seq_len)
OBJ_ADD_TABLE (uint32, MT_SEQ_UINT32)

/* int64 sequence metatable name. */
const char* MT_SEQ_INT64 = "ROS2.rosidl_sequence.int64";

INT_SEQ_SET (int64, int64_t, INT64_MIN, INT64_MAX)
INT_SEQ_GET (int64, int64_t)
OBJ_SEQ_EQ (int64, MT_SEQ_INT64)
OBJ_SEQ_STR (int64)
OBJ_SEQ_COPY (int64, int64_t, MT_SEQ_INT64)
OBJ_SEQ_RESIZE (int64, MT_SEQ_INT64)
INT_SEQ_CALL (int64, int64_t, INT64_MIN, INT64_MAX)

OBJ_METHODS (int64, int8_seq_len)
OBJ_ADD_TABLE (int64, MT_SEQ_INT64)

/* uint64 sequence metatable name. */
const char* MT_SEQ_UINT64 = "ROS2.rosidl_sequence.uint64";

INT_SEQ_SET (uint64, uint64_t, 0, UINT64_MAX)
INT_SEQ_GET (uint64, uint64_t)
OBJ_SEQ_EQ (uint64, MT_SEQ_UINT64)
OBJ_SEQ_STR (uint64)
OBJ_SEQ_COPY (uint64, uint64_t, MT_SEQ_UINT64)
OBJ_SEQ_RESIZE (uint64, MT_SEQ_UINT64)
INT_SEQ_CALL (uint64, uint64_t, 0, UINT64_MAX)

OBJ_METHODS (uint64, int8_seq_len)
OBJ_ADD_TABLE (uint64, MT_SEQ_UINT64)
