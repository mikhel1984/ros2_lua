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

#include <float.h>

#include <rosidl_runtime_c/primitives_sequence.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>

#include "rosidl_luacommon/definition.h"
#include "rosidl_luacommon/sequence_macro.h"

/**
 * Set value by index.
 *
 * \param STRUCT_NAME rosidl structure name
 * \param TYPE_NAME C data type name
 * \param V_MAX upper limit
 * \return setter function
 */
#define FLOAT_SEQ_SET(STRUCT_NAME, TYPE_NAME, V_MAX) \
static int STRUCT_NAME ## _seq_set (lua_State* L) \
{ \
  idl_lua_msg_t* ptr = lua_touserdata(L, 1); \
  lua_Integer ind = luaL_checkinteger(L, 2); \
  lua_Number val = luaL_checknumber(L, 3); \
  luaL_argcheck(L, (-V_MAX) <= val && val <= V_MAX, 3, "wrong value");  \
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
#define FLOAT_SEQ_GET(STRUCT_NAME, TYPE_NAME) \
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
    lua_pushnumber(L, lst[ind-1]); \
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
 * \param V_MAX upper limit
 * \return caller function
 */
#define FLOAT_SEQ_CALL(STRUCT_NAME, TYPE_NAME, V_MAX) \
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
        if (LUA_TNUMBER != lua_type(L, -1)) { \
          stop = true; \
          break; \
        } \
        lua_Number val = lua_tonumber(L, -1); \
        if (!((-V_MAX) <= val && val <= V_MAX)) { \
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

/** Float sequence metatable name. */
const char* MT_SEQ_FLOAT = "ROS2.rosidl_sequence.float";

FLOAT_SEQ_SET (float, float, FLT_MAX)
FLOAT_SEQ_GET (float, float)
OBJ_SEQ_EQ (float, MT_SEQ_FLOAT)
OBJ_SEQ_LEN (float)
OBJ_SEQ_STR (float)
OBJ_SEQ_COPY (float, float, MT_SEQ_FLOAT)
OBJ_SEQ_RESIZE (float, MT_SEQ_FLOAT)
FLOAT_SEQ_CALL (float, float, FLT_MAX)

OBJ_METHODS(float, float_seq_len)
OBJ_ADD_TABLE (float, MT_SEQ_FLOAT)

/** Double sequence metatable name. */
const char* MT_SEQ_DOUBLE = "ROS2.rosidl_sequence.double";

FLOAT_SEQ_SET (double, double, DBL_MAX)
FLOAT_SEQ_GET (double, double)
OBJ_SEQ_EQ (double, MT_SEQ_DOUBLE)
OBJ_SEQ_STR (double)
OBJ_SEQ_COPY (double, double, MT_SEQ_DOUBLE)
OBJ_SEQ_RESIZE (double, MT_SEQ_DOUBLE)
FLOAT_SEQ_CALL (double, double, DBL_MAX)

OBJ_METHODS(double, float_seq_len)
OBJ_ADD_TABLE (double, MT_SEQ_DOUBLE)

/** Long double metatable name. */
const char* MT_SEQ_LDOUBLE = "ROS2.rosidl_sequence.ldouble";

FLOAT_SEQ_SET (long_double, long double, LDBL_MAX)
FLOAT_SEQ_GET (long_double, long double)
OBJ_SEQ_EQ (long_double, MT_SEQ_LDOUBLE)
OBJ_SEQ_STR (long_double)
OBJ_SEQ_COPY (long_double, long double, MT_SEQ_LDOUBLE)
OBJ_SEQ_RESIZE (long_double, MT_SEQ_LDOUBLE)
FLOAT_SEQ_CALL (long_double, long double, LDBL_MAX)

OBJ_METHODS(long_double, float_seq_len)
OBJ_ADD_TABLE (long_double, MT_SEQ_LDOUBLE)
