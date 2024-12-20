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

#ifndef ROSIDL_LUACOMMON__SEQUENCE_MACRO_H_
#define ROSIDL_LUACOMMON__SEQUENCE_MACRO_H_

/**
 * Check equality of two sequences.
 *
 * \param STRUCT_NAME rosidl structure name
 * \param METATABLE metatable name
 * \return function for equality checking
 */
#define OBJ_SEQ_EQ(STRUCT_NAME, METATABLE) \
static int STRUCT_NAME ## _seq_eq (lua_State* L) \
{ \
  idl_lua_msg_t* ptr1 = lua_touserdata(L, 1); \
  idl_lua_msg_t* ptr2 = luaL_checkudata(L, 2, METATABLE); \
  rosidl_runtime_c__ ## STRUCT_NAME ## __Sequence s1, s2; \
  if (ptr1->value == IDL_LUA_SEQ) { \
    rosidl_runtime_c__ ## STRUCT_NAME ## __Sequence* seq = ptr1->obj; \
    s1 = *seq; \
  } else if (ptr1->value > 0) { \
    s1.data = ptr1->obj; \
    s1.size = s1.capacity = ptr1->value; \
  } else { \
    luaL_error(L, "unexpected object"); \
  } \
  if (ptr2->value == IDL_LUA_SEQ) { \
    rosidl_runtime_c__ ## STRUCT_NAME ## __Sequence* seq = ptr2->obj; \
    s2 = *seq; \
  } else if (ptr2->value > 0) { \
    s2.data = ptr2->obj; \
    s2.size = s2.capacity = ptr2->value; \
  } else { \
    luaL_error(L, "unexpected object"); \
  } \
  lua_pushboolean(L, \
    rosidl_runtime_c__ ## STRUCT_NAME ## __Sequence__are_equal(&s1, &s2));  \
  return 1; \
}

/**
 * Get sequence length.
 *
 * \param STRUCT_NAME rosidl structure name
 * \return function for getting length.
 */
#define OBJ_SEQ_LEN(STRUCT_NAME) \
static int STRUCT_NAME ## _seq_len (lua_State* L) \
{ \
  idl_lua_msg_t* ptr = lua_touserdata(L, 1); \
  if (ptr->value > 0) { \
    lua_pushinteger(L, ptr->value); \
  } else if (ptr->value == IDL_LUA_SEQ) { \
    rosidl_runtime_c__ ## STRUCT_NAME ## __Sequence* seq = ptr->obj; \
    lua_pushinteger(L, seq->size); \
  } else { \
    lua_pushnil(L); \
  } \
  return 1; \
}

/**
 * Short string description.
 *
 * \param STRUCT_NAME rosidl structure name
 * \return function for getting string description.
 */
#define OBJ_SEQ_STR(STRUCT_NAME) \
static int STRUCT_NAME ## _seq_str (lua_State* L) \
{ \
  idl_lua_msg_t* ptr = lua_touserdata(L, 1); \
  if (ptr->value > 0) { \
    lua_pushfstring(L, "%s array of size %d", #STRUCT_NAME, ptr->value); \
  } else if (ptr->value == IDL_LUA_SEQ) { \
    rosidl_runtime_c__ ## STRUCT_NAME ##__Sequence* seq = ptr->obj; \
    lua_pushfstring(L, "%s sequence of size %d", #STRUCT_NAME, seq->size); \
  } else { \
    luaL_error(L, "unexpected object"); \
  } \
  return 1; \
}

/**
 * Make copy of sequence.
 *
 * \param STRUCT_NAME rosidl structure name
 * \param TYPE_NAME C data type name
 * \param METATABLE metatable name
 * \return function for making copy.
 */
#define OBJ_SEQ_COPY(STRUCT_NAME, TYPE_NAME, METATABLE) \
static int STRUCT_NAME ## _seq_copy (lua_State* L) \
{ \
  idl_lua_msg_t* dst = luaL_checkudata(L, 1, METATABLE); \
  idl_lua_msg_t* src = luaL_checkudata(L, 2, METATABLE); \
  bool done = false; \
  if (dst->value == IDL_LUA_SEQ) { \
    if (src->value == IDL_LUA_SEQ) { \
      done = rosidl_runtime_c__ ## STRUCT_NAME ##__Sequence__copy(src->obj, dst->obj); \
    } else if (src->value > 0) { \
      rosidl_runtime_c__ ## STRUCT_NAME ##__Sequence tmp; \
      tmp.data = src->obj; \
      tmp.size = tmp.capacity = (size_t) src->value; \
      done = rosidl_runtime_c__ ## STRUCT_NAME ## __Sequence__copy(&tmp, dst->obj); \
    } \
  } else if (dst->value > 0) { \
    TYPE_NAME *a = dst->obj, *b = NULL; \
    if (src->value > 0 && dst->value == src->value) { \
      b = src->obj; \
    } else if (src->value == IDL_LUA_SEQ) { \
      rosidl_runtime_c__ ## STRUCT_NAME ## __Sequence* seq = src->obj; \
      if (seq->size == (size_t) dst->value) { \
        b = seq->data; \
      } \
    } \
    if (b != NULL) { \
      for (int i = 0; i < dst->value; i++) { \
        *a++ = *b++; \
      } \
      done = true; \
    } \
  } \
  lua_pushboolean(L, done); \
  return 1; \
}

/**
 * Resize sequence.
 *
 * \param STRUCT_NAME rosidl structure name
 * \param METATABLE metatable name
 * \return function for changing size.
 */
#define OBJ_SEQ_RESIZE(STRUCT_NAME, METATABLE) \
static int STRUCT_NAME ## _seq_resize (lua_State* L) \
{ \
  idl_lua_msg_t* ptr = luaL_checkudata(L, 1, METATABLE); \
  if (ptr->value != IDL_LUA_SEQ) { \
    lua_pushboolean(L, false); \
    return 1; \
  } \
  lua_Integer len = luaL_checkinteger(L, 2); \
  luaL_argcheck(L, len >= 0, 2, "wrong length"); \
  rosidl_runtime_c__ ## STRUCT_NAME ## __Sequence* seq = ptr->obj; \
  bool done = true; \
  if (seq->capacity == 0) { \
    done = rosidl_runtime_c__ ## STRUCT_NAME ## __Sequence__init(seq, len); \
  } else if (seq->capacity >= (size_t) len) { \
    seq->size = (size_t) len; \
  } else { \
    rosidl_runtime_c__ ## STRUCT_NAME ## __Sequence newseq; \
    if (rosidl_runtime_c__ ## STRUCT_NAME ## __Sequence__init(&newseq, len) && \
        rosidl_runtime_c__ ## STRUCT_NAME ## __Sequence__copy(seq, &newseq)) \
    { \
      rosidl_runtime_c__ ## STRUCT_NAME ## __Sequence tmp = *seq; \
      *seq = newseq; \
      rosidl_runtime_c__ ## STRUCT_NAME ## __Sequence__fini(&tmp);  \
      seq->size = len; \
    } else { \
      done = false; \
    } \
  } \
  lua_pushboolean(L, done); \
  return 1; \
}

/**
 * Prepare list of metamethods.
 *
 * \param STRUCT_NAME rosidl structure name
 * \param LEN_METHOD length getter name
 * \return array of metamethods.
 */
#define OBJ_METHODS(STRUCT_NAME, LEN_METHOD) \
static const struct luaL_Reg STRUCT_NAME ## _seq_methods[] = {\
  {"__index", STRUCT_NAME ## _seq_get}, \
  {"__newindex", STRUCT_NAME ## _seq_set}, \
  {"__eq", STRUCT_NAME ## _seq_eq}, \
  {"__len", LEN_METHOD}, \
  {"__tostring", STRUCT_NAME ## _seq_str}, \
  {"__call", STRUCT_NAME ## _seq_call}, \
  {"copy", STRUCT_NAME ## _seq_copy}, \
  {"resize", STRUCT_NAME ## _seq_resize}, \
  {NULL, NULL} \
};

/**
 * Defin metatable, fill with methods.
 *
 * \param STRUCT_NAME rosidl structure name
 * \param METATABLE metatable name
 * \return function for saving metamethods to library.
 */

#define OBJ_ADD_TABLE(STRUCT_NAME, METATABLE) \
void rosidl_luacommon_add_ ## STRUCT_NAME (lua_State* L) \
{ \
  luaL_newmetatable(L, METATABLE); \
  luaL_setfuncs(L, STRUCT_NAME ## _seq_methods , 0); \
  lua_pop(L, 1); \
}

#endif  // ROSIDL_LUACOMMON__SEQUENCE_MACRO_H_
