
#include <float.h>

#include <rosidl_runtime_c/primitives_sequence.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>

#include "rosidl_luacommon/definition.h"

#define FLOAT_SEQ_SET(STRUCT_NAME, TYPE_NAME, V_MIN, V_MAX) \
static int STRUCT_NAME ## _seq_set (lua_State* L) \
{ \
  TYPE_NAME* lst = NULL;  \
  idl_lua_msg_t* ptr = lua_touserdata(L, 1); \
  lua_Integer ind = luaL_checkinteger(L, 2); \
  lua_Number val = luaL_checknumber(L, 3); \
  luaL_argcheck(L, V_MIN <= val && val <= V_MAX, 3, "wrong value");  \
  if (ptr->value > 0) { \
    luaL_argcheck(L, 0 < ind && ind <= ptr->value, 2, "out of range"); \
    lst = ptr->obj; \
  } else if (ptr->value == IDL_LUA_SEQ) { \
    rosidl_runtime_c__ ## TYPE_NAME ## __Sequence* seq = ptr->obj; \
    luaL_argcheck(L, 0 < ind && ((size_t) ind) <= seq->size, 2, "out of range"); \
    lst = seq->data; \
  } else { \
    luaL_error(L, "unexpected object"); \
  } \
  lst[ind-1] = val; \
  return 0; \
}

#define FLOAT_SEQ_GET(STRUCT_NAME, TYPE_NAME) \
static int TYPE_NAME ## _seq_get (lua_State* L) \
{ \
  idl_lua_msg_t* ptr = lua_touserdata(L, 1); \
  lua_Integer ind = luaL_checkinteger(L, 2); \
  TYPE_NAME* lst = NULL; \
  if (ptr->value > 0) { \
    luaL_argcheck(L, 0 < ind && ind <= ptr->value, 2, "out of range"); \
    lst = ptr->obj; \
  } else if (ptr->value == IDL_LUA_SEQ) { \
    rosidl_runtime_c__ ## TYPE_NAME ## __Sequence* seq = ptr->obj; \
    luaL_argcheck(L, 0 < ind && ((size_t) ind) <= seq->size, 2, "out of range"); \
    lst = seq->data; \
  } else { \
    luaL_error(L, "unexpected object"); \
  } \
  lua_pushnumber(L, lst[ind-1]); \
  return 1; \
}

#define OBJ_SEQ_EQ(STRUCT_NAME, METATABLE) \
static int STRUCT_NAME ## _seq_eq (lua_State* L) \
{ \
  idl_lua_msg_t* ptr1 = lua_touserdata(L, 1); \
  idl_lua_msg_t* ptr2 = luaL_checkudata(L, 2, METATABLE); \
  rosidl_runtime_c__ ## STRUCT_NAME ## __Sequence s1, s2; \
  if (ptr1->value == IDL_LUA_SEQ) { \
    s1 = *((rosidl_runtime_c__ ## STRUCT_NAME ## __Sequence*) ptr1->obj); \
  } else if (ptr1->value > 0) { \
    s1.data = ptr1->obj; \
    s1.size = s1.capacity = ptr1->value; \
  } else { \
    luaL_error(L, "unexpected object"); \
  } \
  if (ptr2->value == IDL_LUA_SEQ) { \
    s2 = *((rosidl_runtime_c__ ## STRUCT_NAME ## __Sequence*) ptr2); \
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

#define OBJ_SEQ_LEN(STRUCT_NAME) \
static int STRUCT_NAME ## _seq_len (lua_State* L) \
{ \
  idl_lua_msg_t* ptr = lua_touserdata(L, 1); \
  if (ptr->value > 0) { \
    lua_pushnumber(L, ptr->value); \
  } else if (ptr->value == IDL_LUA_SEQ) { \
    rosidl_runtime_c__ ## STRUCT_NAME ## __Sequence* seq = ptr->obj; \
    lua_pushnumber(L, seq->size); \
  } else { \
    lua_pushnil(L); \
  } \
  return 1; \
}

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

#define OBJ_SEQ_COPY(STRUCT_NAME, TYPE_NAME, METATABLE) \
static int STRUCT_NAME ## _seq_copy (lua_State* L) \
{ \
  idl_lua_msg_t* dst = luaL_checkudata(L, 1, METATABLE); \
  idl_lua_msg_t* src = luaL_checkudata(L, 2, MT_SEQ_FLOAT); \
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
    } else { \
      done = false; \
    } \
  } \
  lua_pushboolean(L, done); \
  return 1; \
}

#define OBJ_METHODS(STRUCT_NAME, LEN_METHOD) {\
  {"__index", STRUCT_NAME ## _seq_get}, \
  {"__newindex", STRUCT_NAME ## _seq_set}, \
  {"__eq", STRUCT_NAME ## _seq_eq}, \
  {"__len", LEN_METHOD}, \
  {"__tostring", STRUCT_NAME ## _seq_str}, \
  {"copy", STRUCT_NAME ## _seq_copy}, \
  {"resize", STRUCT_NAME ## _seq_resize}, \
  {NULL, NULL} \
}

#define OBJ_ADD_TABLE(STRUCT_NAME, METATABLE, METHODS) \
void rosidl_luacommon_add_ ## STRUCT_NAME (lua_State* L) \
{ \
  luaL_newmetatable(L, METATABLE); \
  luaL_setfuncs(L, METHODS, 0); \
  lua_pop(L, 1); \
}

/* float */

const char* MT_SEQ_FLOAT = "ROS2.rosidl_sequence.float";

FLOAT_SEQ_SET (float, float, FLT_MIN, FLT_MAX)
FLOAT_SEQ_GET (float, float)
OBJ_SEQ_EQ (float, MT_SEQ_FLOAT)
OBJ_SEQ_LEN (float)
OBJ_SEQ_STR (float)
OBJ_SEQ_COPY (float, float, MT_SEQ_FLOAT)
OBJ_SEQ_RESIZE (float, MT_SEQ_FLOAT)

static const struct luaL_Reg float_seq_methods[] = OBJ_METHODS(float, float_seq_len);

OBJ_ADD_TABLE (float, MT_SEQ_FLOAT, float_seq_methods)

/* double */

const char* MT_SEQ_DOUBLE = "ROS2.rosidl_sequence.double";

FLOAT_SEQ_SET (double, double, DBL_MIN, DBL_MAX)
FLOAT_SEQ_GET (double, double)
OBJ_SEQ_EQ (double, MT_SEQ_DOUBLE)
OBJ_SEQ_STR (double)
OBJ_SEQ_COPY (double, double, MT_SEQ_DOUBLE)
OBJ_SEQ_RESIZE (double, MT_SEQ_DOUBLE)

static const struct luaL_Reg double_seq_methods[] = OBJ_METHODS(double, float_seq_len);

OBJ_ADD_TABLE (double, MT_SEQ_DOUBLE, double_seq_methods)


