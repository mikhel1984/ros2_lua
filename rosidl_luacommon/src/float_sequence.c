
#include <float.h>

#include <rosidl_runtime_c/primitives_sequence.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>

#include "rosidl_luacommon/definition.h"
#include "sequence_macro.h"

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
    rosidl_runtime_c__ ## STRUCT_NAME ## __Sequence* seq = ptr->obj; \
    luaL_argcheck(L, 0 < ind && ((size_t) ind) <= seq->size, 2, "out of range"); \
    lst = seq->data; \
  } else { \
    luaL_error(L, "unexpected object"); \
  } \
  lst[ind-1] = val; \
  return 0; \
}

#define FLOAT_SEQ_GET(STRUCT_NAME, TYPE_NAME) \
static int STRUCT_NAME ## _seq_get (lua_State* L) \
{ \
  idl_lua_msg_t* ptr = lua_touserdata(L, 1); \
  lua_Integer ind = luaL_checkinteger(L, 2); \
  TYPE_NAME* lst = NULL; \
  if (ptr->value > 0) { \
    luaL_argcheck(L, 0 < ind && ind <= ptr->value, 2, "out of range"); \
    lst = ptr->obj; \
  } else if (ptr->value == IDL_LUA_SEQ) { \
    rosidl_runtime_c__ ## STRUCT_NAME ## __Sequence* seq = ptr->obj; \
    luaL_argcheck(L, 0 < ind && ((size_t) ind) <= seq->size, 2, "out of range"); \
    lst = seq->data; \
  } else { \
    luaL_error(L, "unexpected object"); \
  } \
  lua_pushnumber(L, lst[ind-1]); \
  return 1; \
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

OBJ_METHODS(float, float_seq_len)
OBJ_ADD_TABLE (float, MT_SEQ_FLOAT)

/* double */

const char* MT_SEQ_DOUBLE = "ROS2.rosidl_sequence.double";

FLOAT_SEQ_SET (double, double, DBL_MIN, DBL_MAX)
FLOAT_SEQ_GET (double, double)
OBJ_SEQ_EQ (double, MT_SEQ_DOUBLE)
OBJ_SEQ_STR (double)
OBJ_SEQ_COPY (double, double, MT_SEQ_DOUBLE)
OBJ_SEQ_RESIZE (double, MT_SEQ_DOUBLE)

OBJ_METHODS(double, float_seq_len)
OBJ_ADD_TABLE (double, MT_SEQ_DOUBLE)

/* long double */

const char* MT_SEQ_LDOUBLE = "ROS2.rosidl_sequence.ldouble";

FLOAT_SEQ_SET (long_double, long double, LDBL_MIN, LDBL_MAX)
FLOAT_SEQ_GET (long_double, long double)
OBJ_SEQ_EQ (long_double, MT_SEQ_LDOUBLE)
OBJ_SEQ_STR (long_double)
OBJ_SEQ_COPY (long_double, long double, MT_SEQ_LDOUBLE)
OBJ_SEQ_RESIZE (long_double, MT_SEQ_LDOUBLE)

OBJ_METHODS(long_double, float_seq_len)
OBJ_ADD_TABLE (long_double, MT_SEQ_LDOUBLE)


