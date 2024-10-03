
#include <rosidl_runtime_c/string.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>

#include "rosidl_luacommon/definition.h"
#include "sequence_macro.h"


const char* MT_SEQ_STRING = "ROS2.rosidl_sequence.String";

static int String_seq_set (lua_State* L)
{
  idl_lua_msg_t* ptr = lua_touserdata(L, 1);
  lua_Integer ind = luaL_checkinteger(L, 2);
  if (!lua_isstring(L, 3)) {
    luaL_error(L, "string is expected");
  }
  size_t len = 0;
  const char* val = lua_tolstring(L, 3, &len);

  rosidl_runtime_c__String* lst = NULL;

  if (ptr->value > 0) {
    luaL_argcheck(L, 0 < ind && ind <= ptr->value, 2, "out of range");
    lst = ptr->obj;
  } else if (ptr->value == IDL_LUA_SEQ) {
    rosidl_runtime_c__String__Sequence* seq = ptr->obj;
    luaL_argcheck(L, 0 < ind && ((size_t) ind) <= seq->size, 2, "out of range");
    lst = seq->data;
  } else {
    luaL_error(L, "unexpected object");
  }
  rosidl_runtime_c__String__assignn(lst + (ind-1), val, len);

  return 0;
}

static int String_seq_get (lua_State* L)
{
  idl_lua_msg_t* ptr = lua_touserdata(L, 1);
  lua_Integer ind = luaL_checkinteger(L, 2);

  rosidl_runtime_c__String* lst = NULL;
  if (ptr->value > 0) {
    luaL_argcheck(L, 0 < ind && ind <= ptr->value, 2, "out of range");
    lst = ptr->obj;
  } else if (ptr->value == IDL_LUA_SEQ) {
    rosidl_runtime_c__String__Sequence* seq = ptr->obj;
    luaL_argcheck(L, 0 < ind && ((size_t) ind) <= seq->size, 2, "out of range");
    lst = seq->data;
  } else {
    luaL_error(L, "unexpected object");
  }
  rosidl_runtime_c__String* s = lst + (ind-1);
  lua_pushlstring(L, s->data, s->size+1);

  return 1;
}

static int String_seq_copy (lua_State* L)
{
  idl_lua_msg_t* dst = luaL_checkudata(L, 1, MT_SEQ_STRING);
  idl_lua_msg_t* src = luaL_checkudata(L, 2, MT_SEQ_STRING);
  bool done = false;

  if (dst->value == IDL_LUA_SEQ) {
    if (src->value == IDL_LUA_SEQ) {
      done = rosidl_runtime_c__String__Sequence__copy(src->obj, dst->obj);
    } else if (src->value > 0) {
      rosidl_runtime_c__String__Sequence tmp;
      tmp.data = src->obj;
      tmp.size = tmp.capacity = (size_t) src->value;
      done = rosidl_runtime_c__String__Sequence__copy(&tmp, dst->obj);
    }
  } else if (dst->value > 0) {
    rosidl_runtime_c__String *a = dst->obj, *b = NULL;
    if (src->value > 0 && dst->value == src->value) {
      b = src->obj;
    } else if (src->value == IDL_LUA_SEQ) {
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

OBJ_SEQ_EQ (String, MT_SEQ_STRING)
OBJ_SEQ_LEN (String)
OBJ_SEQ_STR (String)
OBJ_SEQ_RESIZE (String, MT_SEQ_STRING)

OBJ_METHODS (String, String_seq_len)
OBJ_ADD_TABLE (String, MT_SEQ_STRING)

