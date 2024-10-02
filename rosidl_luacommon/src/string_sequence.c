
#include <rosidl_runtime_c/string.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>

#include "rosidl_luacommon/definition.h"
#include "sequence_macro.h"


const char* MT_SEQ_STRING = "ROS2.rosidl_sequence.String";

static int string_seq_set (lua_State* L)
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

static int string_seq_get (lua_State* L)
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
