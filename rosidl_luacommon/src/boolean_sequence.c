
#include <rosidl_runtime_c/primitives_sequence.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>

#include "rosidl_luacommon/definition.h"
#include "sequence_macro.h"


const char* MT_SEQ_BOOLEAN = "ROS2.rosidl_sequence.boolean";

static int boolean_seq_set (lua_State* L)
{
  bool* lst = NULL;
  idl_lua_msg_t* ptr = lua_touserdata(L, 1);
  lua_Integer ind = luaL_checkinteger(L, 2);
  if (!lua_isboolean(L, 3)) {
    luaL_error(L, "boolean expected");
  }
  bool val = lua_toboolean(L, 3);

  if (ptr->value > 0) {
    luaL_argcheck(L, 0 < ind && ind <= ptr->value, 2, "out of range");
    lst = ptr->obj;
  } else if (ptr->value == IDL_LUA_SEQ) {
    rosidl_runtime_c__boolean__Sequence* seq = ptr->obj;
    luaL_argcheck(L, 0 < ind && ((size_t) ind) <= seq->size, 2, "out of range");
    lst = seq->data;
  } else {
    luaL_error(L, "unexpected object");
  }
  lst[ind-1] = val;
  
  return 0;
}

static int boolean_seq_get (lua_State* L)
{
  idl_lua_msg_t* ptr = lua_touserdata(L, 1);
  lua_Integer ind = luaL_checkinteger(L, 2);
  
  bool* lst = NULL;
  if (ptr->value > 0) {
    luaL_argcheck(L, 0 < ind && ind <= ptr->value, 2, "out of range");
    lst = ptr->obj;
  } else if (ptr->value == IDL_LUA_SEQ) {
    rosidl_runtime_c__boolean__Sequence* seq = ptr->obj;
    luaL_argcheck(L, 0 < ind && ((size_t) ind) <= seq->size, 2, "out of range");
    lst = seq->data;
  } else {
    luaL_error(L, "unexpected object");
  }
  lua_pushboolean(L, lst[ind-1]);
  
  return 1;
}

OBJ_SEQ_EQ (boolean, MT_SEQ_BOOLEAN)
OBJ_SEQ_LEN (boolean)
OBJ_SEQ_STR (boolean)
OBJ_SEQ_COPY (boolean, bool, MT_SEQ_BOOLEAN)
OBJ_SEQ_RESIZE (boolean, MT_SEQ_BOOLEAN)

OBJ_METHODS(boolean, boolean_seq_len)
OBJ_ADD_TABLE (boolean, MT_SEQ_BOOLEAN)

