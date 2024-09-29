
#include <rosidl_runtime_c/primitive_sequences.h>

#include "rosidl_luacommon/definition.h"

const char* MT_SEQ_FLOAT = "ROS2.rosidl_sequence_float";

static int float_seq_set (lua_State* L)
{
  return 0;
}

static int float_seq_get (lua_State* L)
{
  return 1;
}

static int float_seq_eq (lua_State* L)
{
  return 1;
}

static int float_seq_len (lua_State* L)
{
  idl_lua_msg_t* ptr = lua_touserdata(L, 1);
  if (ptr->value > 0) {
    lua_pushnumber(L, ptr->value);
  } else if (ptr->value == IDL_LUA_SEQ) {
    rosidl_runtime_c__float__Sequence* seq = ptr->obj;
    lua_pushnumber(L, seq->size);
  } else {
    lua_pushnil(L);
  }  
  
  return 1;
}

static int float_seq_str (lua_State* L)
{
  idl_lua_msg_t* ptr = lua_touserdata(L, 1);
  if (ptr->value > 0) {
    lua_pushfstring(L, "float array of size %d", ptr->value);
  } else if (ptr->value == IDL_LUA_SEQ) {
    rosidl_runtime_c__float__Sequence* seq = ptr->obj;
    lua_pushfstring(L, "float sequence of size %d", seq->size);    
  } else {
    luaL_error(L, "unexpected object");
  } 
  
  return 1;
}

static int float_seq_copy (lua_State* L)
{
  return 1;
}

static int float_seq_resize (lua_State* L)
{
  return 1;
}


static const struct luaL_Reg float_seq_methods[] = {
  {"__index", float_seq_get},
  {"__newindex", float_seq_set},
  {"__eq", float_seq_eq},
  {"__len", float_seq_len},
  {"__tostring", float_seq_str},
  {"copy", float_seq_copy},
  {"resize", float_seq_resize},
  {NULL, NULL}  
};

void rosidl_luacommon_float (lua_State* L)
{
  luaL_newmetatable(L, MT_SEQ_FLOAT);
  luaL_setfuncs(L, float_seq_methods, 0);
  lua_pop(L, 1);  
}
