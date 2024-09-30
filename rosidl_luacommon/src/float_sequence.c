
#include <limits.h>
#include <rosidl_runtime_c/primitive_sequences.h>

#include "rosidl_luacommon/definition.h"

const char* MT_SEQ_FLOAT = "ROS2.rosidl_sequence_float";

static int float_seq_set (lua_State* L)
{
  float* lst = NULL;
  idl_lua_msg_t* ptr = lua_touserdata(L, 1);
  lua_Integer ind = luaL_checkinteger(L, 2);
  lua_Number val = luaL_checknumber(L, 3);
  luaL_argcheck(L, FLT_MIN <= val && val <= FLT_MAX, 3, "wrong value");  
  
  if (ptr->value > 0) {
    luaL_argcheck(L, 0 < ind && ind <= ptr->value, 2, "out of range");
    lst = ptr->obj;
  } else if (ptr->value == IDL_LUA_SEQ) {
    rosidl_runtime_c__float__Sequence* seq = ptr->obj;
    luaL_argcheck(L, 0 < ind && ind <= seq->size, 2, "out of range");
    lst = seq->data;
  } else {
    luaL_error(L, "unexpected object");
  }
  
  lst[ind-1] = val;  
  
  return 0;
}

static int float_seq_get (lua_State* L)
{
  idl_lua_msg_t* ptr = lua_touserdata(L, 1);
  lua_Integer ind = luaL_checkinteger(L, 2);
  float* lst = NULL;
  
  if (ptr->value > 0) {
    luaL_argcheck(L, 0 < ind && ind <= ptr->value, 2, "out of range");
    lst = ptr->obj;
  } else if (ptr->value == IDL_LUA_SEQ) {
    rosidl_runtime_c__float__Sequence* seq = ptr->obj;
    luaL_argcheck(L, 0 < ind && ind <= seq->size, 2, "out of range");
    lst = seq->data;
  } else {
    luaL_error(L, "unexpected object");
  }
  
  lua_pushnumber(L, lst[ind-1]);  
  
  return 1;
}

static int float_seq_eq (lua_State* L)
{
  idl_lua_msg_t* ptr1 = lua_touserdata(L, 1);
  idl_lua_msg_t* ptr2 = luaL_checkudata(L, 2, MT_SEQ_FLOAT);
  
  rosidl_runtime_c__float__Sequence s1, s2;
  if (ptr1->value == IDL_LUA_SEQ) {
    s1 = *ptr1;
  } else if (ptr1->value > 0) {
    s1.data = ptr1->obj;
    s1.size = s1.capacity = ptr1->value;
  } else {
    luaL_error(L, "unexpected object");
  }
  if (ptr2->value == IDL_LUA_SEQ) {
    s2 = *ptr2;
  } else if (ptr2->value > 0) {
    s2.data = ptr2->obj;
    s2.size = s2.capacity = ptr2->value;
  } else {
    luaL_error(L, "unexpected object");
  }
  
  lua_pushboolean(L, rosidl_runtime_c__float__Sequence__are_equal(&s1, &s2));  
  
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
  // dst
  idl_lua_msg_t* ptr1 = luaL_checkudata(L, 1, MT_SEQ_FLOAT);
  // src
  idl_lua_msg_t* ptr2 = luaL_checkudata(L, 2, MT_SEQ_FLOAT);
  bool done = true;
  
  if (ptr1->value > 0 && ptr2->value > 0) {
    if (ptr1->value == ptr2->value) {
      float *a = ptr1->obj, *b = ptr2->obj;
      for (int i = 0; i < ptr1->value; i++) {
        *a++ = *b++;        
      }
    } else {
      done = false;
    }
  } else if (ptr1->value == IDL_LUA_SEQ && ptr2->value == IDL_LUA_SEQ) {
    done = rosidl_runtime_c__float__Sequence__copy(ptr2->obj, ptr1->obj);
  } else if (ptr2->value == IDL_LUA_SEQ && ptr1->value > 0) {
    rosidl_runtime_c__float__Sequence* seq = ptr2->obj;
    if (seq->size == ptr1->value) {
      float *a = ptr1->obj, *b = seq->data;
      for (int i = 0; i < ptr1->value; i++) {
        *a++ = *b++;
      }
    } else {
      done = false;
    }    
  } else if (ptr1->value == IDL_LUA_SEQ && ptr2->value > 0) {
    rosidl_runtime_c__float__Sequence tmp;
    tmp.data = ptr2->obj;
    tmp.size = tmp.capacity = (size_t) ptr2->value;
    done = rosidl_runtime_c__float__Sequence__copy(&tmp, ptr1->obj);
  }
  lua_pushboolean(L, done);
  
  return 1;
}

static int float_seq_resize (lua_State* L)
{
  idl_lua_msg_t* ptr = luaL_checkudata(L, 1, MT_SEQ_FLOAT); 
  if (ptr->value != IDL_LUA_SEQ) {
    lua_pushboolean(L, false);
    return 1;
  }
  
  lua_Integer len = luaL_checkinteger(L, 2);
  luaL_argcheck(L, len >= 0, 2, "wrong length");
  
  rosidl_runtime_c__float__Sequence* seq = ptr->obj;  
  bool done = true;
  if (seq->size >= len) {
    seq->size = (size_t) len;
  } else {
    rosidl_runtime_c__float__Sequence newseq;
    if (rosidl_runtime_c__float__Sequence__init(&newseq, len) && 
        rosidl_runtime_c__float__Sequence__copy(seq, &newsec)) 
    {
      rosidl_runtime_c__float__Sequence tmp = *seq;  // initial sequence
      *seq = newseq;  // save new values
      rosidl_runtime_c__float__Sequence__fini(&tmp);    
    } else {
      done = false;
    }
  }
  
  lua_pushboolean(L, done); 
  
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
