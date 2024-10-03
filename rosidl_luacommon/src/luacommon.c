
#include "rosidl_luacommon/definition.h"

/* prototipes */

void rosidl_luacommon_add_float (lua_State* L);
void rosidl_luacommon_add_double (lua_State* L);
void rosidl_luacommon_add_long_double (lua_State* L);

void rosidl_luacommon_add_int8 (lua_State* L);
void rosidl_luacommon_add_uint8 (lua_State* L);
void rosidl_luacommon_add_int16 (lua_State* L);
void rosidl_luacommon_add_uint16 (lua_State* L);
void rosidl_luacommon_add_int32 (lua_State* L);
void rosidl_luacommon_add_uint32 (lua_State* L);
void rosidl_luacommon_add_int64 (lua_State* L);
void rosidl_luacommon_add_uint64 (lua_State* L);

void rosidl_luacommon_add_boolean (lua_State* L);
void rosidl_luacommon_add_String (lua_State* L);


/* library */

int luaopen_rosidl_luacommon (lua_State* L)
{
  lua_createtable(L, 0, 0);  // TODO remove ?
  
  /* prepare metatables */
  
  rosidl_luacommon_add_float(L);
  rosidl_luacommon_add_double(L);
  rosidl_luacommon_add_long_double(L);
  
  rosidl_luacommon_add_int8(L);
  rosidl_luacommon_add_uint8(L);
  rosidl_luacommon_add_int16(L);
  rosidl_luacommon_add_uint16(L);
  rosidl_luacommon_add_int32(L);
  rosidl_luacommon_add_uint32(L);
  rosidl_luacommon_add_int64(L);
  rosidl_luacommon_add_uint64(L);  
  
  rosidl_luacommon_add_boolean(L);
  rosidl_luacommon_add_String(L);
  
  return 1;  
}
