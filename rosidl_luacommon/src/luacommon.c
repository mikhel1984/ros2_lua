
#include "rosidl_luacommon/definition.h"

#include "float_sequence.h"

int luaopen_rosidl_luacommon (lua_State* L)
{
  lua_createtable(L, 0, 0);  // TODO set number
  
  rosidl_luacommon_add_float(L);
  
  return 1;  
}
