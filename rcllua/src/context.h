#ifndef RCL_LUA_CONTEXT_H
#define RCL_LUA_CONTEXT_H

#include <lua.h>

/* Save methods into the table on the top of the stack */
void rcl_lua_add_context_methods(lua_State* L);

#endif  // RCL_LUA_CONTEXT_H

