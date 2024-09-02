#ifndef RCL_LUA_CONTEXT_H
#define RCL_LUA_CONTEXT_H

#include <lua.h>

#include <rcl/context.h>

/* Save methods into the table on the top of the stack */
void rcl_lua_add_context_methods(lua_State* L);

rcl_context_t* rcl_lua_context_ref();

#endif  // RCL_LUA_CONTEXT_H

