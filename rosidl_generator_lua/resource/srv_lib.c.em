
#include <lua.h>
#include <lauxlib.h>
@{
from rosidl_generator_lua import make_prefix
}@
@# collect functions and namespaces

// prototypes

@[for srv in content]@
void @(make_prefix(srv.request_message))__add_methods (lua_State* L);
void @(make_prefix(srv.response_message))__add_methods (lua_State* L);
@[end for]@

// library

int luaopen_srv (lua_State* L)
{
  lua_createtable(L, 0, @(len(content)));   // push table "srv"
@[for message in content]@
@{
req_name = srv.request_message.structure.namespaced_type.name.rsplit('_', 1)
}@

  // open "namespace" @(req_name[0])
  lua_createtable(L, 0, 2);    // push table
  @(make_prefix(srv.request_message))__add_methods(L);
  @(make_prefix(srv.response_message))__add_methods(L);
  // close "namespace" @(req_name[0])
  lua_setfield(L, -2, "@(req_name[0])");   // pop table
@[end for]@

  return 1;
}

