
#include <lua.h>
#include <lauxlib.h>
@{
from rosidl_cmake import convert_camel_case_to_lower_case_underscore as camel
}@
@# collect functions and namespaces

// prototypes

@[for srv in content]@
@{
req_type = srv.request_message.structure.namespaced_type
req_prefix = '__'.join(req_type.namespaces + [camel(req_type.name)])
resp_type = srv.response_message.structure.namespaced_type
resp_prefix = '__'.join(resp_type.namespaces + [camel(resp_type.name)])
}@
void @(req_prefix)__add_methods (lua_State* L);
void @(resp_prefix)__add_methods (lua_State* L);
@[end for]@

// library

int luaopen_msg (lua_State* L)
{
  lua_createtable(L, 0, 0);  
@[for message in content]@
@{
req_type = srv.request_message.structure.namespaced_type
req_prefix = '__'.join(req_type.namespaces + [camel(req_type.name)])
req_name = req_type.name.rsplit('_', 1)
resp_type = srv.response_message.structure.namespaced_type
resp_prefix = '__'.join(resp_type.namespaces + [camel(resp_type.name)])
resp_name = resp_type.name.rsplit('_', 1)
}@

  // @(req_name[0])
  lua_createtable(L, 0, 3);
  @(req_prefix)__add_methods(L);
  @(resp_prefix)__add_methods(L);
  lua_setfield(L, -2, @(req_name[0]))
@[end for]@

  return 1;
}

