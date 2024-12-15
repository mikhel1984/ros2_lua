
#include <lua.h>
#include <lauxlib.h>

#include <rosidl_luacommon/definition.h>

@# add headers
@{
from rosidl_generator_lua import make_include_prefix
}@
@[for srv in content]@
#include "@(make_include_prefix(srv))__type_support.h"
@[end for]@

@{
from rosidl_generator_lua import make_prefix
from rosidl_parser.definition import AbstractNestedType, NamespacedType
import sys
}@
@# collect functions and namespaces
@# nested types
@{
#import sys
nested_list = []
nested_list.append(("rosidl_luacommon", "sequence"))  # TODO check if required
for srv in content:
    for message in (srv.request_message, srv.response_message):
        #print(str(message.structure.namespaced_type.namespaced_name()), file=sys.stderr)
        for member in message.structure.members:
            type_ = member.type
            if isinstance(type_, AbstractNestedType):
                type_ = type_.value_type
            if isinstance(type_, NamespacedType):
                #print(type_.namespaced_name(), file=sys.stderr)
                nested_type = tuple(map(str, type_.namespaced_name()[:2]))
                #nested_type = str(type_.namespaced_name()[0])
                if nested_type not in nested_list:
                    nested_list.append(nested_type)
#print(nested_list, file=sys.stderr)
}@

// prototypes

@[for srv in content]@
void @(make_prefix(srv.request_message))__add_methods (lua_State* L);
void @(make_prefix(srv.response_message))__add_methods (lua_State* L);
@[end for]@

// library

int luaopen_@(package_name)_srv (lua_State* L)
{
@[for pair in nested_list]@
  ROSIDL_LUA_REQUIRE("@('.'.join(pair))");
@[end for]@
  
  const rosidl_service_type_support_t *ts;
  lua_createtable(L, 0, @(len(content)));   // push table "srv"
@[for srv in content]@
@{
req_name = srv.request_message.structure.namespaced_type.name.rsplit('_', 1)
}@
  // open "namespace" @(req_name[0])
  lua_createtable(L, 0, 2);    // push table
  @(make_prefix(srv.request_message))__add_methods(L);
  @(make_prefix(srv.response_message))__add_methods(L);
  
  // add type support  
  ts = ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, @(', '.join(srv.namespaced_type.namespaced_name())))();
  lua_pushlightuserdata(L, (void*) ts);
  lua_setfield(L, -2, "_type_support");
  
  // close "namespace" @(req_name[0])
  lua_setfield(L, -2, "@(req_name[0])");   // pop table
@[end for]@

  return 1;
}

