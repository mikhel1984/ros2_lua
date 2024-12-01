
#include <lua.h>
#include <lauxlib.h>

#include <rosidl_luacommon/definition.h>
@{
from rosidl_generator_lua import make_prefix
from rosidl_parser.definition import AbstractNestedType, NamespacedType
import sys
}@
@# nested types
@{
nested_list = []
nested_list.append(("rosidl_luacommon", "sequence"))  # TODO check if required
for message in content:
    #print(str(message.structure.namespaced_type.namespaced_name()), file=sys.stderr)
    for member in message.structure.members:
        type_ = member.type
        if isinstance(type_, AbstractNestedType):
            type_ = type_.value_type
        if isinstance(type_, NamespacedType):
            #print(type_.namespaced_name(), file=sys.stderr)
            nested_type = tuple(map(str, type_.namespaced_name()[:2]))
            #nested_type = str(type_.namespaced_name()[0])
            if nested_type[0] != package_name and nested_type not in nested_list:
                nested_list.append(nested_type)
# print(nested_list, file=sys.stderr)
}@

// prototypes

@[for message in content]@
void @(make_prefix(message))__add_methods (lua_State* L);
@[end for]@

// library

int luaopen_@(package_name)_msg (lua_State* L)
{
@[for pair in nested_list]@
  ROSIDL_LUA_REQUIRE("@('.'.join(pair))");
@[end for]@

  lua_createtable(L, 0, @(len(content)));    // push table "msg"

@[for message in content]@
  @(make_prefix(message))__add_methods(L);
@[end for]@

  return 1;
}

