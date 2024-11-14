
#include <lua.h>
#include <lauxlib.h>
@{
from rosidl_cmake import convert_camel_case_to_lower_case_underscore
from rosidl_parser.definition import AbstractNestedType, NamespacedType
import sys
}@
@# generate function names
@{
function_list = []
for message in content:
    msg_prefix = '__'.join(message.structure.namespaced_type.namespaces + [convert_camel_case_to_lower_case_underscore(message.structure.namespaced_type.name)])
    function_list.append(msg_prefix + '__add_methods')    
}@
@# nested types
@{
nested_list = []
for message in content:
    print(str(message.structure.namespaced_type.namespaced_name()), file=sys.stderr)
    for member in message.structure.members:
        type_ = member.type
        if isinstance(type_, AbstractNestedType):
            type_ = type_.value_type
        if isinstance(type_, NamespacedType):
            print(type_.namespaced_name(), file=sys.stderr)
            nested_type = str(type_.namespaced_name()[0])
            if nested_type not in nested_list:
                nested_list.append(nested_type)
}@
// @(str(nested_list))

// prototypes

@[for function in function_list]@
void @(function) (lua_State* L);
@[end for]@

// library

int luaopen_msg (lua_State* L)
{
  lua_createtable(L, 0, 0);

@[for function in function_list]@
  @(function)(L);
@[end for]@

  return 1;
}

