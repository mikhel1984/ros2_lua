
#include <lua.h>
#include <lauxlib.h>
@{
from rosidl_generator_lua import make_prefix
}@
@# collect functions and namespaces

// prototypes

@[for act in content]@
void @(make_prefix(act.goal))__add_methods (lua_State* L);
void @(make_prefix(act.result))__add_methods (lua_State* L);
void @(make_prefix(act.feedback))__add_methods (lua_State* L);
void @(make_prefix(act.send_goal_service.request_message))__add_methods (lua_State* L);
void @(make_prefix(act.send_goal_service.response_message))__add_methods (lua_State* L);
void @(make_prefix(act.get_result_service.request_message))__add_methods (lua_State* L);
void @(make_prefix(act.get_result_service.response_message))__add_methods (lua_State* L);
void @(make_prefix(act.feedback_message))__add_methods (lua_State* L);

@[end for]@
// library

int luaopen_@(package_name)_action (lua_State* L)
{
  lua_createtable(L, 0, @(len(content)));    // push table "action"
@[for act in content]@
@{
send_names = act.send_goal_service.request_message.structure.namespaced_type.name.split('_')
}@  

  // open "namespace" @(send_names[0])
  lua_createtable(L, 0, 6);              // push table
  @(make_prefix(act.goal))__add_methods(L);
  @(make_prefix(act.result))__add_methods(L);
  @(make_prefix(act.feedback))__add_methods(L);
  @(make_prefix(act.feedback_message))__add_methods(L);
  // open "namespace" @(send_names[1])
  lua_createtable(L, 0, 2);             // push table
  @(make_prefix(act.send_goal_service.request_message))__add_methods(L);
  @(make_prefix(act.send_goal_service.response_message))__add_methods(L);
  // close "namespace" @(send_names[1])
  lua_setfield(L, -2, "@(send_names[1])");   // pop table
@{
result_names = act.get_result_service.request_message.structure.namespaced_type.name.split('_')
}@
  // open "namespace" @(result_names[1])
  lua_createtable(L, 0, 2);             // push table
  @(make_prefix(act.get_result_service.request_message))__add_methods(L);
  @(make_prefix(act.get_result_service.response_message))__add_methods(L);
  // close "namespace" @(result_names[1])
  lua_setfield(L, -2, "@(result_names[1])");  // pop table
  // close "namespace" @(send_names[0])
  lua_setfield(L, -2, "@(send_names[0])");   // pop table
@[end for]@

  return 1;
}

