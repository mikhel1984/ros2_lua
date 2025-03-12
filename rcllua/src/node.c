// Copyright 2025 Stanislav Mikhel
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <lauxlib.h>

#include <rcl/node.h>
#include <rcl/graph.h>
#include <rcl_action/rcl_action.h>

#include "rcllua/node.h"
#include "rcllua/context.h"
#include "rcllua/utils.h"

/** Node object metatable name */
const char * MT_NODE = "ROS2.Node";

/**
 * Create node object.
 *
 * Table: rclbind
 * Method: new_node
 *
 * Arguments:
 * - node name
 * - namespace (string, optional)
 *
 * Return:
 * - node object.
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_node_init(lua_State * L)
{
  /* arg1 - node name */
  const char * name = luaL_checkstring(L, 1);
  /* arg2 - namespace */
  const char * namespace = luaL_optstring(L, 2, "");
  // TODO(Mikhel) add options

  /* initialize */
  rcl_context_t * context = rcl_lua_context_ref();
  if (NULL == context) {
    luaL_error(L, "context is not initialized");
  }

  rcl_node_t * node = lua_newuserdata(L, sizeof(rcl_node_t));  // push object
  *node = rcl_get_zero_initialized_node();
  rcl_node_options_t options = rcl_node_get_default_options();

  rcl_ret_t ret = rcl_node_init(node, name, namespace, context, &options);
  switch (ret) {
    case RCL_RET_OK: break;
    case RCL_RET_BAD_ALLOC:
      luaL_error(L, "allocation error"); break;
    case RCL_RET_NODE_INVALID_NAME:
      luaL_error(L, "invalid node name"); break;
    case RCL_RET_NODE_INVALID_NAMESPACE:
      luaL_error(L, "invalid node namespace"); break;
    default:
      luaL_error(L, "error creating node");
  }

  /* set metamethods */
  luaL_getmetatable(L, MT_NODE);  // push metatable
  lua_setmetatable(L, -2);        // pop metatable

  return 1;
}

/**
 * Node destructor.
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_node_free(lua_State * L)
{
  rcl_node_t * node = lua_touserdata(L, 1);

  rcl_ret_t ret = rcl_node_fini(node);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to fini node");
  }

  return 0;
}

/**
 * Template for getting name of node component.
 * \param fun ROS function to call.
 * \param msg Error message.
 */
#define GET_COMPONENT_NAME(fun, msg) \
  rcl_node_t * node = lua_touserdata(L, 1); \
  luaL_argcheck(L, NULL != node, 1, "node is expected"); \
  const char * name = fun (node); \
  if (NULL == name) { \
    luaL_error(L, msg); \
  } \
  lua_pushstring(L, name); \
  return 1;

/**
 * Get fully qualified node name.
 *
 * Table: Node
 * Method: get_fully_qualified_name
 *
 * Arguments:
 * - node object
 *
 * Return:
 * - node name
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_node_full_qualified_name(lua_State * L)
{
  GET_COMPONENT_NAME(rcl_node_get_fully_qualified_name, "fully qualified name not set")
}

/**
 * Get logger name.
 *
 * Table: Node
 * Method: get_logger_name
 *
 * Arguments:
 * - node object
 *
 * Return:
 * - logger name
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_node_logger_name(lua_State * L)
{
  GET_COMPONENT_NAME(rcl_node_get_logger_name, "logger name not set");
}

/**
 * Get node name.
 *
 * Table: Node
 * Method: get_name
 *
 * Arguments:
 * - node object
 *
 * Return:
 * - node name
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_node_get_name(lua_State * L)
{
  GET_COMPONENT_NAME(rcl_node_get_name, "node name not set");
}

/**
 * Get current namespace.
 *
 * Table: Node
 * Method: get_namespace
 *
 * Arguments:
 * - node object
 *
 * Return:
 * - namespace
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_node_get_namespace(lua_State * L)
{
  GET_COMPONENT_NAME(rcl_node_get_namespace, "namespace not set");
}

/**
 * Get number of publishers.
 *
 * Table: Node
 * Method: get_count_publishers
 *
 * Arguments:
 * - node object
 * - topic name
 *
 * Return:
 * - number of publishers
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_node_count_publishers(lua_State * L)
{
  /* arg1 - node object */
  rcl_node_t * node = luaL_checkudata(L, 1, MT_NODE);
  /* arg2 - topic name */
  const char * topic = luaL_checkstring(L, 2);

  size_t count = 0;
  rcl_ret_t ret = rcl_count_publishers(node, topic, &count);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to count publishers");
  }

  lua_pushinteger(L, count);
  return 1;
}

/**
 * Get number of subscribers.
 *
 * Table: Node
 * Method: get_count_subscribers
 *
 * Arguments:
 * - node object
 * - topic name
 *
 * Return:
 * - number of subscribers
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_node_count_subscribers(lua_State * L)
{
  /* arg1 - node object */
  rcl_node_t * node = luaL_checkudata(L, 1, MT_NODE);
  /* arg2 - topic name */
  const char * topic = luaL_checkstring(L, 2);

  size_t count = 0;
  rcl_ret_t ret = rcl_count_subscribers(node, topic, &count);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to count subscribers");
  }

  lua_pushinteger(L, count);
  return 1;
}

/**
 * Get action client names and types by node.
 *
 * Table: Node
 * Method: get_action_client_names_and_types_by_node
 *
 * Arguments:
 * - node object
 * - remote node name
 * - remote node namespace
 *
 * Return:
 * - table of names and types
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_node_action_client_names_types(lua_State * L)
{
  /* arg1 - node object */
  rcl_node_t * node = luaL_checkudata(L, 1, MT_NODE);
  /* arg2 - remote node name */
  const char * remote_name = luaL_checkstring(L, 2);
  /* arg3 - remote node namespace */
  const char * remote_ns = luaL_checkstring(L, 3);

  rcl_names_and_types_t names_types = rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rcl_action_get_client_names_and_types_by_node(
    node, &allocator, remote_name, remote_ns, &names_types);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to get action client names and types");
  }

  rcl_lua_utils_push_names_types(L, &names_types);
  return 1;
}

/**
 * Get action server names and types by node.
 *
 * Table: Node
 * Method: get_action_server_names_and_types_by_node
 *
 * Arguments:
 * - node object
 * - remote node name
 * - remote node namespace
 *
 * Return:
 * - table of names and types
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_node_action_server_names_types(lua_State * L)
{
  /* arg1 - node object */
  rcl_node_t * node = luaL_checkudata(L, 1, MT_NODE);
  /* arg2 - remote node name */
  const char * remote_name = luaL_checkstring(L, 2);
  /* arg3 - remote node namespace */
  const char * remote_ns = luaL_checkstring(L, 3);

  rcl_names_and_types_t names_types = rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rcl_action_get_server_names_and_types_by_node(
    node, &allocator, remote_name, remote_ns, &names_types);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to get action server names and types");
  }

  rcl_lua_utils_push_names_types(L, &names_types);
  return 1;
}

/**
 * Get action names and types by node.
 *
 * Table: Node
 * Method: get_action_names_and_types
 *
 * Arguments:
 * - node object
 *
 * Return:
 * - table of names and types
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_node_action_names_and_types(lua_State * L)
{
  /* arg1 - node object */
  rcl_node_t * node = luaL_checkudata(L, 1, MT_NODE);

  rcl_names_and_types_t names_types = rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rcl_action_get_names_and_types(node, &allocator, &names_types);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to get action names and types");
  }

  rcl_lua_utils_push_names_types(L, &names_types);
  return 1;
}

/** List of node methods */
static const struct luaL_Reg node_methods[] = {
  {"get_name", rcl_lua_node_get_name},
  {"get_namespace", rcl_lua_node_get_namespace},
  {"get_fully_qualified_name", rcl_lua_node_full_qualified_name},
  {"get_logger_name", rcl_lua_node_logger_name},
  {"get_count_publishers", rcl_lua_node_count_publishers},
  {"get_count_subscribers", rcl_lua_node_count_subscribers},
  {"get_action_client_names_and_types_by_node", rcl_lua_node_action_client_names_types},
  {"get_action_server_names_and_types_by_node", rcl_lua_node_action_server_names_types},
  {"get_action_names_and_types", rcl_lua_node_action_names_and_types},
  {"__gc", rcl_lua_node_free},
  {NULL, NULL}
};

/* Add to library */
void rcl_lua_add_node_methods(lua_State * L)
{
  /* constructor */
  lua_pushcfunction(L, rcl_lua_node_init);  // push function
  lua_setfield(L, -2, "new_node");          // pop, lib['new_node'] = function

  /* metamethods */
  rcl_lua_utils_add_mt(L, MT_NODE, node_methods);
}
