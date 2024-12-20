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
#include <rcl/error_handling.h>

#include "rcllua/node.h"
#include "rcllua/context.h"
#include "rcllua/utils.h"

/** Node object metatable name */
const char* MT_NODE = "ROS2.Node";

/**
 * Create node object.
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
static int rcl_lua_node_init (lua_State* L)
{
  /* arg1 - node name */
  const char* name = luaL_checkstring(L, 1);
  /* arg2 - namespace */
  const char* namespace = luaL_optstring(L, 2, "");
  // TODO(Mikhel) add options

  /* initialize */
  rcl_context_t* context = rcl_lua_context_ref();
  if (NULL == context) {
    luaL_error(L, "context is not initialized");
  }

  rcl_node_t* node = lua_newuserdata(L, sizeof(rcl_node_t));  // push object
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
static int rcl_lua_node_free (lua_State* L)
{
  rcl_node_t* node = lua_touserdata(L, 1);

  rcl_ret_t ret = rcl_node_fini(node);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to fini node: %s", rcl_get_error_string().str);
    rcl_reset_error();
  }

  return 0;
}

/**
 * Get node name.
 *
 * Return:
 * - name
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_node_get_name (lua_State* L)
{
  rcl_node_t* node = luaL_checkudata(L, 1, MT_NODE);
  lua_pushstring(L, rcl_node_get_name(node));

  return 1;
}

/**
 * Get current namespace.
 *
 * Return:
 * - namespace
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_node_get_namespace (lua_State* L)
{
  rcl_node_t* node = luaL_checkudata(L, 1, MT_NODE);
  lua_pushstring(L, rcl_node_get_namespace(node));

  return 1;
}

/** List of node methods */
static const struct luaL_Reg node_methods[] = {
  {"get_name", rcl_lua_node_get_name},
  {"get_namespace", rcl_lua_node_get_namespace},
  {"__gc", rcl_lua_node_free},
  {NULL, NULL}
};

/* Add to library */
void rcl_lua_add_node_methods (lua_State* L)
{
  /* constructor */
  lua_pushcfunction(L, rcl_lua_node_init);  // push function
  lua_setfield(L, -2, "new_node");          // pop, lib['new_node'] = function

  /* metamethods */
  rcl_lua_utils_add_mt(L, MT_NODE, node_methods);
}
