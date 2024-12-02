
#include <lauxlib.h>

#include <rcl/node.h>
#include <rcl/error_handling.h>

#include "node.h"
#include "context.h"
#include "utils.h"

const char* MT_NODE = "ROS2.Node";

static int rcl_lua_node_init (lua_State* L)
{
  // check arguments
  if (!lua_isstring(L, 1)) {
    luaL_argerror(L, 1, "expected string");
  }
  const char* name = lua_tostring(L, 1);
  const char* namespace = "";
  if (!lua_isnil(L, 2)) {
    namespace = luaL_checkstring(L, 2);
  }

  // TODO add options
  rcl_node_options_t options = rcl_node_get_default_options();
  rcl_context_t* context = rcl_lua_context_ref();
  if (NULL == context) {
    luaL_error(L, "context is not initialized");
  }

  rcl_node_t* node = lua_newuserdata(L, sizeof(rcl_node_t));
  *node = rcl_get_zero_initialized_node();

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

  // set metatable
  luaL_getmetatable(L, MT_NODE);
  lua_setmetatable(L, -2);

  return 1;
}

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

static int rcl_lua_node_get_name (lua_State* L)
{
  rcl_node_t* node = luaL_checkudata(L, 1, MT_NODE);
  const char* name = rcl_node_get_name(node);
  lua_pushstring(L, name);

  return 1;
}

static int rcl_lua_node_get_namespace (lua_State* L)
{
  rcl_node_t* node = luaL_checkudata(L, 1, MT_NODE);
  const char* ns = rcl_node_get_namespace(node);
  lua_pushstring(L, ns);

  return 1;
}

static const struct luaL_Reg node_methods[] = {
  {"get_name", rcl_lua_node_get_name},
  {"get_namespace", rcl_lua_node_get_namespace},
  {"__tostring", rcl_lua_node_get_name},
  {"__gc", rcl_lua_node_free},
  {NULL, NULL}
};

void rcl_lua_add_node_methods (lua_State* L)
{
  // make node
  lua_pushcfunction(L, rcl_lua_node_init);
  lua_setfield(L, -2, "node_init");

  // metamethods
  rcl_lua_utils_add_mt(L, MT_NODE, node_methods);
}

