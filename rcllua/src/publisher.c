
#include <lauxlib.h>

#include <rcl/publisher.h>
#include <rcl/node.h>
#include <rcl/error_handling.h>

#include <rmw/types.h>

#include <rosidl_runtime_c/message_type_support_struct.h>

#include <rosidl_luacommon/definition.h>

#include "publisher.h"
#include "node.h"
#include "utils.h"

const char* MT_PUBLISHER = "ROS2.Publisher";

static int rcl_lua_publisher_init (lua_State* L)
{
  rcl_node_t* node = luaL_checkudata(L, 1, MT_NODE);
  rosidl_message_type_support_t *ts = NULL; // TODO read message type

  bool is_ref = false;
  /* check table */
  if (lua_istable(L, 2)) {
    lua_getfield(L, 2, "_type_support");
    if (lua_islightuserdata(L, -1)) {
      ts = lua_touserdata(L, -1);
      lua_pop(L, 1);
      is_ref = true;
    }
  }
  if (!is_ref) {
    luaL_argerror(L, 2, "expected message type");
  }
  if (NULL == ts) {
    luaL_error(L, "message not found");
  }
  const char* topic = lua_tostring(L, 3);
  // TODO read qos profile

  rcl_publisher_t *publisher = lua_newuserdata(L, sizeof(rcl_publisher_t));
  *publisher = rcl_get_zero_initialized_publisher();

  rcl_publisher_options_t publisher_opt = rcl_publisher_get_default_options();

  // TODO update options with qos

  rcl_ret_t ret = rcl_publisher_init(publisher, node, ts, topic, &publisher_opt);
  switch (ret) {
    case RCL_RET_OK: break;
    case RCL_RET_TOPIC_NAME_INVALID:
      luaL_error(L, "invalid topic name %s", topic); break;
    default:
      luaL_error(L, "failed to create publisher");
  }

  /* set metatable */
  luaL_getmetatable(L, MT_PUBLISHER);  // push metatable
  lua_setmetatable(L, -2);   // pop metatable, set

  /* save node reference and metatable */
  lua_createtable(L, 2, 0);  // push table
  lua_pushvalue(L, 1);  // push node
  lua_seti(L, -2, 1);   // pop node, set t[1] = node

  lua_getfield(L, 2, "_metatable");  // push name
  lua_seti(L, -2, 2);   // pop name, set t[2] = name

  lua_rawsetp(L, LUA_REGISTRYINDEX, publisher);  // pop table, reg[pub] = t

  return 1;
}

static int rcl_lua_publisher_free (lua_State* L)
{
  rcl_publisher_t* publisher = lua_touserdata(L, 1);
  // get node
  lua_rawgetp(L, LUA_REGISTRYINDEX, publisher);
  lua_geti(L, -1, 1);
  rcl_node_t* node = lua_touserdata(L, -1);
  
  rcl_ret_t ret = rcl_publisher_fini(publisher, node);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to fini publisher: %s", rcl_get_error_string().str);
    rcl_reset_error();
  }

  // free node pointer
  lua_pushnil(L);
  lua_rawsetp(L, LUA_REGISTRYINDEX, publisher);
  
  return 0;
}

static int rcl_lua_publisher_publish (lua_State* L)
{
  rcl_publisher_t *publisher = luaL_checkudata(L, 1, MT_PUBLISHER);
  /* check message type */
  lua_rawgetp(L, LUA_REGISTRYINDEX, publisher);
  lua_geti(L, -1, 2);
  const char* mt = lua_tostring(L, -1);
  idl_lua_msg_t *msg = luaL_checkudata(L, 2, mt);

  rcl_ret_t ret = rcl_publish(publisher, msg->obj, NULL);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to publish");
  }

  return 0;
}

static const struct luaL_Reg pub_methods[] = {
  {"publish", rcl_lua_publisher_publish},
  {"__gc", rcl_lua_publisher_free},
  {NULL, NULL}
};

void rcl_lua_add_publisher_methods (lua_State* L)
{
  lua_pushcfunction(L, rcl_lua_publisher_init);
  lua_setfield(L, -2, "publisher_init");

  // metatable
  rcl_lua_utils_add_mt(L, MT_PUBLISHER, pub_methods);
}
