
#include <lauxlib.h>

#include <rcl/publisher.h>
#include <rcl/node.h>
#include <rcl/error_handling.h>

#include <rmw/types.h>

#include <rosidl_runtime_c/message_type_support_struct.h>

#include "publisher.h"

const char* MT_PUBLISHER = "ROS2.Publisher";

void rcl_lua_publisher_new (lua_State* L)
{
  rcl_node_t* node = lua_touserdata(L, 1);
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
  luaL_getmetatable(L, MT_PUBLISHER);
  lua_setmetatable(L, -2);

  /* save node reference */
  lua_pushvalue(L, 1);  // duplicate node reference
  lua_rawsetp(L, LUA_REGISTRYINDEX, publisher);  // reg[pub] = node

  return 1;
}

static int rcl_lua_publisher_free (lua_State* L)
{
  rcl_publisher_t* publisher = lua_touserdata(L, 1);
  // get node
  lua_rawgetp(L, publisher);
  rcl_node_t* node = lua_touserdata(L, -1);
  
  rcl_ret_t ret = rcl_publisher_fini(publisher, node);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to fini publisher: %s", rcl_get_error_string().str);
    rcl_reset_error();
  }

  // free node pointer
  lua_pushnil();
  lua_rawsetp(L, LUA_REGISTRYINDEX, publisher);
  
  return 0;
}

static int rcl_lua_publisher_publish (lua_State* L)
{
  rcl_publisher_t *publisher = luaL_checkudata(L, 1, MT_PUBLISHER);
  void* message = NULL;  // get message

  rcl_ret_t ret = rcl_publish(publisher, message, NULL);
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
  // metatable
  luaL_newmetatable(L, MT_PUBLISHER);
  lua_pushvalue(L, -1);
  lua_setfield(L, -2, "__index");
  // publisher methods
  luaL_setfuncs(L, pub_methods, 0);
  // remove methatable from stack
  lua_pop(L, 1);
}
