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

#include <rcl/service.h>
#include <rcl/node.h>
#include <rcl/error_handling.h>
#include <rosidl_runtime_c/service_type_support_struct.h>

#include "service.h"
#include "node.h"
#include "qos.h"
#include "utils.h"

/** Indices of service bindings in register. */
enum SrvReg {
  /** node reference */
  SRV_REG_NODE = 1,
  /** function */
  SRV_REG_CALLBACK
};

/** Service object metatable name. */
const char* MT_SERVICE = "ROS2.Service";

/**
 * Create service object. Save bindings to register.
 *
 * Arguments:
 * - node object
 * - service type (table)
 * - topic name
 * - callback function: fn(request, response) -> nil
 * - qos profile (optional)
 *
 * Return:
 * - service object
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_service_init (lua_State* L)
{
  /* arg1 - node */
  rcl_node_t* node = luaL_checkudata(L, 1, MT_NODE);

  /* arg2 - message type */
  rosidl_service_type_support_t *ts = NULL;
  /* check table */
  if (lua_istable(L, 2)) {
    lua_getfield(L, 2, "_type_support");   // push pointer
    if (lua_islightuserdata(L, -1)) {
      ts = lua_touserdata(L, -1);
      lua_pop(L, 1);                       // pop pointer
    }
  }
  if (NULL == ts) {
    luaL_argerror(L, 2, "expected service type");
  }

  /* arg3 - service name */
  const char* srv_name = luaL_checkstring(L, 3);
  /* arg4 - callback function */
  luaL_argcheck(L, LUA_TFUNCTION == lua_type(L, 4), 4, "function expected");


  /* init service */
  rcl_service_options_t service_ops = rcl_service_get_default_options();
  /* arg5 - QoS profile */
  if (!lua_isnoneornil(L, 5)) {
    rmw_qos_profile_t* qos = luaL_checkudata(L, 5, MT_QOS);
    service_ops.qos = *qos;
  }
  rcl_service_t *srv = lua_newuserdata(L, sizeof(rcl_service_t));  // push object
  *srv = rcl_get_zero_initialized_service();

  rcl_ret_t ret = rcl_service_init(srv, node, ts, srv_name, &service_ops);
  switch (ret) {
    case RCL_RET_OK: break;
    case RCL_RET_SERVICE_NAME_INVALID:
      luaL_error(L, "invalid service name %s", srv_name); break;
    default:
      luaL_error(L, "failed to create service");
  }

  /* set metamethods */
  luaL_getmetatable(L, MT_SERVICE);  // push metatable
  lua_setmetatable(L, -2);           // pop metatable

  /* save reference objects */
  lua_createtable(L, 0, 0);            // push table a
  lua_pushvalue(L, 1);                 // push node
  lua_rawseti(L, -2, SRV_REG_NODE);    // pop node, a[1] = node

  lua_pushvalue(L, 4);                 // push function
  lua_rawseti(L, -2, SRV_REG_CALLBACK);    // pop function, a[.] = callback

  lua_rawsetp(L, LUA_REGISTRYINDEX, srv);  // pop table a

  return 1;
}

/**
 * Service destructor.
 *
 * Arguments:
 * - service object
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_service_free (lua_State* L)
{
  /* arg1 - service object */
  rcl_service_t* srv = lua_touserdata(L, 1);

  /* get node */
  lua_rawgetp(L, LUA_REGISTRYINDEX, srv);  // push table
  lua_rawgeti(L, -1, SRV_REG_NODE);        // push node
  rcl_node_t* node = lua_touserdata(L, -1);

  /* finalize */
  rcl_ret_t ret = rcl_service_fini(srv, node);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to fini service: %s", rcl_get_error_string().str);
    rcl_reset_error();
  }

  /* free dependencies */
  lua_pushnil(L);
  lua_rawsetp(L, LUA_REGISTRYINDEX, srv);

  return 0;
}

/**
 * Get service name.
 *
 * Arguments:
 * - service object
 *
 * Return:
 * - name string
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_service_get_name (lua_State* L)
{
  /* arg1 - service object */
  rcl_service_t* srv = luaL_checkudata(L, 1, MT_SERVICE);
  const char* nm = rcl_service_get_service_name(srv);

  lua_pushstring(L, nm);
  return 1;
}

/**
 * Get QoS profile.
 *
 * Arguments:
 * - service object
 *
 * Return:
 * - qos object
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_service_get_qos (lua_State* L)
{
  /* arg1 - service object */
  rcl_service_t* srv = luaL_checkudata(L, 1, MT_SERVICE);
  const rcl_service_options_t* options = rcl_service_get_options(srv);

  rcl_lua_qos_push_copy(L, &(options->qos));
  return 1;
}

/** List of service methods */
static const struct luaL_Reg srv_methods[] = {
  {"get_qos", rcl_lua_service_get_qos},
  {"get_name", rcl_lua_service_get_name},
  {"__gc", rcl_lua_service_free},
  {NULL, NULL}
};

/* Add service to library */
void rcl_lua_add_service_methods (lua_State* L)
{
  /* constructor */
  lua_pushcfunction(L, rcl_lua_service_init);  // push function
  lua_setfield(L, -2, "new_service");          // pop, lib['new_service'] = function

  /* metamethods */
  rcl_lua_utils_add_mt(L, MT_SERVICE, srv_methods);
}
