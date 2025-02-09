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

#include <rosidl_luacommon/definition.h>

#include "rcllua/service.h"
#include "rcllua/node.h"
#include "rcllua/qos.h"
#include "rcllua/utils.h"

/** Indices of service bindings in register. */
enum SrvReg {
  /** service */
  SRV_REG_SERVICE = 1,
  /** function */
  SRV_REG_CALLBACK,
  /** request message constructor */
  SRV_REG_NEW_REQUEST,
  /** response metatable */
  SRV_REG_MT_RESPONSE,
  /** number of elements + 1 */
  SRV_REG_NUMBER
};

/** List of output elements. */
enum SrvOut {
  /** request message */
  SRV_OUT_REQUEST = 1,
  /** callback funciton */
  SRV_OUT_CALLBACK,
  /** header */
  SRV_OUT_HEADER,
  /** light userdata */
  SRV_OUT_REF,
  /** number of elements + 1 */
  SRV_OUT_NUMBER
};


/** Service object metatable name. */
const char* MT_SERVICE = "ROS2.Service";

/**
 * Create service object or wrap the existed one C structure. 
 * Save bindings to register.
 *
 * Arguments:
 * - node object
 * - service type (table)
 * - topic name
 * - callback function: fn(request) -> response
 * - qos profile (optional)
 * - rcl service object (optional)
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
    if (ROSIDL_LUA_PUSH_TYPESUPPORT(L, 2) == LUA_TLIGHTUSERDATA) {
      ts = lua_touserdata(L, -1);
    }
    lua_pop(L, 1);                       // pop pointer
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

  /* make rcllua service object */
  rcllua_service_wrap *wrap = lua_newuserdata(L, sizeof(rcllua_service_wrap));  // push object
  wrap->node = node;

    /* save reference objects into table */
  lua_createtable(L, SRV_REG_NUMBER-1, 0);  // push table a

  if (lua_islightuserdata(L, 6)) {
    /* save existed service */
    wrap->service = lua_touserdata(L, 6);

  } else {
    /* create new service */
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

    lua_rawseti(L, -2, SRV_REG_SERVICE);   // pop object
    wrap->service = srv;
  }

  /* set metamethods */
  luaL_getmetatable(L, MT_SERVICE);  // push metatable
  lua_setmetatable(L, -3);           // pop metatable

  /* the rest of bindings */
  lua_pushvalue(L, 4);                 // push function
  lua_rawseti(L, -2, SRV_REG_CALLBACK);    // pop function, a[.] = callback

  lua_getfield(L, 2, "Request");       // push table b
  ROSIDL_LUA_PUSH_CONSTRUCTOR(L, -1);         // push function
  lua_rawseti(L, -3, SRV_REG_NEW_REQUEST);  // pop funciton, a[.] = function
  lua_pop(L, 1);                       // pop table b

  lua_getfield(L, 2, "Response");      // push table b
  ROSIDL_LUA_PUSH_MT(L, -1);   // push name
  lua_rawseti(L, -3, SRV_REG_MT_RESPONSE);  // pop name, a[.] = metatable
  lua_pop(L, 1);                       // pop table b

  lua_rawsetp(L, LUA_REGISTRYINDEX, wrap->service);  // pop table a

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
  rcllua_service_wrap *wrap = lua_touserdata(L, 1);

  /* get service */
  lua_rawgetp(L, LUA_REGISTRYINDEX, wrap->service);     // push table
  if (lua_rawgeti(L, -1, SRV_REG_SERVICE) != LUA_TNIL) {    // push service
    rcl_service_t* srv = lua_touserdata(L, -1);

    /* finalize */
    rcl_ret_t ret = rcl_service_fini(srv, wrap->node);
    if (RCL_RET_OK != ret) {
      luaL_error(L, "failed to fini service: %s", rcl_get_error_string().str);
    }

  }
  /* free dependencies */
  lua_pushnil(L);
  lua_rawsetp(L, LUA_REGISTRYINDEX, wrap->service);

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
  rcllua_service_wrap* wrap = lua_touserdata(L, 1);
  const char* nm = rcl_service_get_service_name(wrap->service);

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
  rcllua_service_wrap* wrap = lua_touserdata(L, 1);
  const rcl_service_options_t* options = rcl_service_get_options(wrap->service);

  rcl_lua_qos_push_copy(L, &(options->qos));
  return 1;
}

/**
 * Send service response.
 *
 * Arguments:
 * - table from service request receiving
 * - response message
 *
 * Return:
 * - flag of success
 * - error message (optional)
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_service_send_response (lua_State* L)
{
  /* arg1 - table {request, response, cb, ... } */
  luaL_argcheck(L,
                LUA_TTABLE == lua_type(L, 1) && lua_rawlen(L, 1) == (SRV_OUT_NUMBER-1),
                1, "expected table from service request");

  /* get required elements */
  lua_rawgeti(L, 1, SRV_OUT_REF);        // push lightuserdata
  rcl_service_t* srv = lua_touserdata(L, -1);
  lua_rawgeti(L, 1, SRV_OUT_HEADER);     // push header
  rmw_service_info_t* header = lua_touserdata(L, -1);

  /* arg2 - response object */
  if(lua_rawgetp(L, LUA_REGISTRYINDEX, srv) == LUA_TNIL) {
    luaL_error(L, "service binding not found");
  }
  lua_rawgeti(L, -1, SRV_REG_MT_RESPONSE);
  const char* mt = lua_tostring(L, -1);
  idl_lua_msg_t *resp = luaL_checkudata(L, 2, mt);

  /* send response */
  rcl_ret_t ret = rcl_send_response(srv, &header->request_id, ROSIDL_LUA_GET_MSG(resp));
  switch (ret) {
    case RCL_RET_OK: break;
    case RCL_RET_TIMEOUT:
      lua_pushboolean(L, false);
      lua_pushfstring(L, "failed to send response (timeout): %s", rcl_get_error_string().str);
      rcl_reset_error();
      return 2;
    default:
      luaL_error(L, "failed to send response");
  }

  lua_pushboolean(L, true);
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

  lua_pushcfunction(L, rcl_lua_service_send_response);  // push function
  lua_setfield(L, -2, "service_send_response");   // pop, lib['service_send_response'] = func

  /* metamethods */
  rcl_lua_utils_add_mt(L, MT_SERVICE, srv_methods);
}

/* Receive request */
bool rcl_lua_service_push_callback (lua_State* L, const rcl_service_t* srv)
{
  /* save result into table */
  lua_createtable(L, SRV_OUT_NUMBER-1, 0);  // push table a

  /* save pointer */
  lua_pushlightuserdata(L, (void*) srv);    // push pointer
  lua_rawseti(L, -2, SRV_OUT_REF);          // pop pointer, a[.] = srv

  /* prepare request message */
  if (lua_rawgetp(L, LUA_REGISTRYINDEX, srv) == LUA_TNIL) {  // push table b (bindings)
    lua_pop(L, 2);
    return false;
  }
  lua_rawgeti(L, -1, SRV_REG_NEW_REQUEST);  // push constructor from b
  lua_call(L, 0, 1);                        // pop constructor, push message
  idl_lua_msg_t *msg = lua_touserdata(L, -1);

  /* get request */
  rmw_service_info_t header;
  rcl_ret_t ret = rcl_take_request_with_info(srv, &header, ROSIDL_LUA_GET_MSG(msg));
  switch (ret) {
    case RCL_RET_OK: break;
    case RCL_RET_SERVICE_TAKE_FAILED:
      lua_pop(L, 2);  // keep only a
      return true;      // {nil, ..., srv}
    default:
      luaL_error(L, "service failed to take request");
  }
  lua_rawseti(L, -3, SRV_OUT_REQUEST);     // pop message, a[.] = request

  /* save header */
  rmw_service_info_t* info = lua_newuserdata(L, sizeof(rmw_service_info_t));  // push header
  *info = header;
  lua_rawseti(L, -3, SRV_OUT_HEADER);      // pop header, a[.] = header

  /* save callback function */
  lua_rawgeti(L, -1, SRV_REG_CALLBACK);    // push function from b
  lua_rawseti(L, -3, SRV_OUT_CALLBACK);    // pop function, a[.] = callback

  lua_pop(L, 1);                           // pop table b
  /* keep table 'a' on stack */
  return true;
}
