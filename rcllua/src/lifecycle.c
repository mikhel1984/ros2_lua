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

#include <rcl_lifecycle/rcl_lifecycle.h>
#include <rcl/service.h>
#include <rcl/error_handling.h>

#include <rosidl_runtime_c/message_type_support_struct.h>
#include <rosidl_runtime_c/service_type_support_struct.h>

#include <rosidl_luacommon/definition.h>

#include "rcllua/lifecycle.h"
#include "rcllua/node.h"
#include "rcllua/qos.h"
#include "rcllua/utils.h"

#define MAYBE_NULL(X) (X) ? (X) : ""

enum FsmReg {
  /** node reference */
  FSM_REG_NODE = 1,
  /** number of elements + 1 */
  FSM_REG_NUMBER
};


const char* MT_LIFECYCLE = "ROS2.Lifecycle";

void* rcl_lua_lifecycle_get_typesupport (lua_State* L, int tbl, const char* nm)
{
  void * ts = NULL;
  if (lua_getfield(L, tbl, nm) == LUA_TTABLE) {
    if (ROSIDL_LUA_PUSH_TYPESUPPORT(L, -1) == LUA_TLIGHTUSERDATA) {
      ts = lua_touserdata(L, -1);
    }
  }
  if (NULL == ts) {
    luaL_error(L, "not found type support for %s", nm);
  }
  lua_pop(L, 2);

  return ts;
}

static int rcl_lua_lifecycle_init (lua_State* L)
{
  /* arg1 - node */
  rcl_node_t* node = luaL_checkudata(L, 1, MT_NODE);
  /* arg2 - interface flag */
  bool enable_com_interface = lua_toboolean(L, 2);

  /* arg3 - interface tables */
  luaL_argcheck(L, lua_istable(L, 3), 3, "table with interfaces is expected");
  rosidl_message_type_support_t* ts_pub_notify = 
    rcl_lua_lifecycle_get_typesupport(L, 3, "TransitionEvent");
  rosidl_service_type_support_t* ts_srv_change_state =
    rcl_lua_lifecycle_get_typesupport(L, 3, "ChangeState");
  rosidl_service_type_support_t* ts_srv_get_state = 
    rcl_lua_lifecycle_get_typesupport(L, 3, "GetState");
  rosidl_service_type_support_t* ts_srv_get_available_states = 
    rcl_lua_lifecycle_get_typesupport(L, 3, "GetAvailableStates");
  rosidl_service_type_support_t* ts_srv_get_available_transitions = 
    rcl_lua_lifecycle_get_typesupport(L, 3, "GetAvailableTransitions");
  rosidl_service_type_support_t* ts_srv_get_transition_graph = ts_srv_get_available_transitions;

  rcl_lifecycle_state_machine_t *fsm = lua_newuserdata(L, sizeof(rcl_lifecycle_state_machine_t));
  *fsm = rcl_lifecycle_get_zero_initialized_state_machine();

  rcl_lifecycle_state_machine_options_t ops = rcl_lifecycle_get_default_state_machine_options();
  ops.enable_com_interface = enable_com_interface;

  rcl_ret_t ret = rcl_lifecycle_state_machine_init(
    fsm, node,
    ts_pub_notify,
    ts_srv_change_state,
    ts_srv_get_state,
    ts_srv_get_available_states,
    ts_srv_get_available_transitions,
    ts_srv_get_transition_graph,
    &ops);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to create lifecycle state machine");
  }

  /* set metamethods */
  luaL_getmetatable(L, MT_LIFECYCLE);  // push metatable
  lua_setmetatable(L, -2);             // pop metatable

  /* save references */
  lua_createtable(L, FSM_REG_NUMBER-1, 0);
  lua_pushvalue(L, 1);
  lua_rawseti(L, -2, FSM_REG_NODE);

  lua_rawsetp(L, LUA_REGISTRYINDEX, fsm);

  return 1;
}

static int rcl_lua_lifecycle_free (lua_State* L)
{
  /* arg1 - state machine */
  rcl_lifecycle_state_machine_t* fsm = luaL_checkudata(L, 1, MT_LIFECYCLE);

  /* get node */
  lua_rawgetp(L, LUA_REGISTRYINDEX, fsm);  // push table
  lua_rawgeti(L, -1, FSM_REG_NODE);        // push node
  rcl_node_t* node = lua_touserdata(L, -1);

  rcl_ret_t ret = rcl_lifecycle_state_machine_fini(fsm, node);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to fini lifecycle state machine: %s", rcl_get_error_string().str);
  }

  /* free dependencies */
  lua_pushnil(L);
  lua_rawsetp(L, LUA_REGISTRYINDEX, fsm);

  return 0;
}

static int rcl_lua_lifecycle_is_initialized (lua_State* L)
{
  /* arg1 - state machine */
  rcl_lifecycle_state_machine_t* fsm = lua_touserdata(L, 1);

  rcl_ret_t ret = rcl_lifecycle_state_machine_is_initialized(fsm);

  lua_pushboolean(L, RCL_RET_OK == ret);
  return 1;
}

static int rcl_lua_lifecycle_trigger_by_id (lua_State* L)
{
  /* arg1 - state machine */
  rcl_lifecycle_state_machine_t* fsm = luaL_checkudata(L, 1, MT_LIFECYCLE);
  /* arg2 - transition id */
  int id = luaL_checkinteger(L, 2);
  luaL_argcheck(L, 0 <= id && id < 256, 2, "expected uint8 value");
  /* arg3 - publish flag */
  luaL_argcheck(L, lua_isboolean(L, 3), 3, "boolean expected");

  rcl_ret_t ret = rcl_lifecycle_trigger_transition_by_id(fsm, (uint8_t) id, lua_toboolean(L, 3));
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to trigger lifecycle state machine transition");
  }

  return 0;
}

static int rcl_lua_lifecycle_trigger_by_label (lua_State* L)
{
  /* arg1 - state machine */
  rcl_lifecycle_state_machine_t* fsm = luaL_checkudata(L, 1, MT_LIFECYCLE);
  /* arg2 - label */
  const char* label = luaL_checkstring(L, 2);
  /* arg3 - publish flag */
  luaL_argcheck(L, lua_isboolean(L, 3), 3, "boolean expected");

  rcl_ret_t ret = rcl_lifecycle_trigger_transition_by_label(fsm, label, lua_toboolean(L, 3));
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to trigger lifecycle state machine transition");
  }
  
  return 0;
}

static int rcl_lua_lifecycle_get_by_label (lua_State* L)
{
  /* arg1 - state machine */
  rcl_lifecycle_state_machine_t* fsm = luaL_checkudata(L, 1, MT_LIFECYCLE);
  /* arg2 - label */
  const char* label = luaL_checkstring(L, 2);

  const rcl_lifecycle_transition_t* transition = 
    rcl_lifecycle_get_transition_by_label(fsm->current_state, label);
  if (NULL == transition) {
    lua_pushboolean(L, false);
    lua_pushliteral(L, "failed to get transition from label");
    return 2;
  }

  lua_pushboolean(L, true);
  lua_pushinteger(L, transition->id);
  return 2;
}

static int rcl_lua_lifecycle_get_state (lua_State* L)
{
  /* arg1 - state machine */
  rcl_lifecycle_state_machine_t* fsm = lua_touserdata(L, 1);

  lua_createtable(L, 2, 0);      // push table
  lua_pushinteger(L, fsm->current_state->id);  // push int
  lua_rawseti(L, -2, 1);         // pop int
  lua_pushstring(L, fsm->current_state->label);  // push string
  lua_rawseti(L, -2, 2);         // pop string

  return 1;
}

static int rcl_lua_lifecycle_get_available_states (lua_State* L)
{
  /* arg1 - state machine */
  rcl_lifecycle_state_machine_t* fsm = lua_touserdata(L, 1);

  lua_createtable(L, fsm->transition_map.states_size, 0);  // push table a
  for (size_t i = 0; i < fsm->transition_map.states_size; i++) {
    lua_createtable(L, 2, 0);     // push table b
    lua_pushinteger(L, fsm->transition_map.states[i].id);  // push int
    lua_rawseti(L, -2, 1);        // pop in
    lua_pushstring(L, MAYBE_NULL(fsm->transition_map.states[i].label));  // push string
    lua_rawseti(L, -2, 2);        // pop string
    lua_rawseti(L, -2, i+1);      // pop table b
  }

  return 1;
}

static int rcl_lua_lifecycle_get_available_transitions (lua_State* L)
{
  /* arg1 - state machine */
  rcl_lifecycle_state_machine_t* fsm = lua_touserdata(L, 1);

  lua_createtable(L, fsm->current_state->valid_transition_size, 0);   // push table a
  for (size_t i = 0; i < fsm->current_state->valid_transition_size; ++i) {
    lua_createtable(L, 6, 0);   // push table b
    lua_pushinteger(L, fsm->current_state->valid_transitions[i].id);  // push int
    lua_rawseti(L, -2, 1);      // pop int
    lua_pushstring(L, MAYBE_NULL(fsm->current_state->valid_transitions[i].label));  // push string
    lua_rawseti(L, -2, 2);      // pop string
    lua_pushinteger(L, fsm->current_state->valid_transitions[i].start->id);  // push int
    lua_rawseti(L, -2, 3);      // pop int
    lua_pushstring(L, MAYBE_NULL(fsm->current_state->valid_transitions[i].start->label));  // push string
    lua_rawseti(L, -2, 4);      // pop string
    lua_pushinteger(L, fsm->current_state->valid_transitions[i].goal->id);   // push int
    lua_rawseti(L, -2, 5);      // pop int
    lua_pushstring(L, MAYBE_NULL(fsm->current_state->valid_transitions[i].goal->label));  // push string
    lua_rawseti(L, -2, 6);      // pop string
    lua_rawseti(L, -2, i+1);    // pop table b
  }

  return 1;
}

static int rcl_lua_lifecycle_get_transition_graph (lua_State* L)
{
  /* arg1 - state machine */
  rcl_lifecycle_state_machine_t* fsm = lua_touserdata(L, 1);

  lua_createtable(L, fsm->transition_map.transitions_size, 0);  // push table a
  for (size_t i = 0; i < fsm->transition_map.transitions_size; ++i) {
    lua_createtable(L, 6, 0);   // push table b
    lua_pushinteger(L, fsm->transition_map.transitions[i].id);  // push int
    lua_rawseti(L, -2, 1);      // pop int
    lua_pushstring(L, fsm->transition_map.transitions[i].label);  // push string
    lua_rawseti(L, -2, 2);      // pop string
    lua_pushinteger(L, fsm->transition_map.transitions[i].start->id);  // push int
    lua_rawseti(L, -2, 3);      // pop int
    lua_pushstring(L, fsm->transition_map.transitions[i].start->label);  // push string
    lua_rawseti(L, -2, 4);      // pop string
    lua_pushinteger(L, fsm->transition_map.transitions[i].goal->id);   // push int
    lua_rawseti(L, -2, 5);      // pop int
    lua_pushstring(L, fsm->transition_map.transitions[i].goal->label);  // push string
    lua_rawseti(L, -2, 6);      // pop string
    lua_rawseti(L, -2, i+1); 
  }

  return 1;
}

static int rcl_lua_lifecycle_print (lua_State* L)
{
  /* arg1 - state machine */
  rcl_lifecycle_state_machine_t* fsm = lua_touserdata(L, 1);

  rcl_print_state_machine(fsm);

  return 0;
}

static int rcl_lua_lifecycle_get_service (lua_State* L)
{
  /* arg1 - state machine */
  rcl_lifecycle_state_machine_t* fsm = luaL_checkudata(L, 1, MT_LIFECYCLE);
  /* arg2 - service name */
  const char* name = luaL_checkstring(L, 2);

  if (strcmp(name, "ChangeState") == 0) {
    lua_pushlightuserdata(L, &fsm->com_interface.srv_change_state);
  } else if (strcmp(name, "GetState") == 0) {
    lua_pushlightuserdata(L, &fsm->com_interface.srv_get_state);
  } else if (strcmp(name, "GetAvailableStates") == 0) {
    lua_pushlightuserdata(L, &fsm->com_interface.srv_get_available_states);
  } else if (strcmp(name, "GetAvailableTransitions") == 0) {
    lua_pushlightuserdata(L, &fsm->com_interface.srv_get_available_transitions);
  } else if (strcmp(name, "GetTransitionGraph") == 0) {
    lua_pushlightuserdata(L, &fsm->com_interface.srv_get_transition_graph);
  } else {
    luaL_error(L, "unknown service type: %s", name);
  }

  return 1;
}


/** List of service methods */
static const struct luaL_Reg lifecycle_methods[] = {
  {"__gc", rcl_lua_lifecycle_free},
  {"is_initialized", rcl_lua_lifecycle_is_initialized},
  {"trigger_transition_by_id", rcl_lua_lifecycle_trigger_by_id},
  {"trigger_transition_by_label", rcl_lua_lifecycle_trigger_by_label},
  {"get_transition_by_label", rcl_lua_lifecycle_get_by_label},
  {"current_state", rcl_lua_lifecycle_get_state},
  {"available_states", rcl_lua_lifecycle_get_available_states},
  {"available_transitions", rcl_lua_lifecycle_get_available_transitions},
  {"transition_graph", rcl_lua_lifecycle_get_transition_graph},
  {"print", rcl_lua_lifecycle_print},
  {"get_service", rcl_lua_lifecycle_get_service},
  {NULL, NULL}
};


/* Add service to library */
void rcl_lua_add_lifecycle_methods (lua_State* L)
{
  /* constructor */
  lua_pushcfunction(L, rcl_lua_lifecycle_init);  // push function
  lua_setfield(L, -2, "new_lifecycle");          // pop, lib['new_lifecycle'] = function

  /* metamethods */
  rcl_lua_utils_add_mt(L, MT_LIFECYCLE, lifecycle_methods);
}
