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

#include <rosidl_runtime_c/message_type_support_struct.h>
#include <rosidl_runtime_c/service_type_support_struct.h>

#include <rosidl_luacommon/definition.h>

#include "rcllua/lifecycle.h"
#include "rcllua/node.h"
#include "rcllua/qos.h"
#include "rcllua/utils.h"

#define MAYBE_NULL(X) (X) ? (X) : ""

/** Save bindings in register. */
enum FsmReg
{
  /** node reference */
  FSM_REG_NODE = 1,
  /** labels */
  FSM_REG_LABEL = 2,
  /** number of elements + 1 */
  FSM_REG_NUMBER
};

/** Lifecycle object metatable name. */
const char * MT_LIFECYCLE = "ROS2.Lifecycle";

/**
 * Get information about typesupport from the interface table.
 *
 * \param[inout] L Lua stack.
 * \param[in] tbl Interface table position number.
 * \param[in] nm Interface name.
 * \return typesupport reference.
 */
static void * rcl_lua_lifecycle_get_typesupport(lua_State * L, int tbl, const char * nm)
{
  void * ts = NULL;
  /* get table */
  if (lua_getfield(L, tbl, nm) == LUA_TTABLE) {   // push table
    if (ROSIDL_LUA_PUSH_TYPESUPPORT(L, -1) == LUA_TLIGHTUSERDATA) {  // push typesupport
      ts = lua_touserdata(L, -1);
    }
    lua_pop(L, 1);  // pop typesupport
  }
  if (NULL == ts) {
    luaL_error(L, "not found type support for %s", nm);
  }
  lua_pop(L, 1);  // pop table

  return ts;
}

/**
 * Fill map from name to index for transition results.
 * Save result table on the stack.
 *
 * \param[inout] L Lua stack.
 * \param[in] tbl Interface position on the table.
 */
static void rcl_lua_lifecycle_push_labels(lua_State * L, int tbl)
{
  lua_createtable(L, 3, 0);    // push table
  if (lua_getfield(L, tbl, "Transition") == LUA_TNIL) {  // push interface
    luaL_error(L, "not found table 'Transition'");
  }
  /* keys */
  const char * names[3] = {
    "TRANSITION_CALLBACK_SUCCESS",
    "TRANSITION_CALLBACK_FAILURE",
    "TRANSITION_CALLBACK_ERROR"
  };
  /* values */
  const char * labels[3] = {
    rcl_lifecycle_transition_success_label,
    rcl_lifecycle_transition_failure_label,
    rcl_lifecycle_transition_error_label
  };
  /* fill map */
  for (int i = 0; i < 3; i++) {
    if (lua_getfield(L, -1, names[i]) == LUA_TNIL) {  // push number
      luaL_error(L, "not found '%'", names[i]);
    }
    lua_pushstring(L, labels[i]);  // push string
    lua_rawset(L, -4);             // pop number and string
  }
  lua_pop(L, 1);  // pop interface
}

/**
 * Create state machine object.
 *
 * Table: rclbind
 * Method: new_lifecycle
 *
 * Arguments:
 * - node object
 * - com interface state flag (=true)
 * - table with required services
 * - table with required messages
 *
 * Return:
 * - state machine object
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_lifecycle_init(lua_State * L)
{
  /* arg1 - node */
  rcl_node_t * node = luaL_checkudata(L, 1, MT_NODE);
  /* arg2 - interface flag */
  luaL_argcheck(L, lua_isboolean(L, 2), 2, "com interface state expected");
  /* arg3 - service tables */
  luaL_argcheck(L, lua_istable(L, 3), 3, "table with services is expected");
  /* arg4 - message tables */
  luaL_argcheck(L, lua_istable(L, 4), 4, "table with messages is expected");

  /* get typesupport */
  rosidl_message_type_support_t * ts_pub_notify =
    rcl_lua_lifecycle_get_typesupport(L, 4, "TransitionEvent");
  rosidl_service_type_support_t * ts_srv_change_state =
    rcl_lua_lifecycle_get_typesupport(L, 3, "ChangeState");
  rosidl_service_type_support_t * ts_srv_get_state =
    rcl_lua_lifecycle_get_typesupport(L, 3, "GetState");
  rosidl_service_type_support_t * ts_srv_get_available_states =
    rcl_lua_lifecycle_get_typesupport(L, 3, "GetAvailableStates");
  rosidl_service_type_support_t * ts_srv_get_available_transitions =
    rcl_lua_lifecycle_get_typesupport(L, 3, "GetAvailableTransitions");
  rosidl_service_type_support_t * ts_srv_get_transition_graph = ts_srv_get_available_transitions;

  /* init object */
  rcl_lifecycle_state_machine_t *fsm = lua_newuserdata(L, sizeof(rcl_lifecycle_state_machine_t));
  *fsm = rcl_lifecycle_get_zero_initialized_state_machine();

  rcl_lifecycle_state_machine_options_t ops = rcl_lifecycle_get_default_state_machine_options();
  ops.enable_com_interface = lua_toboolean(L, 2);

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
  lua_createtable(L, FSM_REG_NUMBER - 1, 0);  // push table a
  lua_pushvalue(L, 1);                // push node
  lua_rawseti(L, -2, FSM_REG_NODE);   // pop node

  /* transition results - enum */
  rcl_lua_lifecycle_push_labels(L, 4);   // push table b (enum)
  lua_rawseti(L, -2, FSM_REG_LABEL);     // pop table b

  lua_rawsetp(L, LUA_REGISTRYINDEX, fsm);  // pop table a

  return 1;
}

/**
 * State machine destructor.
 *
 * Arguments:
 * - state machine object
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_lifecycle_free(lua_State * L)
{
  /* arg1 - state machine */
  rcl_lifecycle_state_machine_t * fsm = lua_touserdata(L, 1);

  /* get node */
  lua_rawgetp(L, LUA_REGISTRYINDEX, fsm);  // push table
  lua_rawgeti(L, -1, FSM_REG_NODE);        // push node
  rcl_node_t * node = lua_touserdata(L, -1);

  rcl_ret_t ret = rcl_lifecycle_state_machine_fini(fsm, node);
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to fini lifecycle state machine");
  }

  /* free dependencies */
  lua_pushnil(L);
  lua_rawsetp(L, LUA_REGISTRYINDEX, fsm);

  return 0;
}

/**
 * Check if the state machine is initialized.
 *
 * Table: LifecycleNode
 * Method: is_initialized
 *
 * Arguments:
 * - state machine object
 *
 * Return
 * - true when initialized
 * - optional error message for false
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_lifecycle_is_initialized(lua_State * L)
{
  /* arg1 - state machine */
  rcl_lifecycle_state_machine_t * fsm = lua_touserdata(L, 1);

  rcl_ret_t ret = rcl_lifecycle_state_machine_is_initialized(fsm);
  if (RCL_RET_OK == ret) {
    lua_pushboolean(L, true);
    return 1;
  }

  lua_pushboolean(L, false);
  lua_pushliteral(L, "Got service request while lifecycle state machine is not initialized");
  return 2;
}

/**
 * Trigger transition by ID.
 *
 * Table: LifecycleNode
 * Method: trigger_transition_by_id
 *
 * Arguments:
 * - state machine
 * - transition id
 * - publish flag
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_lifecycle_trigger_by_id(lua_State * L)
{
  /* arg1 - state machine */
  rcl_lifecycle_state_machine_t * fsm = luaL_checkudata(L, 1, MT_LIFECYCLE);
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

/**
 * Trigger transition by label.
 *
 * Table: LifecycleNode
 * Method: trigger_transition_by_label
 *
 * Arguments:
 * - state machine
 * - transition label
 * - publish flag
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_lifecycle_trigger_by_label(lua_State * L)
{
  /* arg1 - state machine */
  rcl_lifecycle_state_machine_t * fsm = luaL_checkudata(L, 1, MT_LIFECYCLE);
  /* arg2 - label */
  const char * label = luaL_checkstring(L, 2);
  /* arg3 - publish flag */
  luaL_argcheck(L, lua_isboolean(L, 3), 3, "boolean expected");

  rcl_ret_t ret = rcl_lifecycle_trigger_transition_by_label(fsm, label, lua_toboolean(L, 3));
  if (RCL_RET_OK != ret) {
    luaL_error(L, "failed to trigger lifecycle state machine transition");
  }

  return 0;
}

/**
 * Get state transition by label.
 *
 * Table: LifecycleNode
 * Method: get_transition_by_label
 *
 * Arguments:
 * - state machine
 * - transition label
 *
 * Return:
 * - success of operation
 * - transition ID or error message
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_lifecycle_get_by_label(lua_State * L)
{
  /* arg1 - state machine */
  rcl_lifecycle_state_machine_t * fsm = luaL_checkudata(L, 1, MT_LIFECYCLE);
  /* arg2 - label */
  const char * label = luaL_checkstring(L, 2);

  const rcl_lifecycle_transition_t * transition =
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

/**
 * Get current state.
 *
 * Table: LifecycleNode
 * Method: current_state
 *
 * Arguments:
 * - state machine
 *
 * Return:
 * - table {id, label}
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_lifecycle_get_state(lua_State * L)
{
  /* arg1 - state machine */
  rcl_lifecycle_state_machine_t * fsm = luaL_checkudata(L, 1, MT_LIFECYCLE);

  lua_createtable(L, 2, 0);      // push table
  lua_pushinteger(L, fsm->current_state->id);  // push int
  lua_rawseti(L, -2, 1);         // pop int
  lua_pushstring(L, fsm->current_state->label);  // push string
  lua_rawseti(L, -2, 2);         // pop string

  return 1;
}

/**
 * Get available system states.
 *
 * Table: LifecycleNode
 * Method: available_states
 *
 * Arguments:
 * - state machine
 *
 * Return:
 * - table of pairs
 * -- id
 * -- label
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_lifecycle_get_available_states(lua_State * L)
{
  /* arg1 - state machine */
  rcl_lifecycle_state_machine_t * fsm = luaL_checkudata(L, 1, MT_LIFECYCLE);

  lua_createtable(L, fsm->transition_map.states_size, 0);  // push table a
  for (size_t i = 0; i < fsm->transition_map.states_size; i++) {
    lua_createtable(L, 2, 0);     // push table b
    lua_pushinteger(L, fsm->transition_map.states[i].id);  // push int
    lua_rawseti(L, -2, 1);        // pop in
    lua_pushstring(L, MAYBE_NULL(fsm->transition_map.states[i].label));  // push string
    lua_rawseti(L, -2, 2);        // pop string
    lua_rawseti(L, -2, i + 1);      // pop table b
  }

  return 1;
}

/**
 * Get available transitions.
 *
 * Table: LifecycleNode
 * Method: available_transitions
 *
 * Arguments:
 * - state machine
 *
 * Return:
 * - table of groups
 * -- transition id
 * -- transigion label
 * -- start id
 * -- start label
 * -- goal id
 * -- goal label
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_lifecycle_get_available_transitions(lua_State * L)
{
  /* arg1 - state machine */
  rcl_lifecycle_state_machine_t * fsm = luaL_checkudata(L, 1, MT_LIFECYCLE);

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
    lua_rawseti(L, -2, i + 1);    // pop table b
  }

  return 1;
}

/**
 * Get transition graph.
 *
 * Table: LifecycleNode
 * Method: transition_graph
 *
 * Arguments:
 * - state machine
 *
 * Return:
 * - table of groups
 * -- transition id
 * -- transigion label
 * -- start id
 * -- start label
 * -- goal id
 * -- goal label
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_lifecycle_get_transition_graph(lua_State * L)
{
  /* arg1 - state machine */
  rcl_lifecycle_state_machine_t * fsm = luaL_checkudata(L, 1, MT_LIFECYCLE);

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
    lua_rawseti(L, -2, i + 1);
  }

  return 1;
}

/**
 * Print state machine to console.
 *
 * Table: LifecycleNode
 * Method: print
 *
 * Arguments:
 * - state machine
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_lifecycle_print(lua_State * L)
{
  /* arg1 - state machine */
  rcl_lifecycle_state_machine_t * fsm = luaL_checkudata(L, 1, MT_LIFECYCLE);

  rcl_print_state_machine(fsm);

  return 0;
}

/**
 * Get service pointer of the given type.
 *
 * Table: LifecycleNode
 * Method: get_service
 *
 * Arguments:
 * - state machine
 * - service name
 *
 * Return:
 * - service as lightuserdata
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_lifecycle_get_service(lua_State * L)
{
  /* arg1 - state machine */
  rcl_lifecycle_state_machine_t * fsm = luaL_checkudata(L, 1, MT_LIFECYCLE);
  /* arg2 - service name */
  const char * name = luaL_checkstring(L, 2);

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

/**
 * Get label for the given return code.
 *
 * Table: LifecycleNode
 * Method: to_label
 *
 * Arguments:
 * - state machine object
 * - code value
 *
 * Return:
 * - code label or nil
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_lifecycle_to_label(lua_State * L)
{
  /* arg1 - state machine */
  rcl_lifecycle_state_machine_t * fsm = luaL_checkudata(L, 1, MT_LIFECYCLE);
  /* arg2 - return code */
  luaL_argcheck(L, lua_isinteger(L, 2), 2, "return code is expected");

  /* get table */
  lua_rawgetp(L, LUA_REGISTRYINDEX, fsm);  // push table
  lua_rawgeti(L, -1, FSM_REG_LABEL);       // push labels

  lua_pushvalue(L, 2);                     // push, copy code
  lua_rawget(L, -2);                       // pop code, push value

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
  {"to_label", rcl_lua_lifecycle_to_label},
  {NULL, NULL}
};

/* Add service to library */
void rcl_lua_add_lifecycle_methods(lua_State * L)
{
  /* constructor */
  lua_pushcfunction(L, rcl_lua_lifecycle_init);  // push function
  lua_setfield(L, -2, "new_lifecycle");          // pop, lib['new_lifecycle'] = function

  /* metamethods */
  rcl_lua_utils_add_mt(L, MT_LIFECYCLE, lifecycle_methods);
}
