-- Copyright 2025 Stanislav Mikhel
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--     http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

local rclbind = require("rcllua.rclbind")
local ros_node = require("rcllua.Node")

local lifecycle_msg = require("lifecycle_msgs.msg")
local lifecycle_srv = require("lifecycle_msgs.srv")
local ts = lifecycle_msg.Transition

-- List of required services
local services = {
  ChangeState = lifecycle_srv.ChangeState,
  GetState = lifecycle_srv.GetState,
  GetAvailableStates = lifecycle_srv.GetAvailableStates,
  GetAvailableTransitions = lifecycle_srv.GetAvailableTransitions,
  GetTransitionGraph = lifecycle_srv.GetAvailableTransitions,
}

-- List of required messages
local messages = {
  TransitionEvent = lifecycle_msg.TransitionEvent,
  Transition = lifecycle_msg.Transition,
}

--- Default response.
--  @return success
local fn_success = function () return ts.TRANSITION_CALLBACK_SUCCESS end


-- ManagedEntity class
local ManagedEntity = {
  -- default callback definition
  on_configure = fn_success,
  on_cleanup   = fn_success,
  on_shutdown  = fn_success,
  on_error     = fn_success,
  -- change internal state
  on_activate = function (self)
    self.enabled = true
    return ts.TRANSITION_CALLBACK_SUCCESS
  end,
  on_deactivate = function (self)
    self.enabled = false
    return ts.TRANSITION_CALLBACK_SUCCESS
  end,
  is_activated = function (self) return self.enabled end,
}

--- Get object field.
--  @param t Source object.
--  @param k Field name.
--  @return found value.
ManagedEntity.__index = function (t, k)
  return t.entity[k] or ManagedEntity[k]
end

--- ManagedEntity constructor.
--  @param obj Object to store.
--  @return ManagedEntity object.
local function new_managed_entity (obj)
  local o = {
    entity = obj,
    enabled = false,
  }
  return setmetatable(o, ManagedEntity)
end


-- LifecycleNode "private" methods.

--- Calback processing.
--  @param node LifecycleNode object.
--  @param current_id ID of current state.
--  @param prev_state Previous state.
--  @return transition status.
local function execute_callback (node, current_id, prev_state)
  local cb = node._lifecycle__callback[current_id]
  local ok, res = pcall(cb or fn_success, node, prev_state)
  return ok and res or ts.TRANSITION_CALLBACK_ERROR
end

--- Apply the given transition.
--  @param node LifecycleNode object.
--  @param transition_id ID of transition.
--  @return transition result code.
local function change_state (node, transition_id)
  local fsm = node._state__machine
  assert(fsm:is_initialized())
  local init_state = fsm:current_state()
  fsm:trigger_transition_by_id(transition_id, true)
  local curr_state = fsm:current_state()

  local ret = execute_callback(node, curr_state[1], init_state)
  fsm:trigger_transition_by_label(fsm:to_label(ret), true)

  if ret == ts.TRANSITION_CALLBACK_ERROR then
    local error_ret = execute_callback(node, curr_state[1], init_state)
    fsm:trigger_transition_by_label(fsm:to_label(error_ret), true)
  end
  return ret
end

--- Apply transition to the stored ManagedEntity objects.
--  @param node LifecycleNode object.
--  @param cb_name Callback method name.
--  @param state Current state.
--  @return transition result code.
local function transition_callback_impl (node, cb_name, state)
  for _, entity in ipairs(node._managed__entities) do
    local ret = entity[cb_name](entity, state)
    if ret ~= ts.TRANSITION_CALLBACK_SUCCESS then
      return ret
    end
  end
  return ts.TRANSITION_CALLBACK_SUCCESS
end

-- List of lifecycle node services.
local state_srv = {}

--- Do change state.
--  @param node LifecycleNode object.
--  @param req Request with new state (ID or label).
--  @return response with result of operation.
state_srv.ChangeState = function (node, req)
  assert(node._state__machine:is_initialized())
  local req_transition = req.transition
  local transition_id, ok = req_transition.id, true
  if #req_transition.label > 0 then
    ok, transition_id = node._state__machine:get_transition_by_label(req_transition.label)
  end
  local resp = lifecycle_srv.ChangeState.Response()
  resp.success = ok and (change_state(node, transition_id) == ts.TRANSITION_CALLBACK_SUCCESS)
  return resp
end

--- Get current state.
--  @param node LifecycleNode object.
--  @param req Request.
--  @return response with state information.
state_srv.GetState = function (node, req)
  assert(node._state__machine:is_initialized())
  local c = node._state__machine:current_state()
  local resp = lifecycle_srv.GetState.Response()
  resp.current_state {id=c[1], label=c[2]}
  return resp
end

--- Get list of available states.
--  @param node LifecycleNode object.
--  @param req Request.
--  @return response with list of states.
state_srv.GetAvailableStates = function (node, req)
  assert(node._state__machine:is_initialized())
  local acc = {}
  for i, v in ipairs(node._state__machine:available_states()) do
    acc[i] = lifecycle_msg.State {id=v[1], label=v[2]}
  end
  local resp = lifecycle_srv.GetAvailableStates.Response()
  resp.available_states(acc)
  return resp
end

--- Get transitions from the current state.
--  @param node LifecycleNode object.
--  @param req Request.
--  @return response with transitions from the current state.
state_srv.GetAvailableTransitions = function (node, req)
  assert(node._state__machine:is_initialized())
  local acc = {}
  for i, v in ipairs(node._state__machine:available_transitions()) do
    local msg = lifecycle_msg.TransitionDescription()
    msg.transition  {id=v[1], label=v[2]}
    msg.start_state {id=v[3], label=v[4]}
    msg.goal_state  {id=v[5], label=v[6]}
    acc[i] = msg
  end
  local resp = lifecycle_srv.GetAvailableTransitions.Response()
  resp.available_transitions(acc)
  return resp
end

--- Get graph of transitions.
--  @param node LifecycleNode object.
--  @param req Request.
--  @return response with list of transitions.
state_srv.GetTransitionGraph = function (node, req)
  assert(node._state__machine:is_initialized())
  local acc = {}
  for i, v in ipairs(node._state__machine:transition_graph()) do
    local msg = lifecycle_msg.TransitionDescription()
    msg.transition  {id=v[1], label=v[2]}
    msg.start_state {id=v[3], label=v[4]}
    msg.goal_state  {id=v[5], label=v[6]}
    acc[i] = msg
  end
  local resp = lifecycle_srv.GetAvailableTransitions.Response()
  resp.available_transitions(acc)
  return resp
end


--- LifecycleNode class.
LifecycleNode = {}

--- Get class methods.
--  Check LifecycleNode tatable, then Node table.
--  @param k Field name.
--  @return found value.
LifecycleNode.__index = function (_, k)
  return LifecycleNode[k] or ros_node[k]
end

--- Call configure transition.
--  @return transition result code.
function LifecycleNode.trigger_configure (self)
  return change_state(self, ts.TRANSITION_CONFIGURE)
end

--- Call cleanup transition.
--  @return transition result code.
function LifecycleNode.trigger_cleanup (self)
  return change_state(self, ts.TRANSITION_CLEANUP)
end

--- Call shutdown transition.
--  @return transition result code.
function LifecycleNode.trigger_shutdown (self)
  local current = self._state__machine:current_state()
  if current[2] == "unconfigured" then
    return change_state(self, ts.TRANSITION_UNCONFIGURED_SHUTDOWN)
  elseif current[2] == "inactive" then
    return change_state(self, ts.TRANSITION_INACTIVE_SHUTDOWN)
  elseif current[2] == "active" then
    return change_state(self, ts.TRANSITION_ACTIVE_SHUTDOWN)
  end
  error "Shutdown transition not possible"
end

--- Call activate transition.
--  @return transition result code.
function LifecycleNode.trigger_activate (self)
  return change_state(self, ts.TRANSITION_ACTIVATE)
end

--- Call deactivate transition.
--  @return transition result code.
function LifecycleNode.trigger_deactivate (self)
  return change_state(self, ts.TRANSITION_DEACTIVATE)
end

--- Add ManagedEntity object.
--  @param entity Object to add.
function LifecycleNode.add_managed_entity (self, entity)
  assert(getmetatable(entity) == ManagedEntity, "expected ManagedEntity instance")
  table.insert(self._managed__entities, entity)
end

--- Default configure callback.
--  @param state Current state.
--  @return transition result code.
function LifecycleNode.on_configure (self, state)
  return transition_callback_impl(self, 'on_configure', state)
end

--- Default cleanup callback.
--  @param state Current state.
--  @return transition result code.
function LifecycleNode.on_cleanup (self, state)
  return transition_callback_impl(self, 'on_cleanup', state)
end

--- Default shutdown callback.
--  @param state Current state.
--  @return transition result code.
function LifecycleNode.on_shutdown (self, state)
  return transition_callback_impl(self, 'on_shutdown', state)
end

--- Default activate callback.
--  @param state Current state.
--  @return transition result code.
function LifecycleNode.on_activate (self, state)
  return transition_callback_impl(self, 'on_activate', state)
end

--- Default deactivate callback.
--  @param state Current state.
--  @return transition result code.
function LifecycleNode.on_deactivate (self, state)
  return transition_callback_impl(self, 'on_deactivate', state)
end

--- Default error callback.
--  @param state Current state.
--  @return transition result code.
function LifecycleNode.on_error (self, state)
  return transition_callback_impl(self, 'on_error', state)
end

--- Create publisher and add to managed entity list.
--  @param ... Publisher parameters.
--  @return publisher object.
function LifecycleNode.create_lifecycle_publisher (self, ...)
  local pub = self:create_publisher(...)
  local entity = new_managed_entity(pub)
  self:add_managed_entity(entity)
  return entity
end

--- Simplify call of transition codes.
LifecycleNode.TransitionCallbackReturn = {
  SUCCESS = ts.TRANSITION_CALLBACK_SUCCESS,
  ERROR = ts.TRANSITION_CALLBACK_ERROR,
}

--- LifecycleNode constructor.
--  @param ... Constructor parameters.
--  @return initialized object.
function LifecycleNode.__call (self, ...)
  local src = self.node
  -- regular node object
  local node = Node.__call(src, ...)
  -- add lifecycle elements
  local set_com = (src.enable_communication_interface ~= false)
  -- fsm
  node._state__machine = rclbind.new_lifecycle(node._node__object, set_com, services, messages)
  -- lifecycle entities
  node._managed__entities = {}
  -- transition callbacks
  local states = lifecycle_msg.State
  node._lifecycle__callback = {
    [states.TRANSITION_STATE_CONFIGURING]  = src.on_configure  or LifecycleNode.on_configure,
    [states.TRANSITION_STATE_CLEANINGUP]   = src.on_cleanup    or LifecycleNode.on_cleanup,
    [states.TRANSITION_STATE_SHUTTINGDOWN] = src.on_shutdown   or LifecycleNode.on_shutdown,
    [states.TRANSITION_STATE_ACTIVATING]   = src.on_activate   or LifecycleNode.on_activate,
    [states.TRANSITION_STATE_DEACTIVATING] = src.on_deactivate or LifecycleNode.on_deactivate,
    [states.TRANSITION_STATE_ERRORPROCESSING] = src.on_error   or LifecycleNode.on_error,
  }
  -- add services
  if set_com then
    for nm, msg in pairs(services) do
      local srv = rclbind.new_service(
        node._node__object,
        msg,      -- service type
        "",       -- service name, get from lifecycle object
        function (req) return state_srv[nm](node, req) end,  -- callback
        nil,      -- QoS, get from lifecycle object
        node._state__machine:get_service(nm))   -- service, get from lifecycle object
      table.insert(node._service__list, srv)
    end
  end
  return setmetatable(node, LifecycleNode)
end

-- Allow to call LifecycleNode table.
setmetatable(LifecycleNode,
{
--- LifecycleNode class constructor.
--  @param param Table with initialization parameters.
--  @return generator of LifecycleNode object.
__call = function (self, param)
  assert(param and param.name, "'name' must be defined")
  local t = {node=setmetatable(param, ros_node)}
  return setmetatable(t, self)
end
})

-- Access to library.
return {
  lifecycle = LifecycleNode,
  new_managed_entity = new_managed_entity,
}
