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

local states = lifecycle_msg.State
local ts = lifecycle_msg.Transition
local services = {
  ChangeState = lifecycle_srv.ChangeState,
  GetState = lifecycle_srv.GetState,
  GetAvailableStates = lifecycle_srv.GetAvailableStates,
  GetAvailableTransitions = lifecycle_srv.GetAvailableTransitions,
  GetTransitionGraph = lifecycle_srv.GetAvailableTransitions,
}
local messages = {
  TransitionEvent = lifecycle_msg.TransitionEvent,
  Transition = lifecycle_msg.Transition,
}

local ManagedEntity = {
  on_configure = function () return ts.TRANSITION_CALLBACK_SUCCESS end,
  on_cleanup = function () return ts.TRANSITION_CALLBACK_SUCCESS end,
  on_shutdown = function () return ts.TRANSITION_CALLBACK_SUCCESS end,
  on_error = function () return ts.TRANSITION_CALLBACK_SUCCESS end,
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
ManagedEntity.__index = function (t, k)
  return ManagedEntity[k] or t.entity[k]
end

local function new_managed_entity (obj)
  local o = {
    entity = obj,
    enabled = false,
  }
  return setmetatable(o, ManagedEntity)
end

local function check_initialized (node)
  if not node._state__machine:is_initialized() then
    error "Got service request while lifecycle state machine is not initialized"
  end
end

local function execute_callback (node, current_id, prev_state)
  local cb = node._lifecycle__callback[current_id] 
  if cb then
    local ok, res = pcall(cb, previous_state)
    return ok and res or ts.TRANSITION_CALLBACK_ERROR
  end
  return ts.TRANSITION_CALLBACK_SUCCESS
end

local function change_state (node, transition_id)
  check_initialized(node)
  local fsm = node._state__machine
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

local function transition_callback_impl (node, cb_name, state)
  for _, entity in ipairs(node._managed__entities) do
    local ret = entity[cb_name](entity, state)
    if ret ~= ts.TRANSITION_CALLBACK_SUCCESS then
      return ret
    end
  end
  return ts.TRANSITION_CALLBACK_SUCCESS
end

local state_srv = {}

-- on_change_state
state_srv.ChangeState = function (node, req)
  check_initialized(node)
  local req_transition = req.transition
  local transition_id, ok = req_transition.id, true
  if req_transition.label then
    ok, transition_id = node._state__machine:get_transition_by_label(req_transition.label)
  end
  local resp = lifecycle_srv.ChangeState.Response()
  if ok then
    resp.success = (change_state(node, transition_id) == ts.TRANSITION_CALLBACK_SUCCESS)
  else
    resp.success = false
  end
  return resp
end

-- on_get_state
state_srv.GetState = function (node, req)
  check_initialized(node)
  local current = node._state__machine:current_state()
  local resp = lifecycle_srv.GetState.Response()
  resp.current_state {
    id = current[1],
    label = current[2]
  }
  return resp
end

-- on_get_available_states
state_srv.GetAvailableStates = function (node, req)
  check_initialized(node)
  local acc = {}
  for i, v in ipairs(node._state__machine:available_states()) do
    acc[i] = lifecycle_msg.State {id=v[1], label=v[2]}
  end
  local resp = lifecycle_srv.GetAvailableStates.Response()
  resp.available_states(acc)
  return resp
end

-- on_get_available_transitions
state_srv.GetAvailableTransitions = function (node, req)
  check_initialized(node)
  local acc = {}
  for i, v in ipairs(node._state__machine:available_transitions()) do
    local msg = lifecycle_msg.TransitionDescription() 
    msg.transition {id=v[1], label=v[2]}
    msg.start_state {id=v[3], label=v[4]}
    msg.goal_state {id=v[5], label=v[6]}
    acc[i] = msg
  end
  local resp = lifecycle_srv.GetAvailableTransitions.Response()
  resp.available_transitions(acc)
  return resp
end

-- on_get_transition_graph
state_srv.GetTransitionGraph = function (node, req)
  check_initialized(node)
  local acc = {}
  for i, v in ipairs(node._state__machine:transition_graph()) do
    local msg = lifecycle_msg.TransitionDescription() 
    msg.transition {id=v[1], label=v[2]}
    msg.start_state {id=v[3], label=v[4]}
    msg.goal_state {id=v[5], label=v[6]}
    acc[i] = msg
  end
  local resp = lifecycle_srv.GetAvailableTransitions.Response()
  resp.available_transitions(acc)
  return resp
end


LifecycleNode = {}
LifecycleNode.__index = function (t, k)
  return LifecycleNode[k] or ros_node[k]
end

function LifecycleNode.trigger_configure (self)
  return change_state(self, ts.TRANSITION_CONFIGURE)
end

function LifecycleNode.trigger_cleanup (self)
  return change_state(self, ts.TRANSITION_CLEANUP)
end

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

function LifecycleNode.trigger_activate (self)
  return change_state(self, ts.TRANSITION_ACTIVATE)
end

function LifecycleNode.trigger_deactivate (self)
  return change_state(self, ts.TRANSITION_DEACTIVATE)
end

function LifecycleNode.add_managed_entity (self, entity)
  assert(getmetatable(entity) == ManagedEntity, "expected ManagedEntity instance")
  table.insert(self._managed__entities, entity)
end

function LifecycleNode.on_configure (self, state)
  return transition_callback_impl(self, 'on_configure', state)
end

function LifecycleNode.on_cleanup (self, state)
  return transition_callback_impl(self, 'on_cleanup', state)
end

function LifecycleNode.on_shutdown (self, state)
  return transition_callback_impl(self, 'on_shutdown', state)
end

function LifecycleNode.on_activate (self, state)
  return transition_callback_impl(self, 'on_activate', state)
end

function LifecycleNode.on_deactivate (self, state)
  return transition_callback_impl(self, 'on_deactivate', state)
end

function LifecycleNode.on_error (self, state)
  return transition_callback_impl(self, 'on_error', state)
end

function LifecycleNode.create_lifecycle_publisher (self, ...)
  local pub = self:create_publisher(...)
  self:add_managed_entity(new_managed_entity(pub))
  return pub
end

LifecycleNode.TransitionCallbackReturn = {
  SUCCESS = ts.TRANSITION_CALLBACK_SUCCESS,
  ERROR = ts.TRANSITION_CALLBACK_ERROR,
}

function LifecycleNode.__call (self, ...)
  local src = self.node
  local node = Node.__call(src, ...)
  -- add lifecycle elements
  -- fsm
  local set_com = (src.enable_communication_interface ~= nil)
  node._state__machine = rclbind.new_lifecycle(node._node__object, set_com, services, messages)
  node._managed__entities = {}
  node._lifecycle__callback = {
    [states.TRANSITION_STATE_CONFIGURING] = src.on_configure or LifecycleNode.on_configure,
    [states.TRANSITION_STATE_CLEANINGUP] = src.on_cleanup or LifecycleNode.on_cleanup,
    [states.TRANSITION_STATE_SHUTTINGDOWN] = src.on_shutdown or LifecycleNode.on_shutdown,
    [states.TRANSITION_STATE_ACTIVATING] = src.on_activate or LifecycleNode.on_activate,
    [states.TRANSITION_STATE_DEACTIVATING] = src.on_deactivate or LifecycleNode.on_deactivate,
    [states.TRANSITION_STATE_ERRORPROCESSING] = src.on_error or LifecycleNode.on_error,
  }
--  if set_com then
--    -- add services
--    for nm, msg in pairs(services) do
--      local srv = rclbind.new_service(
--        node._node__object,
--        msg,      -- service type
--        "",       -- service name, get from lifecycle object
--        function (req) return state_srv[nm](node, req) end,  -- callback
--        nil,      -- QoS, get from lifecycle object
--        node._state__machine:get_service(nm))   -- service, get from lifecycle object
--      table.insert(node._service__list, srv)
--    end
--  end
  return setmetatable(node, LifecycleNode)
end

setmetatable(LifecycleNode, 
{
__call = function (self, param)
  assert(param and param.name, "'name' must be defined")
  local t = {node=setmetatable(param, ros_node)}
  return setmetatable(t, self)
end
})

return {
  lifecycle = LifecycleNode,
  new_managed_entity = new_managed_entity,
} 
