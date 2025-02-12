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
local interfaces = {
  TransitionEvent = lifecycle_msg.TransitionEvent,
  ChangeState = lifecycle_srv.ChangeState,
  GetState = lifecycle_srv.GetState,
  GetAvailableStates = lifecycle_srv.GetAvailableStates,
  GetAvailableTransitions = lifecycle_srv.GetAvailableTransitions,
  GetTransitionGraph = lifecycle_srv.GetAvailableTransitions,
}


LifecycleNode = {}
LifecycleNode.__index = function (t, k)
  return LifecycleNode[k] or ros_node[k]
end

function LifecycleNode.__call (self, ...)
  local node = self(...)
  -- add lifecycle elements
  -- fsm
  local set_com = (self.enable_communication_interface ~= nil)
  node._state__machine = rclbind.new_lifecycle(node._node__object, set_com, interfaces)
  node._lifecycle__callback = {
    [state.TRANSITION_STATE_CONFIGURING] = self.on_configure or LifecycleNode.on_configure,
    [state.TRANSITION_STATE_CLEANINGUP] = self.on_cleanup or LifecycleNode.on_cleanup,
    [state.TRANSITION_STATE_SHUTTINGDOWN] = self.on_shutdown or LifecycleNode.on_shutdown,
    [state.TRANSITION_STATE_ACTIVATING] = self.on_activate or LifecycleNode.on_activate,
    [state.TRANSITION_STATE_DEACTIVATING] = self.on_deactivate or LifecycleNode.on_deactivate,
    [state.TRANSITION_STATE_ERRORPROCESSING] = self.on_error or LifecycleNode.on_error,
  }
  if set_com then
    for nm, msg in pairs(interfaces) do
      if nm ~= TransitionEvent then
        local srv = rclbind.new_service(
          node._node__object, msg, "", fn, nil, node._state__machine:get_service(nm))
        table.insert(node._service__list, srv)
      end
    end
  end
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

return LifecycleNode
