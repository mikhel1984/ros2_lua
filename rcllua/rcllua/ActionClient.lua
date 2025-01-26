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

local action_msgs = require("action_msgs.msg")
local action_srv = require("action_msgs.srv")

local rclbind = require("rcllua.rclbind")
local new_future = require("rcllua.client").new_future


-- ClientGoalHandle class
local ClientGoalHandle = {}
ClientGoalHandle.__index = ClientGoalHandle

function ClientGoalHandle.goal_id (self)
  return self._goal_id
end

function ClientGoalHandle.stamp (self)
  return self._goal_response.stamp
end

function ClientGoalHandle.accepted (self)
  return self._goal_response.accepted
end

function ClientGoalHandle.__eq (self, other)
  return self._goal_id == other._goal_id
end

local function new_goal_handle (action_client, goal_id, goal_response)
  local o = {
    _action_client = action_client,
    _goal_id = goal_id,
    _goal_response = goal_response,
    _status = action_msgs.GoalStatus.STATUS_UNKNOWN,
  }
  return setmetatable(o, ClientGoalHandle)
end


-- ActionClient class
ActionClient = {}
ActionClient.__index = ActionClient


setmetatable(ActionClient, 
{
__call = function (self, node, action_type, action_name, qos)
  local client = rclbind.new_action_client(
    node._node__object, action_type, action_name, 
    action_srv.CancelGoal, action_msg.GoalStatusArray)
  node:add_waitable(client)
  local o = {
    _client = client,  
  }
  return setmetatable(o, self)
end
})


