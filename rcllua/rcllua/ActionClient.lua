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

local action_msg = require("action_msgs.msg")
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

function ClientGoalHandle.cancel_goal_async (self)
  return self._action_client:_cancel_goal_async(self)
end

function ClientGoalHandle.get_result_async (self)
  return self._action_client:_get_result_async(self)
end

function ClientGoalHandle.__eq (self, other)
  return self._goal_id == other._goal_id
end


local function new_goal_handle (action_client, goal_id, goal_response)
  local o = {
    _action_client = action_client,
    _goal_id = goal_id,
    _goal_response = goal_response,
    _status = action_msg.GoalStatus.STATUS_UNKNOWN,
  }
  local s = rclbind.uuid_to_str(goal_id.uuid)
  action_client._uuid_handle[s] = o
  return setmetatable(o, ClientGoalHandle)
end


-- ActionClient class
ActionClient = {}
ActionClient.__index = ActionClient

function ActionClient.send_goal_async (self, goal, cb_feedback, uuid)
  assert(rclbind.is_instance(goal, self._client:get_interface 'Goal'), 'action goal is expected')
  -- prepare message
  local send_goal = self._client:get_interface 'SendGoal'
  local request = send_goal.Request {goal = goal}
  if uuid then
    request.goal_id = uuid       -- UUID message
  else
    request.goal_id.uuid(rclbind.get_uuid())   -- make from array
  end
  -- register callback
  if cb_feedback then 
    self._client:set_feedback_method(request.goal_id.uuid, cb_feedback) 
  end
  -- prepare 'future'
  local future = new_future(nil, getmetatable(request))
  local goal_uuid = request.goal_id()
  local future_cb = function (msg)
    future:_set_result(new_goal_handle(self, goal_uuid, msg))
    return future
  end
  -- send
  future._req_id = self._client:send_goal_request(request, future_cb)
  return future
end

function ActionClient.take_data (self, wait_set)
  local is_feedback, is_status, is_goal, is_cancel, is_result = self._client:is_ready(wait_set)
  local data = {}
  if is_feedback then data['feedback'] = self._client:take_feedback() end
  if is_status then data['status'] = self._client:take_status() end
  if is_goal then data['goal'] = self._client:take_goal_response() end
  if is_cancel then data['cancel'] = self._client:take_cancel_response() end
  if is_result then data['result'] = self._client:take_result_response() end
  -- check any received elements
  if next(data) then
    return data
  end
  return nil
end

function ActionClient.execute (self, data)
  local value = data['goal']
  if value then
    local resp, cb, seq = table.unpack(value)
    local future = cb(resp)  -- make handle
    if future._callback then
      coroutine.yield(function () future._callback(future) end)
    end
  end

  value = data['cancel']
  if value then
    local resp, cb, seq = table.unpack(value)
    local future = cb(resp)
    if future._callback then
      coroutine.yield(function () future._callback(future) end)
    end
  end

  value = data['result']
  if value then
    local resp, cb, seq = table.unpack(value)
    local future = cb(resp)
    if future._callback then
      coroutine.yield(function () future._callback(future) end)
    end
  end

  value = data['feedback']
  if value then
    local msg, fn = table.unpack(value)
    coroutine.yield(function () fn(msg) end)
  end

  value = data['status']
  if value then
    local goal_status = action_msg.GoalStatus
    local status_list = value.status_list
    for _, msg in ipairs(status_list) do
      local s = rclbind.uuid_to_str(msg.goal_info.goal_id.uuid)
      local handle = self._uuid_handle[s]
      if handle then
        local status = msg.status
        handle._status = status
        if goal_status.STATUS_SUCCEEDED == status or 
           goal_status.STATUS_CANCELED == status or 
           goal_status.STATUS_ABORTED == status
        then
          self._uuid_handle[s] = nil
          -- TODO remove feedback and result
        end
      end
    end
  end
end

function ActionClient._cancel_goal_async (self, handle)
  local request = action_srv.CancelGoal.Request()
  request.goal_info.goal_id = handle:goal_id()
  local future = new_future(nil, getmetatable(request))
  local future_cb = function (resp)
    future:_set_result(resp)
    return future
  end
  future._req_id = self._client:send_cancel_request(request, future_cb)
  return future
end

function ActionClient._get_result_async (self, handle)
  local srv = self._client:get_interface 'GetResult'
  local request = srv.Request()
  request.goal_id = handle:goal_id()
  local future = new_future(nil, getmetatable(request))
  local future_cb = function (resp)
    future:_set_result(resp)
    return future
  end
  future._req_id = self._client:send_result_request(request, future_cb)
  return future
end

function ActionClient.server_is_ready (self)
  return self._client:is_action_server_available()
end

function ActionClient.wait_for_server (self, timeout_sec)
  local sleep_time = math.min(0.2, timeout_sec or 1.0)
  timeout_sec = timeout_sec or math.huge
  while rclbind.context_ok() and not self._client:is_action_server_available()
    and timeout_sec > 0
  do
    rclbind.sleep_thread(sleep_time)
    timeout_sec = timeout_sec - sleep_time
  end
  return self._client:is_action_server_available()
end

function ActionClient.get_num_entities (self)
  return self._client:get_num_entities()
end

function ActionClient.add_to_waitset (self, wait_set)
  self._client:add_to_waitset(wait_set)
end


setmetatable(ActionClient, 
{
__call = function (self, node, action_type, action_name, qos)
  local client = rclbind.new_action_client(
    node._node__object, action_type, action_name, qos,
    action_srv.CancelGoal, action_msg.GoalStatusArray)
  local o = {
    _client = client,  
    _uuid_handle = {}
  }
  node:add_waitable(o)
  return setmetatable(o, self)
end
})


