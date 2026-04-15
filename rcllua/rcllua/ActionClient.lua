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

--- Send cancel request.
--  @param client ActionClient.
--  @param handle ClientGoalHandle object.
--  @return future object.
local function _cancel_goal_async (client, handle)
  local request = action_srv.CancelGoal.Request()
  request.goal_info.goal_id = handle:goal_id()
  local future = new_future(nil, getmetatable(request))
  future._req_id = client._client:send_cancel_request(request,
    function (resp)
      future:_set_result(resp)
      return future
    end)
  return future
end

--- Add condition to weak up and wait for result.
--  @param client ActionClient object.
--  @param future Future object from request.
--  @param timeout_sec (=nil) Time to wait.
local function wait_response (client, future, timeout_sec)
  client._weak.node:wait(
    function () return future._is_done end,
    timeout_sec)
  return future:result()
end

--- Send cancel request and wait for result.
--  @param client ActionClient object.
--  @param handle ClientGoalHandle object.
--  @param timeout_sec (=nil) Time to wait.
--  @return server response or nil.
local function _cancel_goal (client, handle, timeout_sec)
  return wait_response(
    client,
    _cancel_goal_async(client, handle),
    timeout_sec)
end

--- Send result request.
--  @param client ActionClient object.
--  @param handle ClientGoalHandle object.
--  @return future object.
local function _get_result_async (client, handle)
  local request = client._client:get_interface('GetResult').Request()
  request.goal_id = handle:goal_id()
  local future = new_future(nil, getmetatable(request))
  future._req_id = client._client:send_result_request(request,
    function (resp)
      future:_set_result(resp)
      return future
    end)
  return future
end

--- Send result request and wait for response.
--  @param client ActionClient object.
--  @param handle ClientGoalHandle object.
--  @param timeout_sec (=nil) Time to wait.
--  @return response or nil.
local function _get_result (client, handle, timeout_sec)
  return client:_wait_response(
    _get_result_async(client, handle),
    timeout_sec)
end


-- ClientGoalHandle class.
local ClientGoalHandle = {}
ClientGoalHandle.__index = ClientGoalHandle

--- Get goal UUID.
--  @return UUID object.
function ClientGoalHandle.goal_id (self)
  return self._goal_id
end

--- Get response time.
--  @return Time object.
function ClientGoalHandle.stamp (self)
  return self._goal_response.stamp
end

--- Check if the goal is accepted.
--  @return true when accepted
function ClientGoalHandle.accepted (self)
  return self._goal_response.accepted
end

--- Send cancel request.
--  @return Future object.
function ClientGoalHandle.cancel_goal_async (self)
  return _cancel_goal_async(self._action_client, self)
end

--- Send cancel request and wait for response.
--  @param timeout_sec (=nil) Time to wait.
--  @return server response or nil.
function ClientGoalHandle.cancel_goal (self, timeout_sec)
  return _cancel_goal(self._action_client, self, timeout_sec)
end

--- Send result request.
--  @return Future object.
function ClientGoalHandle.get_result_async (self)
  return _get_result_async(self._action_client, self)
end

--- Send result request and wait for response.
--  @param timeout_sec (=nil) Time to wait.
--  @return server response or nil.
function ClientGoalHandle.get_result (self, timeout_sec)
  return _get_result(self._action_client, self, timeout_sec)
end

--- Check UUID equality.
--  @return true when ID are equal.
function ClientGoalHandle.__eq (self, other)
  return self._goal_id == other._goal_id
end

--- ClientGoalHandle constructor.
--  @param action_client Action client object.
--  @param goal_id Goal UUID.
--  @param goal_response Goal response message.
--  @return new ClientGoalHandle object.
local function new_goal_handle (action_client, goal_id, goal_response)
  local o = {
    _action_client = action_client,
    _goal_id = goal_id,
    _goal_response = goal_response,
    _status = action_msg.GoalStatus.STATUS_UNKNOWN,
  }
  local s = rclbind.uuid.str(goal_id.uuid)
  action_client._uuid_handle[s] = o
  return setmetatable(o, ClientGoalHandle)
end


-- ActionClient class
ActionClient = {}
ActionClient.__index = ActionClient

--- Send new goal request to action server.
--  @param goal Goal message.
--  @param cb_feedback (=nil) Function to process feedback messages (optional).
--  @param uuid (=nil) Task UUID (optional).
--  @return Future object.
function ActionClient.send_goal_async (self, goal, cb_feedback, uuid)
  assert(rclbind.is_instance(goal, self._client:get_interface 'Goal'), 'action goal is expected')
  -- prepare message
  local send_goal = self._client:get_interface 'SendGoal'
  local request = send_goal.Request {goal = goal}
  if uuid then
    request.goal_id = uuid       -- UUID message
  else
    request.goal_id.uuid(rclbind.uuid.new())   -- make from array
  end
  -- register callback
  if cb_feedback then
    self._client:set_feedback_method(request.goal_id.uuid, cb_feedback)
  end
  -- prepare 'future'
  local future = new_future(nil, getmetatable(request))
  local goal_uuid = request.goal_id('copy')
  local future_cb = function (msg)
    future:_set_result(new_goal_handle(self, goal_uuid, msg))
    return future
  end
  -- send
  future._req_id = self._client:send_goal_request(request, future_cb)
  return future
end

--- Send new goal request to action server and wait for result.
--  @param goal Goal message.
--  @param cb_feedback (=nil) Function to process feedback messages (optional).
--  @param uuid (=nil) Task UUID (optional).
--  @param timeout_sec (=nil) Time to wait (optional).
--  @return server request or nil.
function ActionClient.send_goal (self, goal, cb_feedback, uuid, timeout_sec)
  return wait_response(
    self,
    ActionClient.send_goal_async(self, goal, cb_feedback, uuid),
    timeout_sec)
end

--- Check if there are available messages.
--  @param wait_set WaitSet object.
--  @return table with incoming data or nil.
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

--- Process incoming data. The method works inside a Lua coroutine.
--  @param data Table with incoming data.
function ActionClient.execute (self, data)
  local value = data['goal']
  if value then
    local resp, cb, seq = table.unpack(value)
    local future = cb(resp)
    if future._callback then
      coroutine.yield(function () future:_callback() end)
    end
  end

  value = data['cancel']
  if value then
    local resp, cb, seq = table.unpack(value)
    local future = cb(resp)
    if future._callback then
      coroutine.yield(function () future:_callback() end)
    end
  end

  value = data['result']
  if value then
    local resp, cb, seq = table.unpack(value)
    local future = cb(resp)
    if future._callback then
      coroutine.yield(function () future:_callback() end)
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
    for _, msg in ipairs(value.status_list) do
      local s = rclbind.uuid.str(msg.goal_info.goal_id.uuid)
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

--- Check if the action server is available.
--  @return true if server is ready.
function ActionClient.server_is_ready (self)
  return self._client:is_action_server_available()
end

--- Sleep until action server become ready.
--  @param timeout_sec (=inf) Wait time (optional).
--  @return true if service is ready.
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

--- Get number of available interfaces.
--  @return 5 numbers.
function ActionClient.get_num_entities (self)
  return self._client:get_num_entities()
end

--- Add action client object to wait set.
function ActionClient.add_to_waitset (self, wait_set)
  self._client:add_to_waitset(wait_set)
end

-- Allow to call ActionClient table.
setmetatable(ActionClient,
{
--- ActionClient constructor.
--  @param node Source node object.
--  @param action_type Action service type.
--  @param action_name Action service name.
--  @param qos (={}) Table with quality of service for each client component.
--  @return new ActionClient object.
__call = function (self, node, action_type, action_name, qos)
  local client = rclbind.new_action_client(
    node._node__object,
    action_type, action_name,
    qos,
    action_srv.CancelGoal, action_msg.GoalStatusArray)
  local o = {
    _client = client,
    _uuid_handle = {},
    _weak = setmetatable({node=node}, {__mode='v'}),
  }
  node:add_waitable(o)
  return setmetatable(o, self)
end
})

