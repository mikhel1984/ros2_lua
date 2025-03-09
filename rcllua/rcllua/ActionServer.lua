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
local to_str = rclbind.uuid.str

--- Action method wrapper.
--  Execute main process and send result.
--  @param handle ServerGoalHandle object.
--  @param fn Function for execution.
local function exec_and_response (handle, fn)
  local result = fn(handle)
  if handle._result_header then
    local srv = handle._weak.srv
    local resp = srv:get_interface("GetResult").Response()
    resp.status = handle:status()
    resp.result = result
    srv:send_result_response(resp, handle._result_header)
  end
end

--- Default method for handle preprocessing before execution.
local function default_handle_accepted_callback (handle)
  handle:execute()
end

--- Update action server internal state.
--  @param goal ServerGoalHandle.
--  @param ev New event.
function update_state (goal, ev)
  goal._handle:update_goal_state(ev)
  goal._weak.srv:publish_status()
  if not goal._handle:is_active() then
    goal._weak.srv:notify_goal_done()
  end
end


-- ServerGoalHandle class
local ServerGoalHandle = {}
ServerGoalHandle.__index = ServerGoalHandle

--- Start process execution.
function ServerGoalHandle.execute (self)
  update_state(self, rclbind.GoalEvent.EXECUTE)
  assert(coroutine.resume(self._co, self, self._exec))
end

--- Check equality of two goal handles.
--  @param other Second object.
--  @return true when handles have the same ID.
function ServerGoalHandle.__eq (self, other)
  return self.goal_id == other.goal_id
end

--- Check if the goal handle is active.
--  @return true when goal is active.
function ServerGoalHandle.is_active (self)
  return self._handle:is_active()
end

--- Get goal status.
--  @return goal status (enum).
function ServerGoalHandle.status (self)
  return self._handle:get_status()
end

--- Check if the goal is canceled.
--  @return true for canceled goal.
function ServerGoalHandle.is_cancel_requested (self)
  return self:status() == action_msg.GoalStatus.STATUS_CANCELING
end

--- Send feedback to client.
--  @param msg Feedback message.
function ServerGoalHandle.publish_feedback (self, msg)
  local srv = self._weak.srv
  local feedback_msg = srv:get_interface "FeedbackMessage" ()
  feedback_msg.goal_id = self.goal_id
  feedback_msg.feedback = msg
  srv:publish_feedback(feedback_msg)
end

--- Set status 'succeed'.
function ServerGoalHandle.succeed (self)
  update_state(self, rclbind.GoalEvent.SUCCEED)
end

--- Set status 'abort'.
function ServerGoalHandle.abort (self)
  update_state(self, rclbind.GoalEvent.ABORT)
end

--- Set status 'canceled'.
function ServerGoalHandle.canceled (self)
  update_state(self, rclbind.GoalEvent.CANCELED)
end

--- ServerGoalHandle constructor.
--  @param server ActionServer object.
--  @param goal_info Goal info message.
--  @param goal_request Goal request object.
--  @param exec Action process.
--  @return ServerGoalHandle object.
local function new_server_goal_handle (server, goal_info, goal_request, exec)
  local o = {
    _handle = rclbind.new_action_goal_handle(server, goal_info),
    _weak = setmetatable({srv=server}, {__mode="v"}),
    goal_id = goal_info.goal_id,
    request = goal_request,
    _exec = exec,
    _co = nil,
    _result_header = nil,
  }
  o._co = coroutine.create(exec_and_response)
  return setmetatable(o, ServerGoalHandle)
end


-- ActionServer class
ActionServer = {}
ActionServer.__index = ActionServer

--- Check WaitSet for incoming data.
--  @param wait_set WaitSet object.
--  @return table with data or nil.
function ActionServer.take_data (self, wait_set)
  local data = {}
  local srv = self._server
  local is_goal, is_cancel, is_result, is_expired = srv:is_ready(wait_set)
  if is_goal    then data['goal'] = srv:take_goal_request() end
  if is_cancel  then data['cancel'] = srv:take_cancel_request() end
  if is_result  then data['result'] = srv:take_result_request() end
  if is_expired then 
    local n = 0   -- number of handles
    for _ in pairs(self._handles) do n = n + 1 end
    data['expired'] = srv:expire_goals(n) 
  end
  -- check any received elements
  if next(data) then
    return data
  end
  return nil
end

--- Incoming data processing.
--  @param data New requests.
function ActionServer.execute (self, data)
  local srv = self._server
  local tbl = data["goal"]
  if tbl then
    local req, check, header = table.unpack(tbl)
    local uuid_str = to_str(req.goal_id.uuid)
    
    -- send response
    local resp = srv:get_interface("SendGoal").Response()
    local accept = (self._handles[uuid_str] == nil) and check(req)
    resp.accepted = accept
    local stamp = resp.stamp
    local now = self._clock:now()
    stamp.sec = now.sec
    stamp.nanosec = now.nanosec
    srv:send_goal_response(resp, header)

    if accept then
      -- start new process
      local goal_info = action_msg.GoalInfo {goal_id = req.goal_id}
      local handle = new_server_goal_handle(srv, goal_info, req.goal, srv:get_executable())
      self._handles[uuid_str] = handle
      local fn = srv:get_handle_preprocessing() 
      fn(handle)  -- call execution
    end
  end

  tbl = data["cancel"]
  if tb then
    local req, check, header = table.unpack(tbl)
    local resp = srv:process_cancel_request(req)

    local upd = {}  -- check if need canceling
    for _, goal in ipairs(resp.goals_canceling) do
      local handle = self._handles[ to_str(goal.goal_id.uuid) ]
      if handle and check(handle) then
        upd[#upd+1] = goal
        update_state(handle, rclbind.GoalEvent.CANCEL_GOAL)
      end
    end
    resp.goals_canceling(upd)  -- updated list of goals

    srv:send_cancel_response(resp, header)
  end

  tbl = data["result"]
  if tbl then
    local req, _, header = table.unpack(tbl)

    local handle = self._handles[ to_str(req.goal_id.uuid) ]
    if handle then
      handle._result_header = header
    else
      -- no such goal
      local resp = srv:get_interface("GetResult").Response()
      resp.status = action_msg.GoalStatus.STATUS_UNKNOWN
      srv:send_result_response(resp, header)
    end
  end

  tbl = data["expired"]
  if tbl then
    for _, uuid_str in ipairs(tbl) do
      self._handles[uuid_str] = nil
    end
  end
end

--- Get number of entities for wait set.
--  @param 5 numbers.
function ActionServer.get_num_entities (self)
  return self._server:get_num_entities()
end

--- Add action server object to wait set.
function ActionServer.add_to_waitset (self, wait_set)
  self._server:add_to_waitset(wait_set)
end

-- Allow to call ActionServer table.
setmetatable(ActionServer, 
{
--- ActionServer constructor.
--  @param node Source node object.
--  @param action_type Action service type.
--  @param action_name Action service name.
--  @param exec Method for goal processing.
--  @param param Table with QoS, callbacks and result timeout.
--  @return new ActionServer object.
__call = function (self, node, action_type, action_name, exec, param)
  param = param or {}
  param.handle_accepted_callback =
    param.handle_accepted_callback or default_handle_accepted_callback
  local server = rclbind.new_action_server(
    node._node__object, 
    node._clock__object, 
    action_type, action_name, 
    param, 
    exec, 
    action_srv.CancelGoal)
  local o = {
    _server = server,
    _clock = node._clock__object,
    _handles = {},
  }
  node:add_waitable(o)
  return setmetatable(o, self)
end
})

