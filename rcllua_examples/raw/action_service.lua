-- rclbind example
-- Make simple action service

local rclbind = require("rcllua.rclbind")
local Fibonacci = require("action_tutorials_interfaces.action").Fibonacci
local action_msg = require("action_msgs.msg")
local action_srv = require("action_msgs.srv")

-- Init ROS environment
rclbind.context_init(arg)

-- Keep references
local process, yielded = {}, {}

-- Main process
function action_exec (req, srv, handle)
  -- feedback message with goal information
  local feedback_full = Fibonacci.FeedbackMessage()
  feedback_full.goal_id = req.goal_id
  local feedback_msg = feedback_full.feedback
  -- store results into Lua table
  local sequence = {0, 1}

  -- processing
  local goal = req.goal
  for i = 1, goal.order do
    sequence[#sequence+1] = sequence[#sequence] + sequence[#sequence-1]
    feedback_msg.partial_sequence(sequence)
    srv:publish_feedback(feedback_full)
    sleep(1.0)  -- yield
  end

  update_state(rclbind.GoalEvent.SUCCEED, handle)

  local result = Fibonacci.Result()
  result.sequence(sequence)
  return result
end

-- Make objects
local node = rclbind.new_node('raw_action_service')
local clock = rclbind.new_clock()
local act_srv = rclbind.new_action_server(
  node, clock, Fibonacci, 'fibonacci', {},
  action_exec, action_srv.CancelGoal)

-- Change goal status
function update_state (ev, handle)
  handle:update_goal_state(ev)
  act_srv:publish_status()
  if not handle.is_active() then
    act_srv:notify_goal_done()
  end
end

-- Stop execution, call yield
function sleep (timeout)
  local co = coroutine.running()
  -- wake up time
  yielded[co] = clock:now() + rclbind.new_duration_sec(timeout)
  coroutine.yield()
  yielded[co] = nil
end

-- Set message time
function set_time (msg)
  local now = clock:now()
  msg.stamp.sec = now.sec
  msg.stamp.nanosec = now.nanosec
end

-- Prepare WaitSet
local sub_no, guard_no, timer_no, cli_no, srv_no = act_srv:get_num_entities()
local wait_set = rclbind.new_wait_set(sub_no, guard_no, timer_no, cli_no, srv_no, 0)
local time_ns = math.floor(0.1 * 1E9)  -- wait time, nanoseconds

-- Main loop
while rclbind.context_ok() do
  -- prepare
  wait_set:clear()
  act_srv:add_to_waitset(wait_set)

  wait_set:wait(time_ns)

  -- check input
  local data = {}
  local is_goal, is_cancel, is_result, is_expired = act_srv:is_ready(wait_set)
  if is_goal    then data['goal'] = act_srv:take_goal_request() end
  if is_cancel  then data['cancel'] = act_srv:take_cancel_request() end
  if is_result  then data['result'] = act_srv:take_result_request() end
  if is_expired then
    local n = 0   -- number of handles
    for _ in pairs(process) do n = n + 1 end
    data['expired'] = act_srv:expire_goals(n)
  end

  -- new goal
  if data['goal'] then
    local req, _, header = table.unpack(data["goal"])
    local uuid_str = rclbind.uuid.str(req.goal_id.uuid)
    local resp = act_srv:get_interface("SendGoal").Response()
    resp.accepted = (process[uuid_str] == nil)  -- ignore existed
    set_time(resp)
    act_srv:send_goal_response(resp, header)

    if not process[uuid_str] then
      -- add for processing
      local goal_info = action_msg.GoalInfo {goal_id = req.goal_id}
      goal_info.stamp = resp.stamp
      local proc = {}
      proc.handle = rclbind.new_action_goal_handle(act_srv, goal_info)
      proc.co = coroutine.create(act_srv:get_executable())
      update_state(rclbind.GoalEvent.EXECUTE, proc.handle)
      process[uuid_str] = proc
      coroutine.resume(proc.co,
                       req, act_srv, proc.handle)
    end
  end

  -- cancel the process
  if data["cancel"] then
    local req, _, header = table.unpack(data["cancel"])
    local response = act_srv:process_cancel_request(req)
    local upd = {}
    for _, goal in ipairs(response.goals_canceling) do
      local uuid_str = rclbind.uuid.str(goal.goal_id.uuid)
      if process[uuid_str] then
        upd[#upd+1] = goal
        update_state(rclbind.GoalEvent.CANCEL_GOAL, process[uuid_str].handle)
      end
    end
    response.goals_canceling(upd)  -- updated list of goals
    act_srv:send_cancel_response(response, header)
  end

  -- result of the process
  if data["result"] then
    local req, _, header = table.unpack(data["result"])
    local uuid_str = rclbind.uuid.str(req.goal_id.uuid)
    if process[uuid_str] then
      process[uuid_str].result = header
    else
      -- no such goal
      local resp = act_srv:get_interface("GetResult").Response()
      resp.status = action_msg.GoalStatus.STATUS_UNKNOWN
      act_srv:send_result_response(resp, header)
    end
  end

  -- remove old goals
  if data["expired"] then
    local lst = data["expired"]
    for _, uuid_str in ipairs(data["expired"]) do
      process[uuid_str] = nil
    end
  end

  -- dispatch processes
  for id, t in pairs(process) do
    local co = t.co
    if coroutine.status(co) == 'dead' then
      process[id] = nil
    elseif yielded[co] and yielded[co] <= clock:now() then
      -- wake up and execute
      local ok, res = coroutine.resume(co)
      if ok and coroutine.status(co) == 'dead' then
        local resp = act_srv:get_interface("GetResult").Response()
        resp.status = t.handle:get_status()
        resp.result = res
        act_srv:send_result_response(resp, t.result)
      end
    end
  end

end

-- Shutdown ROS context
rclbind.context_shutdown()

