-- rclbind example
-- Make simple action service

local rclbind = require("rcllua.rclbind")
local Fibonacci = require("action_tutorials_interfaces.action").Fibonacci

local action_msg = require("action_msgs.msg")

-- Init ROS environment
rclbind.context_init(arg)

local process, handles = {}, {}
local yielded, results = {}, {}

local clock = rclbind.new_clock()

local function update_state (ev, handle, srv)
  handle:update_goal_state(ev)
  srv:publish_status()
  if not handle.is_active() then
    srv:notify_goal_done()
  end
end

local function sleep (timeout)
  local co, _ = coroutine.running()
  -- wake up time
  yielded[co] = clock:now() + rclbind.new_duration_sec(timeout)
  coroutine.yield()
  yielded[co] = nil
end


local function action_exec (goal, srv, handle, uuid)
  local feedback_full = Fibonacci.FeedbackMessage()
  feedback_full.goal_id = uuid
  local feedback_msg = feedback_full.feedback
  local sequence = {0, 1}

  for i = 1, goal.order do
    sequence[#sequence+1] = sequence[#sequence] + sequence[#sequence-1]
    feedback_msg.partial_sequence(sequence)
    srv:publish_feedback(feedback_full)
    sleep(1.0)  -- yield
  end

  update_state(rclbind.GoalEvent.SUCCEED, handle, srv)

  local result = Fibonacci.Result()
  result.sequence(sequence)
  return result
end

-- Make objects
local node = rclbind.new_node('raw_action_service')
local act_srv = rclbind.new_action_server(node, clock, Fibonacci, 'fibonacci', {}, action_exec)

local sub_no, guard_no, timer_no, cli_no, srv_no = act_srv:get_num_entities()
local wait_set = rclbind.new_wait_set(sub_no, guard_no, timer_no, cli_no, srv_no, 0)
local time_ns = math.floor(0.1 * 1E9)  -- to nanoseconds

-- Main loop
while rclbind.context_ok() do  
  -- prepare
  wait_set:clear()
  act_srv:add_to_waitset(wait_set)

  wait_set:wait(time_ns)
  
  local is_goal, is_cancel, is_result, is_expired = act_srv:is_ready(wait_set)
  
  local data = {}
  if is_goal then data['goal'] = act_srv:take_goal_request() end
  if is_cancel then data['cancel'] = act_srv:take_cancel_request() end
  if is_result then data['result'] = act_srv:take_result_request() end
  --if is_expired then data['expired'] = act_srv:expire_goals() end

  if data['goal'] then
    print "goal"
    local req, _, header = table.unpack(data["goal"])
    print(req)
    local uuid_str = rclbind.uuid_to_str(req.goal_id.uuid)
    local resp_interface = act_srv:get_interface "SendGoal"
    local resp = resp_interface.Response()
    resp.accepted = process[uuid_str] ~= nil
    local now = clock:now()
    resp.stamp.sec = now.sec
    resp.stamp.nanosec = now.nanosec
    act_srv:send_goal_response(resp, header)

    if not process[uuid_str] then
      local co = coroutine.create(act_srv:get_executable())
      process[uuid_str] = co
      handles[uuid_str] = rclbind.new_action_goal_handle(act_srv, action_msg.GoalInfo())
      update_state(rclbind.GoalEvent.EXECUTE, handles[uuid_str], act_srv)
      local v, err = coroutine.resume(co, req.goal, act_srv, handles[uuid_str], req.goal_id)
      print(v, err)
    end
  end
--
--  if data["cancel"] then
--  end
--
--  if data["result"] then
--    local req, _, header = table.unpack(data["result"])
--    local uuid_str = rclbind.uuid_to_str(req.goal_id.uuid)
--    if goals[uuid_str] then
--      results[uuid_str] = header
--    else
--      local resp_interface = act_srv:get_interface "GetResult"
--      local resp = resp_interface.Response()
--      resp.status = action_msg.GoalStatus.STATUS_UNKNOWN
--      act_srv:send_result_response(resp, header)
--    end
--  end
--
--  if data["expired"] then
--  end
--
  for id, co in pairs(process) do
    if coroutine.status(co) == 'dead' then
      process[id] = nil
      handles[id] = nil
    elseif yielded[co] and yielded[co] <= clock:now() then
      local ok, res = coroutine.resume(co)
    end
  end

end

-- Shutdown ROS context
rclbind.context_shutdown()

