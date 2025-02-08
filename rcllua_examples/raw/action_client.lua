-- rclbind example
-- Make simple action client

local rclbind = require("rcllua.rclbind")
local Fibonacci = require("action_tutorials_interfaces.action").Fibonacci
local action_msg = require("action_msgs.msg")
local action_srv = require("action_msgs.srv")

-- Init ROS environment
rclbind.context_init(arg)

-- Make objects
local node = rclbind.new_node('raw_action_client')
local act_cli = rclbind.new_action_client(
  node, Fibonacci, 'fibonacci', nil, 
  action_srv.CancelGoal, action_msg.GoalStatusArray)

-- Put to mark callback
local function empty() end

-- Process feedback message
local function msg_callback (msg)
  local v = msg.feedback.partial_sequence
  print(table.concat(v, ' '))
end

-- Send goal to action service
local function async_send (goal)
  assert(rclbind.is_instance(goal, act_cli:get_interface 'Goal'), 'Not a goal message')
  local uuid = rclbind.uuid.new()  -- make random
  local send_goal = act_cli:get_interface('SendGoal')
  local req = send_goal.Request { goal = goal }
  assert(req.goal_id.uuid(uuid), 'Unable to set UUID')
  act_cli:set_feedback_method(uuid, msg_callback)
  local seq = act_cli:send_goal_request(req, empty)
  return seq, uuid
end

-- Send goal cancel request
local function cancel_goal (uuid)
  local req = action_srv.CancelGoal.Request()
  req.goal_info.goal_id.uuid(uuid)
  return act_cli:send_cancel_request(req, empty)
end

-- Send result request
local function get_result (uuid)
  local req = Fibonacci.GetResult.Request()
  assert(req.goal_id.uuid(uuid))
  return act_cli:send_result_request(req, empty)
end

-- Wait for server
while not act_cli:is_action_server_available() do
  rclbind.simp_log(rclbind.LogLevel.INFO, node:get_name(), "waiting for service")
  rclbind.sleep_thread(1.0)
end

-- Send goal
local msg = Fibonacci.Goal {order = 10}
local seq, uuid = async_send(msg)

-- Prepare WaitSet
local sub_no, guard_no, timer_no, cli_no, srv_no = act_cli:get_num_entities()
local wait_set = rclbind.new_wait_set(sub_no, guard_no, timer_no, cli_no, srv_no, 0)

-- Processsing
while rclbind.context_ok() do
  -- prepare
  wait_set:clear()
  act_cli:add_to_waitset(wait_set)

  -- wait for service or timer (wait_set:wait(-1))
  if not pcall(wait_set.wait, wait_set, -1) then
    break
  end

  local is_feedback, is_status, is_goal, is_cancel, is_result = act_cli:is_ready(wait_set)

  local data = {}
  if is_feedback then data['feedback'] = act_cli:take_feedback() end
  if is_status   then data['status'] = act_cli:take_status() end
  if is_goal     then data['goal'] = act_cli:take_goal_response() end
  if is_cancel   then data['cancel'] = act_cli:take_cancel_response() end
  if is_result   then data['result'] = act_cli:take_result_response() end

  -- processing

  if data['goal'] then
    local resp, fn, seq = table.unpack(data['goal'])
    -- fn(resp)
    if resp.accepted then
      get_result(uuid)
    else
      rclbind.simp_log(rclbind.LogLevel.WARN, node:get_name(), "not accepted")
      break
    end
  end
  
  if data['cancel'] then
    -- local resp, fn, seq = table.unpack(data['cancel'])
    -- process
    break 
  end

  if data['result'] then
    local resp, fn, seq = table.unpack(data['result'])
    rclbind.simp_log(rclbind.LogLevel.INFO, node:get_name(), 
     string.format('Result: %s', table.concat(resp.result.sequence, ' ')))
    break
  end

  if data['feedback'] then
    local msg, fn = table.unpack(data['feedback'])
    fn(msg)
  end

  if data['status'] then
    local msg = data['status']
    -- process status list
  end
end

-- Shutdown ROS context
rclbind.context_shutdown()
