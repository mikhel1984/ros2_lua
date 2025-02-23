-- rclbind example
-- Make simple service

local rclbind = require("rcllua.rclbind")
local std_srvs = require("std_srvs.srv")

local is_on = false

-- Service function
local function call_trigger (req)
  is_on = not is_on
  resp = std_srvs.Trigger.Response()
  resp.success = true
  resp.message = is_on and 'Node is on' or 'Node is off'
  return resp
end

-- Init ROS environment
rclbind.context_init(arg)

-- Make objects
local node = rclbind.new_node('raw_service')
local srv = rclbind.new_service(node, std_srvs.Trigger, '/state_trigger', call_trigger)
local wait_set = rclbind.new_wait_set(0, 0, 1, 0, 1, 0)  -- 5th is service number

local function timer_cb ()
  if is_on then
    rclbind.simp_log(rclbind.LogLevel.INFO, node:get_name(), "I'm working...")
  end
end

-- Make timer
local clock = rclbind.new_clock()
local timer = rclbind.new_timer(clock, 0.5, timer_cb)
timer:call()

-- Main loop
while rclbind.context_ok() do
  -- prepare
  wait_set:clear()
  wait_set:add_service(srv)
  wait_set:add_timer(timer)

  -- wait for service or timer (wait_set:wait(-1))
  if not pcall(wait_set.wait, wait_set, -1) then
    break
  end
  -- collect services
  local lst = wait_set:ready_services()
  for i = 1, #lst do
    -- get list {request, callback, ...}
    local t = lst[i]
    local req, fn = table.unpack(t)
    local resp = fn (req)  -- call function
    rclbind.service_send_response (t, resp)
  end

  -- collect timers
  lst = wait_set:ready_timers()
  for i = 1, #lst do
    local fn, ref = table.unpack(lst[i])
    fn()
    rclbind.timer_call(ref)
  end
end

-- Shutdown ROS context
rclbind.context_shutdown()

