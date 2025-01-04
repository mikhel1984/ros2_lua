-- rclbind example
-- Make simple publisher

local rclbind = require("rcllua.rclbind")
local std_msgs = require("std_msgs.msg")

-- Init ROS environment
rclbind.context_init(arg)

-- Prepare main components
local node = rclbind.new_node('raw_publisher')
local qos = rclbind.new_qos()
qos.depth = 10
local pub = rclbind.new_publisher(node, std_msgs.String, '/topic', qos)
local wait_set = rclbind.new_wait_set(0, 0, 1, 0, 0, 0)  -- 3rd is timer number

local count = 0

-- Timer callback
local function timer_cb ()
  local msg = std_msgs.String()
  msg.data = string.format('Hello World: %d', count)
  pub:publish(msg)
  rclbind.simp_log(rclbind.LogLevel.INFO,
    node:get_name(),
    string.format('Publishing: %s', msg.data))
  count = count + 1
end

-- Make timer
local clock = rclbind.new_clock()
local timer = rclbind.new_timer(clock, 0.5, timer_cb)

-- Main loop
while rclbind.context_ok() do
  -- prepare
  timer:call()
  wait_set:clear()
  wait_set:add_timer(timer)

  -- wait for timer (wait_set:wait(-1))
  if not pcall(wait_set.wait, wait_set, -1) then
    break
  end

  -- collect ready timers
  local lst = wait_set:ready_timers()
  for i = 1, #lst do
    -- get list {callback, timer_reference}
    local fn, ref = table.unpack(lst[i])
    -- execute
    fn()
  end
end

-- Shutdown ROS context
rclbind.context_shutdown()
