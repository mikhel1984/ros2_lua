-- rclbind example
-- Make simple subscriber

local rclbind = require("rcllua.rclbind")
local std_msgs = require("std_msgs.msg")

-- Init ROS environment
rclbind.context_init(arg)

-- Prepare node
local node = rclbind.new_node('raw_subscription')

-- Subscription callback
local function subscription_cb (msg)
  rclbind.simp_log(
    rclbind.LogLevel.INFO,
    node:get_name(),
    string.format('I heard: %s', msg.data))
end

-- Make objects
local qos = rclbind.new_qos()
qos.depth = 10
local sub = rclbind.new_subscription(
  node, std_msgs.String, 'topic', subscription_cb, qos)
local wait_set = rclbind.new_wait_set(1, 0, 0, 0, 0, 0)  -- 1st is subscription number

-- Main loop
while rclbind.context_ok() do
  -- prepare
  wait_set:clear()
  wait_set:add_subscription(sub)

  -- wait for message (wait_set:wait(-1))
  if not pcall(wait_set.wait, wait_set, -1) then
    break
  end

  -- collect ready subscriptions
  local lst = wait_set:ready_subscriptions()
  for i = 1, #lst do
    -- get list {message, callback}
    local msg, fn = table.unpack(lst[i])
    fn(msg)
  end
end

-- Free context
rclbind.context_shutdown()
