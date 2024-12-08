
local rclbind = require("rcllua.rclbind")
local some_msgs = require("some_msgs.msg")

rclbind.context_init(arg)

local function subscription_cb (msg)
  rclbind.simp_log(rclbind.LogLevel.INFO, "node2", msg.data)
end

local node = rclbind.node_init('lua_node')
local sub = rclbind.subscription_init(
  node, some_msgs.Text, '/check_output', subscription_cb)

while rclbind.context_ok() do
  local wait_set = rclbind.new_wait_set(1, 0, 0, 0, 0, 0)
  wait_set:clear()
  assert(wait_set:add_subscription(sub) == 0)
  -- call wait_set:wait(-1) 
  if not pcall(wait_set.wait, wait_set, -1) then break end
  local lst = wait_set:ready_subscriptions()
  for _, t in ipairs(lst) do
    local msg, fn = t[1], t[2]
    fn(msg)
  end
end

rclbind.context_shutdown()
