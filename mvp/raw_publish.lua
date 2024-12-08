
local rclbind = require("rcllua.rclbind")
local some_msgs = require("some_msgs.msg")

rclbind.context_init(arg)

local node = rclbind.node_init('lua_node')
local pub = rclbind.publisher_init(node, some_msgs.Text, '/check_output')
local msg = some_msgs.Text {data="Hello world"}


local function timer_cb ()
  pub:publish(msg)
  rclbind.simp_log(rclbind.LogLevel.INFO, "node1", "send")
end

local clock = rclbind.new_clock()
local timer = clock:new_timer(0.5, timer_cb)

while rclbind.context_ok() do
  local wait_set = rclbind.new_wait_set(0, 0, 1, 0, 0, 0)
  wait_set:clear()
  timer:call()
  wait_set:add_timer(timer)
  -- call wait_set:wait(-1) 
  if not pcall(wait_set.wait, wait_set, -1) then break end
  local lst = wait_set:ready_timers()
  for _, fn in ipairs(lst) do fn() end
end

rclbind.context_shutdown()
