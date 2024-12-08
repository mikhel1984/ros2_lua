
local rclbind = require("rcllua.rclbind")

rclbind.context_init(arg)
print(rclbind.context_ok())

local function timer_cb ()
  rclbind.simp_log(rclbind.LogLevel.INFO, "node", "Hello world")
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
