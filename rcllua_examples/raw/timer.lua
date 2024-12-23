-- rclbind example.
-- Call timer without node.

-- Load library
local rclbind = require("rcllua.rclbind")

-- Define timer callback
local function timer_cb ()
  -- write log
  rclbind.simp_log(rclbind.LogLevel.INFO, "rcllua", "Hello world")
end

-- Initialize context
rclbind.context_init(arg)

-- Create objects
local clock = rclbind.new_clock(rclbind.ClockType.STEADY_TIME)
local timer = rclbind.new_timer(clock, 0.5, timer_cb)
local wait_set = rclbind.new_wait_set(0, 0, 1, 0, 0, 0)

-- Main loop
while rclbind.context_ok() do
  -- prepare 
  wait_set:clear()
  wait_set:add_timer(timer)
  timer:call()

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

-- Free context
rclbind.context_shutdown()
