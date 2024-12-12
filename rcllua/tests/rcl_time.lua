local ut = require 'rcllua_unit.testing'

local rclbind

local function empty(x) end

function ut:time_value()
  rclbind = require('rcllua.rclbind')

  local tm = rclbind.new_time(123, 456)

  local float = tm:seconds()
  assert(ut:eqlf(math.floor(float), 123))

  assert(ut:eql(tm.sec, 123))
  assert(ut:eql(tm.nsec, 456))
  assert(ut:eql(tm.clock_type, 2))

  -- immutable
  assert(not pcall(function() tm.sec = 3 end))
end

function ut:duration_value()
  local dur = rclbind.new_duration(123, 456)

  local float = dur:seconds()
  assert(ut:eqlf(math.floor(float), 123))

  assert(ut:eql(dur.sec, 123))
  assert(ut:eql(dur.nsec, 456))

  -- immutable
  assert(not pcall(function() tm.sec = 3 end))
end

function ut:compare_time()
  local t1 = rclbind.new_time(1, 2)
  local t2 = rclbind.new_time(3, 4)

  assert(t1 == t1, "eq failed")
  assert(t2 > t1, "gt failed")
  assert(t2 >= t2, "ge failed")

  -- different clock type
  local t3 = rclbind.new_time(1, 2, 3)
  assert(not pcall(function() empty(t1 == t3) end))
end

function ut:compare_duration()
  local d1 = rclbind.new_duration(-1, -2)
  local d2 = rclbind.new_duration(1, 2)

  assert(d1 == d1, "eq failed")
  assert(d2 > d1, "gt failed")
  assert(d2 >= d2, "ge failed")
end

function ut:add_sub()
  local t1 = rclbind.new_time(100)
  local d = rclbind.new_duration(1)

  local t2 = t1 + d
  assert(t2 > t1)

  local t3 = t1 + (-d)
  assert(t3 < t1)

  local d2 = t2 - t1
  assert(d == d2)

  local d3 = t1 - t2
  assert(d3 == -d)
end

ut:run()

