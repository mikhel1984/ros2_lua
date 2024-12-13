local rut = require 'rcllua_unit.testing'

local rclbind

function rut:time_value()
  rclbind = require('rcllua.rclbind')

  local tm = rclbind.new_time(123, 456)

  local float = tm:seconds()
  assert(rut:eqlf(math.floor(float), 123))

  assert(rut:eql(tm.sec, 123))
  assert(rut:eql(tm.nsec, 456))
  assert(rut:eql(tm.clock_type, 2))

  -- immutable
  rut:catch(function() tm.sec = 3 end)
end

function rut:duration_value()
  local dur = rclbind.new_duration(123, 456)

  local float = dur:seconds()
  assert(rut:eqlf(math.floor(float), 123))

  assert(rut:eql(dur.sec, 123))
  assert(rut:eql(dur.nsec, 456))

  -- immrutable
 rut:catch(function() tm.sec = 3 end)
end

function rut:compare_time()
  local t1 = rclbind.new_time(1, 2)
  local t2 = rclbind.new_time(3, 4)

  assert(t1 == t1, "eq failed")
  assert(t2 > t1, "gt failed")
  assert(t2 >= t2, "ge failed")

  -- different clock type
  local t3 = rclbind.new_time(1, 2, 3)
  rut:catch(function() return (t1 == t3) end)
end

function rut:compare_duration()
  local d1 = rclbind.new_duration(-1, -2)
  local d2 = rclbind.new_duration(1, 2)

  assert(d1 == d1, "eq failed")
  assert(d2 > d1, "gt failed")
  assert(d2 >= d2, "ge failed")
end

function rut:add_sub()
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

rut:run()

