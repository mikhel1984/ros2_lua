local rut = require 'rcllua_unit.testing'

local rclbind, clock

function rut:init()
  rclbind = require('rcllua.rclbind')
  clock = rclbind.new_clock()
end

function rut:methods()
  assert(rut:eql(clock:clock_type(), rclbind.ClockType.SYSTEM_TIME))
  local t = clock:now()
  assert(t.sec > 0, "wrong time")
  assert(rut:eql(t.clock_type, clock:clock_type()))
end

rut:run()
