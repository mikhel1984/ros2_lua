local ut = require 'rcllua_unit.testing'

local rclbind

function ut:time_value()
  rclbind = require('rcllua.rclbind')

  local tm = rclbind.new_time(123, 456)

  local float = tm:seconds()
  assert(ut:eqlf(math.floor(float), 123))

  assert(ut:eql(tm.sec, 123))
  assert(ut:eql(tm.nsec, 456))

  -- immutable
  assert(not pcall(function () tm.sec = 3 end))
end

function ut:duration_value()
  local dur = rclbind.new_duration(123, 456)

  local float = dur:seconds()
  assert(ut:eqlf(math.floor(float), 123))

  assert(ut:eql(dur.sec, 123))
  assert(ut:eql(dur.nsec, 456))

  -- immutable
  assert(not pcall(function () tm.sec = 3 end))
end

ut:run()

