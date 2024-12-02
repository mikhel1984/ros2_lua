local ut = require 'rcllua_unit.testing'

local rclbind
function ut:time_value()
  rclbind = require('rcllua.rclbind')

  local tm = rclbind.new_time(123, 456)

  local sec, nsec = tm:get()
  assert(ut:eql(sec, 123))
  assert(ut:eql(nsec, 456))

  local float = tm:seconds()
  assert(ut:eqlf(math.floor(float), 123))
end

ut:run()

