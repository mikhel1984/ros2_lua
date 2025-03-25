local rut = require 'rcllua_unit.testing'

local rclbind, clock

function rut:init()
  rclbind = require('rcllua.rclbind')
  clock = rclbind.new_clock()

  -- clock types
  assert(rclbind.ClockType.UNINITIALIZED, "UNINITIALIZED not defined")
  assert(rclbind.ClockType.ROS_TIME, "ROS_TIME not defined")
  assert(rclbind.ClockType.SYSTEM_TIME, "SYSTEM_TIME not defined")
  assert(rclbind.ClockType.STEADY_TIME, "STEADY_TIME not defined")
end

function rut:get_time()
  assert(rut:eql(clock:clock_type(), rclbind.ClockType.SYSTEM_TIME))
  local t = clock:now()
  assert(t.sec > 0, "wrong time")
  assert(rut:eql(t.clock_type, clock:clock_type()))
end

function rut:methods()
  local clock2 = rclbind.new_clock(rclbind.ClockType.ROS_TIME)
  clock2:set_ros_time_override_is_enabled(true)
  assert(clock2:get_ros_override_is_enabled(), "override check failed")

  local t = rclbind.new_time(123, 456)
  clock2:set_ros_time_override(t)

  clock2:set_ros_time_override_is_enabled(false)
  assert(not clock2:get_ros_override_is_enabled(), "override check failed")
end

rut:run()
