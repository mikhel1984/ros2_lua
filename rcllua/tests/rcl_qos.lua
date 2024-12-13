local rut = require 'rcllua_unit.testing'

local rclbind, qos

function rut:setting()
  rclbind = require('rcllua.rclbind')
  qos = rclbind.new_qos()

  qos.history = 1
  qos.depth = 2
  qos.reliability = 3
  qos.durability = 4
  qos.liveliness = 5
  qos.avoid_ros_namespace_conventions = true

  local dur = rclbind.new_duration(123, 456)
  qos.lifespan = dur
  qos.deadline = dur
  qos.liveliness_lease_duration = dur

  -- unknown field
  rut:catch(function() qos.foo = 42 end)
end

function rut:getting()
  assert(rut:eql(qos.history, 1))
  assert(rut:eql(qos.depth, 2))
  assert(rut:eql(qos.reliability, 3))
  assert(rut:eql(qos.durability, 4))
  assert(rut:eql(qos.liveliness, 5))
  assert(rut:eql(qos.avoid_ros_namespace_conventions, true))
  assert(rut:eql(qos.lifespan.sec, 123))
  assert(rut:eql(qos.deadline.sec, 123))
  assert(rut:eql(qos.liveliness_lease_duration.nsec, 456))
end

function rut:qos_profile()
  local q = rclbind.new_qos('qos_profile_default')
  assert(rut:eql(q.depth, 10))

  q = rclbind.new_qos('qos_profile_sensor_data')
  assert(rut:eql(q.depth, 5))
  q = rclbind.new_qos('qos_profile_system_default')
  assert(rut:eql(q.depth, 0))
  q = rclbind.new_qos('qos_profile_service_default')
  assert(rut:eql(q.depth, 10))
  q = rclbind.new_qos('qos_profile_parameter_events')
  assert(rut:eql(q.depth, 1000))
  q = rclbind.new_qos('qos_profile_unknown')
  assert(rut:eql(q.depth, 0))
  q = rclbind.new_qos('qos_profile_parameters')
  assert(rut:eql(q.depth, 1000))

  -- unknown profile
  rut:catch(function() rclbind.new_qos('other') end)
end

rut:run()
