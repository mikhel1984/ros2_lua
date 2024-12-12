local rut = require 'rcllua_unit.testing'

local rclbind

function rut:load()
  rclbind = require('rcllua.rclbind')
end

function rut:init_context()
  rclbind.context_init(arg)
  assert(rclbind.context_ok())
end

function rut:shrutdown()
  rclbind.context_shrutdown()
  assert(not rclbind.context_ok())
end

rut:run()

