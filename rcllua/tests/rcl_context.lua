local ut = require 'rcllua_unit.testing'

local rclbind

function ut:load()
  rclbind = require('rcllua.rclbind')
end

function ut:init_context()
  rclbind.context_init(arg)
  assert(rclbind.context_ok())
end

function ut:shutdown()
  rclbind.context_shutdown()
  assert(not rclbind.context_ok())
end

ut:run()

