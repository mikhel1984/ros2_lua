local rut = require 'rcllua_unit.testing'

local rclbind
function rut:init_context()
  rclbind = require('rcllua.rclbind')
  rclbind.context_init(arg)
end

function rut:logs()
  local Log = rclbind.LogLevel
  assert(Log.UNSET and Log.DEBUG and Log.INFO and
         Log.WARN and Log.ERROR and Log.FATAL)
  rclbind.write_log(Log.INFO, "some_node", "Hello World", 
                    "function name", "file_name", 42)
  rclbind.simp_log(Log.WARN, "some_node", "Message")
end

function rut:shutdown()
  rclbind.context_shutdown()
end

rut:run()

