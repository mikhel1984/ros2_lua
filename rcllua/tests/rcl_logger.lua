local ut = require 'rcllua_unit.testing'

local rclbind
function ut:init_context()
  rclbind = require('rcllua.rclbind')
  rclbind.context_init(arg)
end

function ut:logs()
  local Log = rclbind.LogLevel
  assert(Log.UNSET and Log.DEBUG and Log.INFO and
         Log.WARN and Log.ERROR and Log.FATAL)
  rclbind.write_log(
    Log.INFO, "some_node", "Hello World", 
    "function name", "file_name", 42)
  rclbind.simp_log(
    Log.WARN, "some_node", "Message")
end

function ut:shutdown()
  rclbind.context_shutdown()
end

ut:run()

