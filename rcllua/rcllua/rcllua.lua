
local rclbind = require("rcllua.rclbind")
require("rcllua.Executor")

-- global
rcllua = {}

function rcllua.init (self, tbl) 
  rclbind.context_init(tbl or arg)
end

rcllua.ok = rclbind.context_ok

rcllua.shutdown = rclbind.context_shutdown

function rcllua.spin (self, node)
  local executor = Executor()
  executor:add_node(node)
  executor:spin()
  executor:remove_node(node)
end

return rcllua
