
local rclbind = require("rcllua.rclbind")

-- global
rcllua = {}

function rcllua.init (tbl) 
  rclbind.context_init(tbl or arg)
end

rcllua.ok = rclbind.context_ok

rcllua.shutdown = rclbind.context_shutdown

function rcllua.spin (node)
  print("spin")
end

return rcllua
