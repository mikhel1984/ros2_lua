
local rclbind = require("rcllua.rclbind")

rcllua = {}

function rcllua.init (args)
  rclbind.context_init(args)
end

function rcllua.spin (node)
  print("spin")
end

function rcllua.ok ()
  return rclbind.context_ok()
end

function rcllua.shutdown ()
  rclbind.context_shutdown()
end

return rcllua
