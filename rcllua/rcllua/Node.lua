
local rclbind = require("rclbind")

Node = {}

function Node._init (name, ns)
  return rclbind.node_init(name, ns)
end

return Node
