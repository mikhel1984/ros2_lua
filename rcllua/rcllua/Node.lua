
local rclbind = require("rcllua.rclbind")

Node = {}

function Node._init (name, ns)
  return rclbind.node_init(name, ns)
end

return Node
