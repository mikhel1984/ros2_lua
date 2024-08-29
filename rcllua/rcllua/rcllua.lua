
local rclbind = require("rclbind")
rcllua = {}

rcllua.init = function (args)
  print("init")
end

rcllua.spin = function (node)
  print("spin")
end

rcllua.shutdown = function (node)
  print("node")
end

return rcllua
