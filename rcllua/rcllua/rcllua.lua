
local rclbind = require("rcllua.rclbind")
require("rcllua.Executor")

-- global
rcllua = {}

local _executor = nil

local function get_global_executor()
  if not _executor then
    _executor = Executor()
  end
  return _executor
end

function rcllua.init (self, tbl) 
  rclbind.context_init(tbl or arg)
end

rcllua.ok = rclbind.context_ok

rcllua.shutdown = rclbind.context_shutdown

function rcllua.spin (self, node, executor)
  local exec = executor or get_global_executor()
  exec:add_node(node)
  exec:spin()
  exec:remove_node(node)
end

function rcllua.spin_until_future_complete (self, node, future, executor, timeout_sec)
  local exec = executor or get_global_executor()
  local is_add = exec:add_node(node)
  exec:spin_until_future_complete(future, timeout_sec)
  if is_add then exec:remove_node(node) end
end

return rcllua
