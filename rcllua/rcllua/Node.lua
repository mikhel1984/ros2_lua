
local rclbind = require("rcllua.rclbind")


local Logger = {name='rcllua'}
Logger.__index = Logger

-- local
Node = {}
Node.__index = Node

function Node.create_publisher (self, ...)
  return rclbind.new_publisher(self._node__object, ...)
end

function Node.create_timer (self, period)
  local timer = rclbind.new_timer(self._clock__object, period)
  table.insert(self._timer__list, timer)
  return timer
end

function Node.get_logger (self)
  local o = {name=self._node__name}
  return setmetatable(o, Logger)
end

local protected = {name=true, namespace=true, init=true}

function Node.__call (self, ...)
  local param = self._init__list
  -- make instance
  local o = {}
  -- create node object
  o._node__object = rclbind.new_node(param.name, param.namespace)
  -- add default clock
  o._clock__object = rclbind.new_clock()
  -- save name for quick access
  o._node__name = param.name
  -- references
  o._timer__list = {}
  o._subscription__list = {}
  o._client__list = {}
  o._service__list = {}
  o._guard__list = {}
  o._event__list = {}
  -- call initialization
  if param.init then
    param.init(self, ...)
  end
  -- copy other elements
  for k, v in pairs(param) do
    if not protected[k] then o[k] = v end
  end
  setmetatable(o, Node)
  return o
end

local node_mt = {}

function node_mt.__call (self, param)
  assert(param and param.name, "'name' must be defined")
  -- prepare object
  local o = {_init__param=param}
  setmetatable(o, self)
  return o
end

setmetatable(Node, node_mt)

local LogLevel = rclbind.LogLevel
local sformat = string.format

function Logger.info (self, ...)
  rclbind.simp_log(LogLevel.INFO, self.name, sformat(...))
end

function Logger.warn (self, ...)
  rclbind.simp_log(LogLevel.WARN, self.name, sformat(...))
end

function Logger.error (self, ...)
  rclbind.simp_log(LogLevel.ERROR, self.name, sformat(...))
end

function Logger.fatal (self, ...)
  rclbind.simp_log(LogLevel.FATAL, self.name, sformat(...))
end

function Logger.debug (self, ...)
  rclbind.simp_log(LogLevel.DEBUG, self.name, sformat(...))
end

return Node
