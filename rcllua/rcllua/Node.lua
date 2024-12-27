-- Copyright 2025 Stanislav Mikhel
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--     http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

local rclbind = require("rcllua.rclbind")
local client_lib = require("rcllua.Client")

--- List of predefined Node keywords.
local protected = {name=true, namespace=true, init=true, bind=true}

--- Logger class.
local Logger = {name='rcllua'}

-- Node class.
Node = {}           -- global
Node.__index = Node

--- Create publisher object.
--  @param msg Message type.
--  @param topic Topic name.
--  @param qos QoS profile (optional).
--  @return publisher (userdata).
function Node.create_publisher (self, msg, topic, qos)
  if type(qos) == 'number' then
    local q = rclbind.new_qos()
    q.depth = qos
    qos = q
  end
  return rclbind.new_publisher(self._node__object, msg, topic, qos)
end

--- Create subscription object.
--  @param msg Message type.
--  @param topic Topic name.
--  @param callback Callback function fn(message) --> nil.
--  @param qos QoS profile (optional).
--  @return subscription (userdata).
function Node.create_subscription (self, msg, topic, callback, qos)
  if type(qos) == 'number' then
    local q = rclbind.new_qos()
    q.depth = qos
    qos = q
  end
  local sub = rclbind.new_subscription(self._node__object, msg, topic, callback, qos)
  table.insert(self._subscription__list, sub)
  return sub
end

--- Create service object.
--  @param srv Service type.
--  @param name Service name.
--  @param func Service function fn(request, response) --> nil.
--  @param qos QoS profile (optional).
--  @return service (userdata).
function Node.create_service (self, srv, name, func, qos)
  local srv = rclbind.new_service(self._node__object, srv, name, func, qos)
  table.insert(self._service__list, srv)
  return srv
end

--- Create client object.
--  @param srv Service type.
--  @param name Service name.
--  @param qos QoS profile (optional).
--  @return client (table).
function Node.create_client (self, srv, name, qos)
  local cli = client_lib.new_client(self._node__object, srv, name, qos)
  table.insert(self._client__list, cli)
  return cli
end

--- Create timer object.
--  @param period Time in seconds.
--  @param callback Function to execute fn() --> nil.
--  @return timer (userdata).
function Node.create_timer (self, period, callback)
  local timer = rclbind.new_timer(self._clock__object, period, callback)
  table.insert(self._timer__list, timer)
  return timer
end

--- Create object for logging.
--  @return logger.
function Node.get_logger (self)
  local o = {name=self._node__name}
  return setmetatable(o, Logger)
end

--- Update reference to Executor object.
--  @param executor New reference or nil.
function Node.set_executor (self, executor)
  if self._executor__weak.ref then
    self._executor__weak.ref:remove_node(self)
  end
  self._executor__weak.ref = executor
end

--- Get current Executor object.
--  @return reference to executor.
function Node.executor (self)
  return self._executor__weak.ref
end

--- Wrapper for function binding.
--  Allows to call function fn(obj, arg1...argN) as _fn(arg1...argN).
--  @param name Function name in node table.
--  @return function for binding.
function Node.bind (self, name)
  return function (...)
    return self[name](self, ...)
  end
end

--- Node object constructor.
--  @param ... Additional parameters for passing to 'init' funciton.
--  @return initialized object.
function Node.__call (self, ...)
  local param = self._init__param
  -- make instance
  local o = {}
  -- create node object
  o._node__object = rclbind.new_node(param.name, param.namespace)
  -- add default clock
  o._clock__object = rclbind.new_clock()
  -- save name for quick access
  o._node__name = param.name
  -- save executor later
  o._executor__weak = setmetatable({ref=nil}, {__mode='v'})
  -- references
  o._timer__list = {}
  o._subscription__list = {}
  o._client__list = {}
  o._service__list = {}
  o._guard__list = {}
  o._event__list = {}
  -- copy other elements
  for k, v in pairs(param) do
    if not protected[k] then o[k] = v end
  end
  for k, v in pairs(self) do
    if v ~= param then o[k] = v end
  end
  -- add Node methods
  setmetatable(o, Node)
  -- call initialization
  if param.init then
    param.init(o, ...)
  end
  return o
end

-- Allow to call Node table.
setmetatable(Node, {
--- Node class constructor.
--  @param param Table with initialization parameters.
__call = function (self, param)
  assert(param and param.name, "'name' must be defined")
  -- save init parameters
  local o = {_init__param=param}
  return setmetatable(o, self)
end })

--- List of log levels
local LogLevel = rclbind.LogLevel
Logger.__index = Logger

--- Pring message with INFO level.
--  @param ... Format string and parameters.
function Logger.info (self, ...)
  rclbind.simp_log(LogLevel.INFO, self.name, string.format(...))
end

--- Pring message with WARN level.
--  @param ... Format string and parameters.
function Logger.warn (self, ...)
  rclbind.simp_log(LogLevel.WARN, self.name, string.format(...))
end

--- Pring message with ERROR level.
--  @param ... Format string and parameters.
function Logger.error (self, ...)
  rclbind.simp_log(LogLevel.ERROR, self.name, string.format(...))
end

--- Pring message with FATAL level.
--  @param ... Format string and parameters.
function Logger.fatal (self, ...)
  rclbind.simp_log(LogLevel.FATAL, self.name, string.format(...))
end

--- Pring message with DEBUG level.
--  @param ... Format string and parameters.
function Logger.debug (self, ...)
  rclbind.simp_log(LogLevel.DEBUG, self.name, string.format(...))
end

return Node
