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
local client_lib = require("rcllua.client").Client

-- "Lazy" access
local node_params = nil  -- parameter methods
local builtin_msg = nil  -- builtin interfaces

--- List of predefined Node keywords.
local protected = {
  name=true, namespace=true, init=true, bind=true,
  allow_undeclared_parameters=true,
  parameter_overrides=true,
  start_parameter_services=true}

--- Logger class.
local Logger = {name='rcllua'}

--    NODE

-- Node class.
Node = {}           -- global

--- Allow 'multiple inheritance'
Node.__index = function (t, k)
  return Node[k] or node_params and node_params[k] or nil
end

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
--  @param qos QoS profile.
--  @param callback Callback function fn(message) --> nil.
--  @return subscription (userdata).
function Node.create_subscription (self, msg, topic, qos, callback)
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
  timer:call()
  return timer
end

--- Create object for logging.
--  @return logger.
function Node.get_logger (self)
  local o = {name=self._node__name}
  return setmetatable(o, Logger)
end

--- Get node name.
--  @return name string.
function Node.get_name (self)
  return self._node__object:get_name()
end

--- Get node namespace.
--  @return node namespace string.
function Node.get_namespace (self)
  return self._node__object:get_namespace()
end

--- Get clock object.
--  @return node clock.
function Node.get_clock (self)
  return self._clock__object
end

--- Update reference to Executor object.
--  @param executor New reference or nil.
function Node.set_executor (self, executor)
  if self._executor__weak.ref then
    self._executor__weak.ref:remove_node(self)
  end
  self._executor__weak.ref = executor
end

function Node.add_waitable (self, action)
  table.insert(self._action__list, action)
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
  local fn = self[name]
  return function (...)
    return fn(self, ...)
  end
end

--- Similar to 'bind' method, but it puth function to coroutine.
--  It allows to use 'wait' and suspend execution.
--  @param name Function name in node table.
--  @return function with coroutine inside.
function Node.wrap (self, name)
  return coroutine.wrap(Node.bind(self, name))
end

--- "Sleep" until the condition is fulfilled.
--  @param cond Condition, it can be time in seconds or funciton() -> bool.
function Node.wait (self, cond)
  if type(cond) == 'number' then
    -- sleep for some time
    assert(cond >= 0, 'Expected positive duration')
    local clock = self._clock__object
    local finish = clock:now() + rclbind.new_duration_sec(cond)
    cond = function ()
      return clock:now() > finish
    end
  end
  assert(type(cond) == 'function', 'Wrong condition method')
  local co, main = coroutine.running()
  assert(not main, "method must be created with 'wrap' to call 'wait'")
  -- yield execution
  self._resume__list[co] = cond
  coroutine.yield()
  self._resume__list[co] = nil
end

--- Get list of yielded threads.
--  @return table with coroutines.
function Node.get_waited_list (self)
  -- make copy
  local t = {}
  for k, v in pairs(self._resume__list) do t[k] = v end
  return t
end

--- Get time as builtin_interfaces.Time object.
--  Try to load interface first. Return get_clock():now() by default.
--  @param t (=nil) rcllua time object.
--  @return time representation in form of builtin_interfaces.Time object.
function Node.get_time_msg (self, t)
  builtin_msg = builtin_msg or require('builtin_interfaces.msg')
  t = t or self._clock__object:now()
  local msg = builtin_msg.Time()
  msg.sec = t.sec
  msg.nanosec = t.nanosec
  return msg
end

--- Load table with parameters methods.
function Node.load_parameter_methods (self)
  if not node_params then
    -- add parameter methods
    node_params = require('rcllua.node_parameters')
    node_params._add_event_publisher(self)
    if self._start_parameter_services ~= false then
      -- add parameter service
      local lib_param = require('rcllua.Parameter')
      lib_param.new_parameter_service(self)
    end
  end
end

--- Declare and initialize parameter.
--  @param name Fully-qualified name of the parameter.
--  @param value (=nil) Value of the parameter to declare.
--  @param descriptor (=nil) Descriptor of the parameter to declare.
--  @param ignore_override (=false) True if overrides should ot be taken into account.
--  @return parameter with assigned value.
function Node.declare_parameter (self, name, value, descriptor, ignore_override)
  Node.load_parameter_methods(self)
  return node_params._declare_parameter(self, name, value, descriptor, ignore_override)
end

--- Declare a list of parameters.
--  @param namespace Namespace for parameters.
--  @param params List of tuples {name, value, type, ParameterDescriptor}
--  @param ignore_override (=false) True if overrides should not be taken into account.
--  @return parameter list.
function Node.declare_parameters (self, namespace, params, ignore_override)
  Node.load_parameter_methods(self)
  return node_params._declare_parameters(self, namespace, params, ignore_override)
end

--- Set parameters.
--  @param params The list of parameters to set.
--  @return list of results for every set action.
function Node.set_parameters (self, params)
  Node.load_parameter_methods(self)
  return node_params._set_parameters(self, params)
end

--- Check if the parameter is defined.
--  @param name Parameter name.
--  @return true if the parameter is found.
function Node.has_parameter (self, name)
  return self._parameter__list[name] ~= nil
end

--- Node object constructor.
--  @param ... Additional parameters for passing to 'init' funciton.
--  @return initialized object.
function Node.__call (self, ...)
  -- make instance
  local o = {}
  -- create node object
  o._node__object = rclbind.new_node(self.name, self.namespace)
  -- add default clock
  o._clock__object = rclbind.new_clock()
  -- save name for quick access
  o._node__name = self.name
  -- save executor later
  o._executor__weak = setmetatable({ref=nil}, {__mode='v'})
  -- wait for resume
  o._resume__list = {}
  -- references
  o._timer__list = {}
  o._subscription__list = {}
  o._client__list = {}
  o._service__list = {}
  o._guard__list = {}
  o._event__list = {}
  o._action__list = {}
  -- for parameters
  o._parameter__list = {}
  o._descriptor__list = {}
  o._allow_undeclared_parameters = self.allow_undeclared_parameters
  o._start_parameter_services = self.start_parameter_services
  o._parameter__overrides = self.parameter_overrides or {}
  -- copy other elements
  for k, v in pairs(self) do
    if not protected[k] then o[k] = v end
  end
  -- add Node methods
  setmetatable(o, Node)
  -- call initialization
  if self.init then
    self.init(o, ...)
  end
  -- add parameter service
  if self.start_parameter_services then
    Node.load_parameter_methods(o)
  end
  return o
end

-- Allow to call Node table.
setmetatable(Node,
{
--- Node class constructor.
--  @param param Table with initialization parameters.
__call = function (self, param)
  assert(param and param.name, "'name' must be defined")
  -- save as init parameters
  return setmetatable(param, self)
end
})

--    LOGGER

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
