-- rcllua example
-- Parameter declaration.

require "rcllua.rcllua"
require "rcllua.Node"

-- for parameter types
require "rcllua.Parameter"
-- for descriptor
local param_msg = require 'rcl_interfaces.msg'


-- Define 'class'
local ParamNode = Node { name = "parameter_example" }

-- Constructor
function ParamNode.init (self)
  -- single parameter
  self:declare_parameter('p1', 0.5)    -- double
  self:declare_parameter('p2', 'foo')  -- string
  self:declare_parameter('p3', {'a', 'b', 'c'})   -- string array
  -- with descriptor
  local descriptor = param_msg.ParameterDescriptor {
    description = 'demo parameter',
    read_only = true,
  }
  self:declare_parameter('p4', 123, descriptor)
  -- list of parameters
  local list = {
    {'p5', nil, Parameter.INTEGER_ARRAY},             -- only type
    {'p6', {1.0, 2.0, 3.0}, Parameter.DOUBLE_ARRAY},  -- value + type
    {'p7', true, nil, descriptor}   -- value + descriptor
  }
  local namespace = ''
  self:declare_parameters(namespace, arg)

  -- get value
  local p = self:get_parameter('p2')
  self:get_logger():info(tostring(p))

  -- get descriptor
  local d = self:describe_parameter('p4')
  self:get_logger():info(d.description)

  -- get with alternative value
  p = self:get_parameter_or('p10', Parameter('uu', 42))
  self:get_logger():info(tostring(p))
end


-- Execute
rcllua:init()

rcllua:spin(ParamNode())

rcllua:shutdown()
