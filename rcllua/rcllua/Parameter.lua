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

-- https://docs.ros.org/en/rolling/Concepts/Basic/About-Parameters.html

local rclbind = require("rcllua.rclbind")

-- parameter definitions and interfaces
local param_msg = require('rcl_interfaces.msg')
local param_srv = require('rcl_interfaces.srv')
local ParameterType = param_msg.ParameterType

local TOPIC_SEPARATOR_STRING = '/'

--    PARAMETER

--- Parameter class.
Parameter = {
  PARAMETER_SEPARATOR_STRING = '.',

  -- parameter types
  NOT_SET = ParameterType.PARAMETER_NOT_SET,
  BOOL = ParameterType.PARAMETER_BOOL,
  INTEGER = ParameterType.PARAMETER_INTEGER,
  DOUBLE = ParameterType.PARAMETER_DOUBLE,
  STRING = ParameterType.PARAMETER_STRING,
  BYTE_ARRAY = ParameterType.PARAMETER_BYTE_ARRAY,
  BOOL_ARRAY = ParameterType.PARAMETER_BOOL_ARRAY,
  DOUBLE_ARRAY = ParameterType.PARAMETER_DOUBLE_ARRAY,
  INTEGER_ARRAY = ParameterType.PARAMETER_INTEGER_ARRAY,
  STRING_ARRAY = ParameterType.PARAMETER_STRING_ARRAY,
}
Parameter.__index = Parameter

--- Parameter constructor.
--  @param name Name string.
--  @param tp (=nil) Type index.
--  @param value (=nil) Parameter value.
function Parameter.new_parameter (name, tp, value)
  -- compare type and value
  local vt = value and Parameter.from_parameter_value(value)
  if tp and vt then
    assert(tp == vt or
      vt == Parameter.BYTE_ARRAY and (tp == Parameter.INTEGER_ARRAY or tp == Parameter.DOUBLE_ARRAY
        or tp == Parameter.STRING_ARRAY or tp == Parameter.BOOL_ARRAY) or
      tp == Parameter.DOUBLE and vt == Parameter.INTEGER,
      "different type name and value")
  end
  -- object
  local o = {
    _name = name,
    _type = tp or vt or Parameter.from_parameter_value(value),
    _value = value,
  }
  return setmetatable(o, Parameter)
end

--- Define type based on the given value.
--  @param value Parameter value.
--  @return type index.
function Parameter.from_parameter_value (value)
  if value == nil then
    return Parameter.NOT_SET
  elseif type(value) == 'boolean' then
    return Parameter.BOOL
  elseif type(value) == 'number' then
    return math.type(value) == 'integer' and Parameter.INTEGER or Parameter.DOUBLE
  elseif type(value) == 'string' then
    return Parameter.STRING
  elseif type(value) == 'table' then
    -- check types
    local tmp, n, t = {}, 0, nil
    for i = 1, #value do  -- collect
      tmp[ Parameter.from_parameter_value(value[i]) ] = true
    end
    for k in pairs(tmp) do  -- find table length
      t, n = k, n+1
    end
    if n == 0 then 
      return Parameter.BYTE_ARRAY
    end
    if n ~= 1 then error('Not a list of one allowed type') end
    -- get type
    if t == Parameter.BOOL then
      return Parameter.BOOL_ARRAY
    elseif t == Parameter.STRING then
      return Parameter.STRING_ARRAY
    elseif t == Parameter.DOUBLE then
      return Parameter.DOUBLE_ARRAY
    elseif t == Parameter.INTEGER then
      return Parameter.INTEGER_ARRAY
    end
  end
  error('Not allowed value type')
end

--- Make Parameter object from Parameter message.
--  @param msg Parameter message.
--  @return Parameter object.
function Parameter.from_parameter_message (msg)
  local t, value = msg.value.type, nil
  if t == Parameter.BOOL then
    value = msg.value.bool_value
  elseif t == Parameter.INTEGER then
    value = msg.value.integer_value
  elseif t == Parameter.DOUBLE then
    value = msg.value.double_value
  elseif t == Parameter.STRING then
    value = msg.value.string_value
  elseif t == Parameter.BYTE_ARRAY then
    value = msg.value.byte_array_value
  elseif t == Parameter.BOOL_ARRAY then
    value = msg.value.bool_array_value
  elseif t == Parameter.INTEGER_ARRAY then
    value = msg.value.integer_array_value
  elseif t == Parameter.DOUBLE_ARRAY then
    value = msg.value.double_array_value
  elseif t == Parameter.STRING_ARRAY then
    value = msg.value.string_array_value
  end
  return Parameter.new_parameter(msg.name, t, value)
end

--- Fill ParameterValue message.
--  @return ParameterValue object.
function Parameter.get_parameter_value (self)
  local t = self._type
  local msg = param_msg.ParameterValue()
  msg.type = t
  if t == Parameter.BOOL then
    msg.bool_value = self._value
  elseif t == Parameter.INTEGER then
    msg.integer_value = self._value
  elseif t == Parameter.DOUBLE then
    msg.double_value = self._value
  elseif t == Parameter.STRING then
    msg.string_value = self._value
  elseif t == Parameter.BYTE_ARRAY then
    msg.byte_array_value(self._value)
  elseif t == Parameter.BOOL_ARRAY then
    msg.bool_array_value(self._value)
  elseif t == Parameter.INTEGER_ARRAY then
    msg.integer_array_value(self._value)
  elseif t == Parameter.DOUBLE_ARRAY then
    msg.double_array_value(self._value)
  elseif t == Parameter.STRING_ARRAY then
    msg.string_array_value(self._value)
  end
  return msg
end

--- Fill Parameter message.
--  @return Parameter object.
function Parameter.to_parameter_msg (self)
  return param_msg.Parameter {
    name = self._name,
    value = self:get_parameter_value()
  }
end

--- Get parameter name.
--  @return name.
function Parameter.name (self)
  return self._name
end

--- Get parameter type.
--  @return type index.
function Parameter.type (self)
  return self._type
end

--- Get parameter value.
--  @return value.
function Parameter.value (self)
  return self._value
end

--- Print parameter object.
--  @return string representation.
function Parameter.__tostring (self)
  local s, v = nil, self._value
  if self._type == Parameter.BOOL_ARRAY then
    v = {}
    for i = 1, #self._value do 
      v[i] = self._value[i] and 'true' or 'false' 
    end
  end
  if type(v) == 'table' then
    s = string.format('{%s}', table.concat(v, ','))
  else
    s = tostring(v)
  end
  return string.format("%s = %s", self._name, s)
end 

-- Make object as Parameter(...).
setmetatable(Parameter, 
{
--- Call parameter constructor.
--  @param name Parameter name.
--  @param value (=nil) Initial value.
--  @param type_ (=nil) Parameter type.
--  @return Parameter object.
__call = function (self, name, value, type_)
  return Parameter.new_parameter(name, type_, value)  
end
})

--    PARAMETER SERVICE

--- ParameterService class.
local parameter_service = {}

--- Initialize parameter services.
--  @param node Node object.
function parameter_service.new_service (node)
  local prefix = node:get_name() .. TOPIC_SEPARATOR_STRING
  local qos_param = rclbind.new_qos('qos_profile_parameters')

  node:create_service(
    param_srv.DescribeParameters, 
    prefix .. 'describe_parameters',
    function (req, resp) parameter_service._describe_parameter_callback(node, req, resp) end, 
    qos_param)

  node:create_service(
    param_srv.GetParameters,
    prefix .. 'get_parameters',
    function (req, resp) parameter_service._get_parameters_callback(node, req, resp) end,
    qos_param)

  node:create_service(
    param_srv.GetParameterTypes,
    prefix .. 'get_parameter_types',
    function (req, resp) parameter_service._get_parameter_types_callback(node, req, resp) end,
    qos_param)

  node:create_service(
    param_srv.ListParameters,
    prefix .. 'list_parameters',
    function (req, resp) parameter_service._list_parameters_callback(node, req, resp) end,
    qos_param)

  node:create_service(
    param_srv.SetParameters,
    prefix .. 'set_parameters',
    function (req, resp) parameter_service._set_parameters_callback(node, req, resp) end,
    qos_param)

  node:create_service(
    param_srv.SetParametersAtomically,
    prefix .. 'set_parameters_atomically',
    function (req, resp) parameter_service._set_parameters_atomically_callback(node, req, resp) end,
    qos_param)
end

--- Method for parameter description.
--  @param node Node object.
--  @param req Request with list of names.
--  @param resp Response with list of descriptors.
function parameter_service._describe_parameter_callback (node, req, resp)
  local acc = {}
  for i = 1, #req.names do
    local ok, descriptor = pcall(node.describe_parameter, node, req.names[i])
    if not ok then return end
    acc[i] = descriptor
  end
  resp.descriptors(acc)
end

--- Method for getting parameter list.
--  @param node Node object.
--  @param req Request with list of names.
--  @param resp Response with list of parameters.
function parameter_service._get_parameters_callback (node, req, resp)
  local acc = {}
  for i = 1, #req.names do
    local ok, param = pcall(node.get_parameter, node, req.names[i])
    if not ok then return end
    acc[i] = param:get_parameter_value()
  end
  resp.values(acc)
end

--- Method for getting parameter types.
--  @param node Node object.
--  @param req Request with list of names.
--  @param resp Response with list of types.
function parameter_service._get_parameter_types_callback (node, req, resp)
  local acc = {}
  for i = 1, #req.names do
    local ok, tp = pcall(node.get_parameter_type, node, req.names[i])
    if not ok then return end
    acc[i] = tp
  end
  resp.types(acc)
end

--- Find names with specific number of parameter separators.
--  @param lst List with parameter names.
--  @param depth Maximal number of separators.
--  @return list with names where number of separators less then depth.
local function _sym_filtered (lst, depth)
  local res = {}
  for i = 1, #lst do
    local name = lst[i]
    -- calc repetition
    local a, b, n = 1, 0, 0
    while a do
      a, b = string.find(name, Parameter.PARAMETER_SEPARATOR_STRING, b+1, true)
      if a then n = n + 1 end
    end
    if n < depth then res[#res+1] = name end
  end
  return res
end

--- Find string prefix before the last separator.
--  @param name Parameter name.
--  @return prefix string.
local function _max_prefix (name)
  local a, b = 1, 0
  local pa, pb = a, b
  while a do
    pa, pb = a, b
    a, b = string.find(name, Parameter.PARAMETER_SEPARATOR_STRING, b+1, true)
  end
  return string.sub(name, 1, pa-1)
end

--- Get parameters with specific prefix.
--  @param node Node object.
--  @param req Request with the prefix list.
--  @param resp Response with names and prefixes.
function parameter_service._list_parameters_callback (node, req, resp)
  local acc, names_with_prefixes = {}, {}

  -- collect parameters
  for name, p in pairs(node._parameter__list) do
    if string.find(name, Parameter.PARAMETER_SEPARATOR_STRING, 1, true) then
      table.insert(names_with_prefixes, name)
    elseif #req.prefixes > 0 then
      -- select specific names
      for i = 1, #req.prefixes do
        local templ = '^' .. req.prefixes[i]
        if string.find(name, templ) then
          acc[#acc+1] = name
        end
      end
    else
      acc[#acc+1] = name
    end
  end
  if req.depth == 1 then
    resp.result.names(acc)
    return
  end

  -- process prefixes
  local pref_dict = {}
  if req.depth == param_srv.ListParameters.Request.DEPTH_RECURSIVE then
    names_with_prefixes = _sym_filtered(names_with_prefixes, req.depth)
  end
  for _, name in ipairs(names_with_prefixes) do
    if #req.prefixes > 0 then
      for i = 1, #req.prefixes do
        local templ = '^' .. req.prefixes[i] .. Parameter.PARAMETER_SEPARATOR_STRING
        if string.find(name, templ) then
          acc[#acc+1] = name
          pref_dict[req.prefixes[i]] = true
          pref_dict[_max_prefix(name)] = true
        end
      end
    else
      acc[#acc+1] = name
      pref_dict[_max_prefix(name)] = true
    end
  end
  local lst = {}
  for k in pairs(pref_dict) do lst[#lst+1] = k end

  resp.result.names(acc)
  resp.result.prefixes(lst)
end

--- Update parameters.
--  @param node Node object.
--  @param req Request with new parameters.
--  @param resp Response with result of operation.
function parameter_service._set_parameters_callback (node, req, resp)
  local acc = {}
  for i = 1, #req.parameters do
    local param = Parameter.from_parameter_message(req.parameters[i])
    local ok, res = pcall(node.set_parameters_atomically, node, {param})
    acc[i] = ok and res or param_msg.SetParametersResult {
      successful = false,
      reason = res,
    }
  end
  resp.results(acc)
end

--- Update parameters atomically.
--  @param node Node object.
--  @param req Request with new parameters.
--  @param resp Response with result of operation.
function parameter_service._set_parameters_atomically_callback (node, req, resp)
  local lst = {}
  for i = 1, #req.parameters do
    lst[i] = Parameter.from_parameter_message(req.parameters[i])
  end
  local ok, res = pcall(node.set_parameters_atomically, node, lst)
  resp.result = res
end

-- Interface.
return {
  parameter = Parameter,
  new_parameter_service = parameter_service.new_service
}
