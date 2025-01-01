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

-- parameter definitions and interfaces
local param_msg = require 'rcl_interfaces.msg'

local ParameterType = param_msg.ParameterType

--- Parameter class.
Parameter = {
  PARAMETER_SEPARATOR_STRING = '.',

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

function Parameter.new_parameter (name, tp, value)
  local vt = value and Parameter.from_parameter_value(value)
  if tp and vt then
    assert(tp == vt or
      tp == Parameter.INTEGER_ARRAY and vt == Parameter.BYTE_ARRAY or
      tp == Parameter.DOUBLE and vt == Parameter.INTEGER,
      "different type name and value")
  end

  local o = {
    _name = name,
    _type = tp or vt or Parameter.from_parameter_value(value),
    _value = value,
  }
  return setmetatable(o, Parameter)
end


function Parameter.from_parameter_value (self, value)
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
    for i = 1, #value do
      tmp[ Parameter.from_parameter_value(self, value[i]) ] = true
    end
    for k in pairs(tmp) do
      t, n = k, n+1
    end
    if n ~= 1 then error('Not a list of one allowed type') end
    if t == Parameter.BOOL then
      return Parameter.BOOL_ARRAY
    elseif t == Parameter.STRING then
      return Parameter.STRING_ARRAY
    elseif t == Parameter.DOUBLE then
      return Parameter.DOUBLE_ARRAY
    elseif t == Parameter.INTEGER then
      -- check byte array
      for _, v in ipairs(value) do
        if v < 0 or v >= 256 then
          return Parameter.INTEGER_ARRAY
        end
      end
      return Parameter.BYTE_ARRAY
    end
  else
    error('Not allowed value type')
  end
end

function Parameter.get_parameter_value (self, value)
  local t = self._type
  local msg = param_msg.ParameterValue()
  msg.type = t
  if t == Parameter.BOOL then
    msg.bool_value = value
  elseif t == Parameter.INTEGER then
    msg.integer_value = value
  elseif t == Parameter.DOUBLE then
    msg.double_value = value
  elseif t == Parameter.STRING then
    msg.string_value = string
  elseif t == Parameter.BYTE_ARRAY then
    msg.byte_array_value(value)
  elseif t == Parameter.BOOL_ARRAY then
    msg.bool_array_value(value)
  elseif t == Parameter.INTEGER_ARRAY then
    msg.integer_array_value(value)
  elseif t == Parameter.DOUBLE_ARRAY then
    msg.double_array_value(value)
  elseif t == Parameter.STRING_ARRAY then
    msg.string_array_value(string)
  end
  return msg
end

function Parameter.name (self)
  return self._name
end

function Parameter.type (self)
  return self._type
end

function Parameter.value (self)
  return self._value
end


-- Module interface
return {
  new_parameter = new_parameter,
}
