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

local param_lib = require 'rcllua.Parameter'
local param_msg = require 'rcl_interfaces.msg'

local PARAM_REL_TOL = 1E-6
local Type = param_lib.parameter

--- Collect methods with parameters.
local node_param = {}

--- Get a parameter type by name.
--  @param name Fully-qualified name of the parameter.
--  @return found type.
function node_param.get_parameter_type (self, name)
  if self._parameter__list[name] then
    return self._parameter__list[name]:type()
  elseif self._allow_undeclared_parameters then
    return Type.NOT_SET
  end
  error(name..' not declared')
end

--- Get a list of parameter types.
--  @param names Fully-qualified names of the parameters to get.
--  @return list of types.
function node_param.get_parameter_types (self, names)
  local res = {}
  for i = 1, #names do
    assert(type(names[i]) == 'string', 'not a string')
    res[i] = node_param.get_parameter_type(self, names[i])
  end
  return res
end

--- Get a parameter by name.
--  @param name Fully-qualified name of the parameter.
--  @return parameter object.
function node_param.get_parameter (self, name)
  local p = self._parameter__list[name]
  if p then
    if p:type() ~= Type.NOT_SET or self._descriptor__list[name].dynamic_typing then
      return p
    end
    error(name..' not declared')
  elseif self._allow_undeclared_parameters then
    return param_lib.parameter.new_parameter(name, Type.NOT_SET)
  end
  error(name..' not declared')
end

--- Get a list of parameters.
--  @param names Fully-qualified names of the parameters to get.
--  @return list of parameters.
function node_param.get_parameters (self, names)
  local res = {}
  for i = 1, #names do
    assert(type(names[i]) == 'string', 'not a string')
    res[i] = node_param.get_parameter(self, names[i])
  end
  return res
end

--- Get a parameter of the alternative value.
--  @param name Fully-qualified name of the parameter.
--  @param alternate (optional) Alternative parameter to get if it had not been declared before.
--  @return parameter object.
function node_param.get_parameter_or (self, name, alternate)
  local p = self._parameter__list[name]
  if p and p:type() ~= Type.NOT_SET then
    return p
  end
  return alternate or param_lib.parameter.new_parameter(name, Type.NOT_SET)
end

--- Get parameters with the given previx in names.
--  @param prefix Prefix string.
--  @return dictionary {truncated_name = parameter}.
function node_param.get_parameters_by_prefix (self, prefix)
  if prefix ~= '' then
    prefix = prefix .. param_lib.parameter.PARAMETER_SEPARATOR_STRING
  end
  local templ = string.format('^%s(.*)$', prefix)
  local res = {}
  for name, p in ipairs(self._parameter__list) do
    local rest = string.match(name, templ)
    if rest then res[rest] = p end
  end
  return res
end

--- Get the parameter descriptor of a given parameter.
--  @param name Fully-qualified name of the parameter.
--  @return corresponding ParameterDescriptor object.
function node_param.describe_parameter (self, name)
  if self._descriptor__list[name] then
    return self._descriptor__list[name]
  elseif self._allow_undeclared_parameters then
    return param_msg.ParameterDescriptor()
  end
  error(name..' not declared')
end

--- Get the parameter descriptors of a given list of parameters.
--  @param names Fully-qualified names of the parameters to get.
--  @return list of ParameterDescriptor objects.
function node_param.describe_parameters (self, names)
  local res = {}
  for i = 1, #names do
    assert(type(names[i]) == 'string', 'not a string')
    res[i] = node_param.describe_parameter(names[i])
  end
  return res
end

--- Check restrictions for for integer value.
--  @param param Parameter object.
--  @param int_range IntegerRange object.
--  @return SetParametersResult object.
function node_param._apply_integer_range (self, param, int_range)
  local min_value = math.min(int_range.from_value, int_range.to_value)
  local max_value = math.max(int_range.from_value, int_range.to_value)

  local v = param:value()
  -- check range
  if v < min_value or v > max_value then
    return param_msg.SetParametersResult {
      successful=false,
      reason='parameter out of range'
    }
  end
  -- check step
  if int_range.step ~= 0 and (v - min_value) % int_range.step ~= 0 then
    return param_msg.SetParametersResult {
      successful=false,
      reason='not a valid step'
    }
  end

  return param_msg.SetParametersResult {successful=true}
end

--- Check restrictions for floating point value.
--  @param param Parameter object.
--  @param float_range FloatingPointRange object.
--  @return SetParametersResult object.
function node_param._apply_floating_point_range (self, param, float_range)
  local min_value = math.min(float_range.from_value, float_range.to_value)
  local max_value = math.max(float_range.from_value, float_range.to_value)

  local v = param:value()
  -- check range
  if v < min_value-PARAM_REL_TOL or v > max_value+PARAM_REL_TOL then
    return param_msg.SetParametersResult {
      successful=false,
      reason='parameter out of range'
    }
  end
  -- check step
  if float_range.step ~= 0 then
    local dist_int_steps = (v - min_value) / float_range.step
    local tmp = math.floor(dist_int_steps)
    dist_int_steps =
      math.abs(dist_int_steps - tmp) >= 0.5 and math.ceil(dist_int_steps) or tmp
    if math.abs(min_value + dist_int_steps*float_range.step - v) > PARAM_REL_TOL then
      return param_msg.SetParametersResult {
        successful=false,
        reason='not a valid step'
      }
    end
  end

  return param_msg.SetParametersResult {successful=true}
end

--- Apply descriptor to the parameter and return a result without saving the parameter.
--  @param param Parameter for checking.
--  @param descriptor (optional) Descriptor to apply.
--  @param check_read_only (optional, true) Apply read-only check.
function node_param._apply_descriptor (self, param, descriptor, check_read_only)
  -- check arguments
  if check_read_only == nil then
    check_read_only = true
  end
  local name = param:name()
  descriptor = descriptor or node_param.describe_parameter(self, name)

  if check_read_only and descriptor.read_only then
    return param_msg.SetParametersResult {
      successful=false,
      reason='trying to set read-only parameter'
    }
  end

  local tp = param:type()
  if descriptor.dynamic_typing then
    descriptor.type = tp
  elseif self._parameter__list[name] and tp == Type.NOT_SET then
    -- don't allow undeclare parameter
    return param_msg.SetParametersResult {
      successful=false,
      reason='static parameter cannot be undeclared'
    }
  elseif tp ~= Type.NOT_SET and tp ~= descriptor.type then
    return param_msg.SetParametersResult {
      successful=false,
      reason='wrong parameter type'
    }
  end

  if tp == Type.INTEGER and #descriptor.integer_range > 0 then
    return node_param._apply_integer_range(param, descriptor.integer_range[1])
  end
  if tp == Type.DOUBLE and #descriptor.floating_point_range > 0 then
    return node_param._apply_floating_point_range(param, descriptor.floating_point_range[1])
  end

  return param_msg.SetParametersResult {successful=true}
end

--- Apply descriptors and return aggregated result without saving.
--  @param param Parameters to be checked.
--  @param descriptors Descriptors to apply.
--  @param check_read_only True for read-only checks.
--  @return SetParametersResult object.
function node_param._apply_descriptors (self, params, descriptors, check_read_only)
  local res = {}
  for i, p in ipairs(params) do
    local d = descriptors[p:name()]
    if d then
      local res = node_param._apply_descriptor(self, p, d, check_read_only)
      if not res.successful then
        return res
      end
    end
  end
  return param_msg.SetParametersResult {successful=true}
end

--- Set all the given parameters.
--  @param params List of parameters to set.
--  @param descriptors (=nil) New descriptors to apply.
--  @param allow_not_set_type (=false) False if NOT_SET should be undeclared.
--  @return SetParametersResult object.
function node_param._set_parameters_atomically (self, params, descriptors, allow_not_set_type)
  -- apply new or old descriptors
  local res = node_param._apply_descriptors(
    self, params, descriptors or self._descriptor__list, not descriptors)
  if not res.successful then
    return res
  end  -- TODO add callbacks

  local param_event = param_msg.ParameterEvent()
  local ns = self:get_namespace()
  if ns == '/' then
    param_event.node = ns..self:get_name()
  else
    param_event.node = ns..'/'..self:get_name()
  end

  local del, new, change = {}, {}, {}  -- collect changes
  for _, p in ipairs(params) do
    local nm = p:name()
    if not allow_not_set_type and p:type() == Type.NOT_SET then
      -- undeclare parameters without type and value
      del[#del+1] = p:to_parameter_msg()
      self._parameter__list[nm] = nil
      self._descriptor__list[nm] = nil
    else
      -- update descriptor
      if descriptors then
        self._descriptor__list[nm] = descriptors[nm]
      elseif not self._descriptor__list[nm] then
        descriptor = param_msg.ParameterDescriptor()
        descriptor.dynamic_typing = true
        self._descriptor__list[nm] = descriptor
      end
    end

    if Type.NOT_SET == node_param.get_parameter_or(self, nm):type() then
      new[#new+1] = p:to_parameter_msg()
    else
      change[#change+1] = p:to_parameter_msg()
    end
    -- descriptor have been applied
    self._parameter__list[nm] = p
  end
  -- update message lists
  param_event.new_parameters(new)
  param_event.deleted_parameters(del)
  param_event.changed_parameters(change)

  local now = self:get_clock():now()
  param_event.stamp {
    sec = now.sec,
    nsec = now.nsec
  }
  --self._parameter_event__publisher.publish(param_event)

  return param_msg.SetParametersResult {successful=true}
end

--- Check if parameter list is correct. Raise error otherwise.
--  @param params List of parameters.
function node_param._check_undeclared_parameters (self, params)
  local undeclared = {}
  for _, p in ipairs(params) do
    assert(getmetatable(p) == param_lib.parameter, 'must be Parameter object')
    local nm = p:name()
    -- collect undeclared
    if not self._parameter__list[nm] then undeclared[#undeclared+1] = nm end
  end
  if not self._allow_undeclared_parameters and #undeclared > 0 then
    error('undeclared parameters: '..table.concat(undeclared, ', '))
  end
end

--- Check parameters and set.
--  @param params List of parameters to set.
--  @return SetParametersResult object.
function node_param.set_parameters_atomically (self, params)
  node_param._check_undeclared_parameters(self, params)
  return node_param._set_parameters_atomically(self, params)
end

--- Set parameters and get result for the set action.
--  @param params List of parameters.
--  @param descriptors (=nil) List of descriptors to set.
--  @param raise_on_failure (=false) Allow raise error.
--  @param allow_undeclared_parameters (=false) Check for undeclared if false.
--  @return result for each set action as list.
function node_param._set_parameters (
  self, params, descriptors, raise_on_failure, allow_undeclared)

  if not allow_undeclared then
    node_param._check_undeclared_parameters(self, params)
  end

  local results = {}
  for i, p in ipairs(params) do
    assert(not descriptors or descriptors[p:name()])
    -- set
    local res = self._set_parameters_atomically(self, {p}, descriptors, allow_undeclared)
    if raise_on_failure and not res.successful then
      error(res.reason)
    end
    results[i] = res
  end
  return results
end

--- Declare a list of parameters.
--  @param namespace Namespace for parameters.
--  @param params List of tuples {name, type|value, ParameterDescriptor}
--  @param ignore_override (=false) True if overrides should not be taken into account.
--  @return parameter list.
function node_param._declare_parameters (self, namespace, params, ignore_override)
  local param_list, descriptors = {}, {}

  -- get parameters and descriptors
  for i, tuple in ipairs(params) do
    -- checking
    assert(0 < #tuple and #tuple <= 4, "invalid parameter tuple length")
    local name, value, tp, descriptor = tuple[1], tuple[2], tuple[3], tuple[4]
    descriptor = descriptor or param_msg.ParameterDescriptor()
    assert(type(name) == 'string', 'first element is not a string')
    assert(getmetatable(descriptor).__name == param_msg.ParameterDescriptor._metatable,
           'not a ParameterDescriptor')

    if value == nil and tp == nil then
      descriptor.dynamic_typing = true
    end

    if tp ~= nil then
      -- type is known
      assert(tp ~= Type.NOT_SET, 'cannot declare as statically typed')
      assert(not descriptor.dynamic_typing, 'parameter type is provided')
      descriptor.type = tp
    elseif value ~= nil then
      -- only value is known
      if not descriptor.dynamic_typing then
        local mt = getmetatable(value)
        if mt and mt.__name == param_msg.ParameterValue._metatable then
          assert(value.type ~= Type.NOT_SET, 'cannot declare as statically typed')
          descriptor.type = value.type
        else
          descriptor.type = Type.from_parameter_value(value)
        end
      end
    end

    -- update value
    if not ignore_override and self._parameter__overrides[name] then
      value = self._parameter__overrides[name]:value()
    end
    if namespace ~= '' then
      name = namespace ..
             param_lib.parameter.PARAMETER_SEPARATOR_STRING ..
             name  -- TODO validate?
    end

    param_list[#param_list+1] = param_lib.parameter.new_parameter(name, nil, value)
    descriptors[name] = descriptor
  end

  -- notify about already declared parameters
  local declared = {}
  for _, p in ipairs(param_list) do
    if self._parameter__list[p:name()] then
      declared[#declared+1] = p:name()
    end
  end
  if #declared > 0 then
    error('already declared: '..table.concat(declared, ', '))
  end

  -- update
  node_param._set_parameters(self, param_list, descriptors, true, true)

  return param_list
end

--- Declare and initialize parameter.
--  @param name Fully-qualified name of the parameter.
--  @param value (=nil) Value of the parameter to declare.
--  @param descriptor (=nil) Descriptor of the parameter to declare.
--  @param ignore_override (=false) True if overrides should ot be taken into account.
--  @return parameter with assigned value.
function node_param._declare_parameter (self, name, value, descriptor, ignore_override)
  local args = {name}
  if value or descriptor then
    args = {name, value, nil, descriptor or param_msg.ParameterDescriptor()}
  end
  return node_param._declare_parameters(self, '', {args}, ignore_override)[1]
end

return node_param
