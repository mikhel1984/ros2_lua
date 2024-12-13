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

-- Define and call unit tests in Lua.
--
-- Usage:
--
-- Load moule, define test functions as a methods for the loaded table.
-- Checking can be done with assertions. Call 'run' method to execute
-- all the unit tests.


--- Unit test library
local rut = {
_fn = {},
_nm = {},
}

--- Compare float numbers
--  @param x First number.
--  @param y Second number.
--  @param tol Required tolerance, default is 1E-3.
--  @return true when the found difference is less then the tolerance value.
rut.eqlf = function (self, x, y, tol)
  tol = tol or 1E-3
  if math.abs(x-y) > tol then
    return false, string.format('|%f - %f| > %f', x, y, tol)
  end
  return true
end

--- Compare two objects with '=='.
--  @param x First object.
--  @param y Second object.
--  @return true when equality is found.
rut.eql = function (self, x, y)
  if x ~= y then
    return false, tostring(x)..' is not '..tostring(y)
  end
  return true
end

--- Raise error when function does not do it.
--  @param fn function with error.
rut.catch = function (self, fn, ...)
  if pcall(fn, ...) then error('error expected') end
end

--- Execute the registered unit tests.
rut.run = function (self)
  local err = 0
  -- execute
  for i, fn in ipairs(self._fn) do
    local ok, res = pcall(fn)
    if ok then
      io.write('[  OK  ] ', self._nm[i], '\n')
    else
      io.stderr:write('[FAILED] ', self._nm[i], ', ', res, '\n')
      err = err + 1
    end
  end
  -- summary
  if err > 0 then
    -- io.stderr:write(tostring(err), ' tests failed\n')
    os.exit(1)
  else
    -- io.write('all tests passed\n')
  end
end

-- Make library
return setmetatable(rut,
{
  -- Save registered functions into the specific tables.
  __newindex = function (t, k, v)
    table.insert(t._nm, k)  -- names
    table.insert(t._fn, v)  -- functions
  end,
})

