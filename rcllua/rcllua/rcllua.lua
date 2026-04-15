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

assert(_VERSION >= 'Lua 5.3', 'Lua 5.3+ expected')

local rclbind = require("rcllua.rclbind")
require("rcllua.Executor")


--- Library (global)
rcllua = {}

--- Default executor
local _executor = nil

--- Get reference to default executor, create if need
--  @return Executor object
local function get_global_executor()
  if not _executor then
    _executor = Executor()
  end
  return _executor
end

--- Pretty print for message.
--  @param msg ROS message object.
--  @param n Shift index.
--  @return string representation of the message.
local function _get_structure (msg, n)
  local shift = string.rep('  ', n)
  local acc, tp = {}, ~msg
  if type(tp) == 'table' then
    -- structure
    for _, nm in ipairs(tp) do
      local v = msg[nm]
      if type(v) == 'userdata' then
        -- message
        if #v == nil then
          -- structure
          acc[#acc+1] = string.format('%s%s:', shift, nm)
          acc[#acc+1] = _get_structure(v, n+1)
        else
          -- array
          acc[#acc+1] = string.format(
            '%s%s%s:', shift, nm, (~v == 'static') and string.format('[%d]', #v) or '')
          acc[#acc+1] = _get_structure(v, n)
        end
      else
        -- standard type
        acc[#acc+1] = string.format('%s%s: %s', shift, nm, tostring(v))
      end
    end
  else
    -- array
    for i = 1, #msg do
      local v = msg[i]
      if type(v) == 'userdata' then
        -- message
        local s = _get_structure(v, n+1)  -- structure, cannot be array
        local ln = shift .. '- '
        acc[#acc+1] = ln .. string.sub(s, #ln+1)
      else
        -- standard type
        acc[#acc+1] = string.format('%s- %s', shift, tostring(v))
      end
    end
  end
  return table.concat(acc, '\n')
end

--- Initialize ROS environment.
--  @param tbl Initialization arguments, CLI arguments by default.
function rcllua.init (self, tbl)
  rclbind.context_init(tbl or arg)
end

--- Get context status.
rcllua.ok = rclbind.context_ok

--- Shutdown context.
rcllua.shutdown = rclbind.context_shutdown

--- Run execution loop.
--  @param node Node object.
--  @param executor Executor object (optional)
function rcllua.spin (self, node, executor)
  local exec = executor or get_global_executor()
  exec:add_node(node)
  exec:spin()
  exec:remove_node(node)
end

--- Run spinning until Future task is complete or time is out.
--  @param node Node object.
--  @param future Future object.
--  @param executor Executor object (optional).
--  @param timeout_sec Wait time (optional).
function rcllua.spin_until_future_complete (self, node, future, executor, timeout_sec)
  local exec = executor or get_global_executor()
  local is_add = exec:add_node(node)
  exec:spin_until_future_complete(future, timeout_sec)
  if is_add then exec:remove_node(node) end
end

--- Stop execution for some time.
--  @param time Sleep time (float).
function rcllua.sleep_sec (self, time)
  rclbind.sleep_thread(time)
end

--- Get string representation for a ROS message.
--  @param msg Message object.
--  @return string representation.
function rcllua.tostring (self, msg)
  return _get_structure(msg, 0)
end

return rcllua
