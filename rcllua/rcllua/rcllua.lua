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

--- Library
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

return rcllua
