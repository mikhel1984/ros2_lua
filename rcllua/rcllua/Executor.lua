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

--- Collect ready for execution tasks.
--  @param timeout_sec Wait time.
--  @return coroutine yield with function for execution.
local function _wait_for_ready_callbacks (executor, timeout_sec)
  local subscriptions, sub_cnt = {}, 0
  local timers, timer_cnt = {}, 0
  local services, srv_cnt = {}, 0
  local clients, cli_cnt = {}, 0
  local guards, guard_cnt = {}, 0
  local events, ev_cnt = {}, 0
  local actions = {}

  for _, node in ipairs(executor._nodes) do
    for _, sub in ipairs(node._subscription__list) do
      subscriptions[#subscriptions+1] = sub
    end
    sub_cnt = #node._subscription__list + sub_cnt

    for _, timer in ipairs(node._timer__list) do
      timers[#timers+1] = timer
    end
    timer_cnt = #node._timer__list + timer_cnt

    for _, cli in ipairs(node._client__list) do
      clients[#clients+1] = cli
    end
    cli_cnt = #node._client__list + cli_cnt

    for _, srv in ipairs(node._service__list) do
      services[#services+1] = srv
    end
    srv_cnt = #node._service__list + srv_cnt

    for _, act in ipairs(node._action__list) do
      actions[#actions+1] = act
      local sub_no, guard_no, timer_no, cli_no, srv_no = act:get_num_entities()
      sub_cnt   = sub_cnt + sub_no
      timer_cnt = timer_cnt + timer_no
      cli_cnt   = cli_cnt + cli_no
      srv_cnt   = srv_cnt + srv_no
      guard_cnt = guard_cnt + guard_no
    end
  end

  if executor._sub_no ~= sub_cnt then
    executor._sub_no = sub_cnt
    executor._wait_set = nil
  end
  if executor._timer_no ~= timer_cnt then
    executor._timer_no = timer_cnt
    executor._wait_set = nil
  end
  if executor._cli_no ~= cli_cnt then
    executor._cli_no = cli_cnt
    executor._wait_set = nil
  end
  if executor._srv_no ~= srv_cnt then
    executor._srv_no = srv_cnt
    executor._wait_set = nil
  end
  if executor._guard_no ~= guard_cnt then
    executor._guard_no = guard_cnt
    executor._wait_set = nil
  end

  executor._wait_set = executor._wait_set or
    rclbind.new_wait_set(
      sub_cnt,
      guard_cnt,
      timer_cnt,
      cli_cnt,
      srv_cnt,
      ev_cnt)

  local wait_set = executor._wait_set
  wait_set:clear()

  for i = 1, #subscriptions do wait_set:add_subscription(subscriptions[i]) end

  for i = 1, #timers do wait_set:add_timer(timers[i]) end

  for i = 1, #clients do wait_set:add_client(clients[i]:handle()) end

  for i = 1, #services do wait_set:add_service(services[i]) end

  for i = 1, #actions do actions[i]:add_to_waitset(wait_set) end

  if timeout_sec > 0 then
    -- to nanoseconds
    timeout_sec = math.floor(timeout_sec * 1E9)
  end

  wait_set:wait(timeout_sec)
  if not rclbind.context_ok() then return end

  -- collect result
  subscriptions = wait_set:ready_subscriptions()
  timers = wait_set:ready_timers()
  clients = wait_set:ready_clients()
  services = wait_set:ready_services()

    -- execute
  for _, act in ipairs(actions) do
    local data = act:take_data(wait_set)
    if data then
      act:execute(data)
    end
  end

  for i = 1, #subscriptions do
    local msg, fn = table.unpack(subscriptions[i])
    coroutine.yield(function() fn(msg) end)
  end

  for i = 1, #timers do
    local fn, ref = table.unpack(timers[i])
    if rclbind.is_timer_ready(ref) then
      coroutine.yield(fn)
      rclbind.timer_call(ref)
    end
  end

  for i = 1, #services do
    local t = services[i]
    local req, fn = table.unpack(t)
    coroutine.yield(
      function()
        local resp = fn(req)
        rclbind.service_send_response(t, resp)
      end)
  end

  for i = 1, #clients do
    local resp, fn = table.unpack(clients[i])
    if resp and fn then
      coroutine.yield(function() fn(resp) end)
    end
  end
end

--- Resume coroutines if conditions are fulfilled.
--  @param executor Executor object.
local function _resume_waiters (executor)
  for i = 1, #executor._nodes do
    for co, condition in pairs(executor._nodes[i]:get_waited_list()) do
      if rclbind.context_ok() and condition() then
        assert(coroutine.resume(co))
      end
    end
  end
end

--- Find timeout value.
--  @param executor Executor object.
--  @param spin_timeout Initial timeout.
--  @return minimal timeout among requests or -1.
local function _resume_time (executor, spin_timeout)
  local tmin = spin_timeout and spin_timeout >=0 and spin_timeout or math.huge
  for i = 1, #executor._nodes do
    -- find minimal time
    local ti = executor._nodes[i]:get_shortest_time()
    if ti < tmin then
      tmin = ti
    end
  end
  return (tmin < math.huge) and tmin or -1
end


--- Executor class.
Executor = {}
Executor.__index = Executor

--- Add Node object.
--  @param node Object to add.
--  @return true if the node is new.
function Executor.add_node (self, node)
  for i = 1, #self._nodes do
    if self._nodes[i] == node then
      return false
    end
  end
  table.insert(self._nodes, node)
  node:set_executor(self)
  return true
end

--- Remove node object.
--  @param node Object to remove.
--  @return true if node found.
function Executor.remove_node (self, node)
  for i = 1, #self._nodes do
    if self._nodes[i] == node then
      table.remove(self._nodes, i)
      node:set_executor(nil)  -- remove
      return true
    end
  end
  return false
end

--- Run data spin.
function Executor.spin (self)
  while rclbind.context_ok() do
    -- check context status after some time
    Executor.spin_once(self, 5.0)
  end
end

--- Spin data until time is out or task is completed.
--  @param future Future object.
--  @param timeout_sec Wait time (optional).
function Executor.spin_until_future_complete (self, future, timeout_sec)
  if not future._is_future then 
    error 'Future expected' 
  end
  if not timeout_sec or timeout_sec < 0 then
    while rclbind.context_ok() and not future:done() do
      Executor.spin_once(self, timeout_sec)
    end
  else
    local finish = self._clock:now() + rclbind.new_duration_sec(timeout_sec)
    while rclbind.context_ok() 
      and timeout_sec > 0
      and not future:done()
    do
      Executor.spin_once(timeout_sec)
      timeout_sec = (finish - self._clock:now()):seconds()
    end
  end
end

--- Spin until time is out or got new data.
--  @param timeout_sec Wait time (optional).
function Executor.spin_once (self, timeout_sec)
  timeout_sec = _resume_time(self, timeout_sec)
  -- wait for message or timeout
  local ok, handle
  if self._cb_iter then
    ok, handle = coroutine.resume(self._cb_iter)
  else
    self._cb_iter = coroutine.create(_wait_for_ready_callbacks)
    ok, handle = coroutine.resume(self._cb_iter, self, timeout_sec)
  end

  if not ok then
    error(handle)   -- resend error
  elseif coroutine.status(self._cb_iter) == 'dead' then
    self._cb_iter = nil  -- finished
  end
  -- execute
  if handle then handle() end
  _resume_waiters(self)
end

-- Allow to call Executor table.
setmetatable(Executor,
{
--- Create Executor object.
__call = function ()
  local o = {}
  o._nodes = {}
  o._clock = rclbind.new_clock(rclbind.ClockType.STEADY_TIME)
  -- counters
  o._sub_no = 0
  o._guard_no = 0
  o._timer_no = 0
  o._cli_no = 0
  o._srv_no = 0
  o._ev_no = 0
  setmetatable(o, Executor)
  return o
end
})

return Executor
