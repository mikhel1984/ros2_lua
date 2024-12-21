
local rclbind = require("rcllua.rclbind")

local function _sec_nsec (time_sec)
  return math.floor(time_sec), math.floor((time_sec % 1)*1E9)
end

local function _wait_ready_callbacks (executor, timeout_sec)
  local subscriptions = {}
  local timers = {}
  local services = {}
  local clients = {}
  local guards = {}
  local events = {}

  for _, node in ipairs(executor._nodes) do
    for _, sub in ipairs(node._subscription__list) do
      subscriptions[#subscriptions+1] = sub
    end
    if executor._sub_no ~= #subscriptions then 
      executor._sub_no = #subscriptions
      executor._wait_set = nil
    end

    for _, timer in ipairs(node._timer__list) do
      timers[#timers+1] = timer
    end
    if executor._timer_no ~= #timers then
      executor._timer_no = #timers
      executor._wait_set = nil
    end

    for _, cli in ipairs(node._client__list) do
      clients[#clients+1] = cli
    end
    if executor._cli_no ~= #clients then
      executor._cli_no = #clients
      executor._wait_set = nil
    end

    for _, srv in ipairs(node._service__list) do
      services[#services+1] = srv
    end
    if executor._srv_no ~= #services then
      executor._srv_no = #services
      executor._wait_set = nil
    end
  end

  executor._wait_set = executor._wait_set or 
    rclbind.new_wait_set(
      executor._sub_no,
      executor._guard_no,
      executor._timer_no,
      executor._cli_no,
      executor._srv_no,
      executor._ev_no)


  local wait_set = executor._wait_set
  wait_set:clear()

  for i = 1, #subscriptions do wait_set:add_subscription(subscriptions[i]) end

  for i = 1, #timers do
    timers[i]:call()
    wait_set:add_timer(timers[i])
  end

  for i = 1, #clients do wait_set:add_client(clients[i]:handle()) end

  for i = 1, #services do wait_set:add_service(services[i]) end

  if timeout_sec > 0 then
    -- to nanoseconds
    timeout_sec = math.floor(timeout_sec * 1E9)
  end

  wait_set:wait(timeout_sec)

  -- collect result
  subscriptions = wait_set:ready_subscriptions()
  timers = wait_set:ready_timers()
  clients = wait_set:ready_clients()
  services = wait_set:ready_services()

  -- execute
  for i = 1, #subscriptions do
    local msg, fn = table.unpack(subscriptions[i])
    coroutine.yield(function() fn(msg) end)
  end

  for i = 1, #timers do
    local fn, ref = table.unpack(timers[i])
    if rclbind.is_timer_ready(ref) then
      coroutine.yield(fn)
    end
  end

  for i = 1, #services do
    local t = services[i]
    local req, resp, fn = table.unpack(t)
    coroutine.yield(
      function()
        fn(req, resp)
        rclbind.service_send_response(t)
      end)
  end

  for i = 1, #clients do
    local resp, fn = table.unpack(clients[i])
    coroutine.yield(function() fn(resp) end)
  end
  
  return true
end


Executor = {}
Executor.__index = Executor

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

function Executor.get_nodes (self)
  local ns = {}
  for i = 1, #self._nodes do ns[i] = self._nodes[i] end
  return ns
end

function Executor.spin (self)
  while rclbind.context_ok() and not self._is_shutdown do
    Executor.spin_once(self)
  end
end

function Executor.spin_until_future_complete (self, future, timeout_sec)
  if not future._is_future then error('Future expected') end
  if not timeout_sec or timeout_sec < 0 then
    while rclbind.context_ok() and not self._is_shutdown and not future:done() do
      Executor.spin_once(self, timeout_sec)
    end
  else
    local dur = rclbind.new_duration(_sec_nsec(timeout_sec))
    local finish = self._clock:now() + dur
    while rclbind.context_ok() and not self._is_shutdown and timeout_sec > 0 
          and not future:done() 
    do
      Executor.spin_once(timeout_sec)
      timeout_sec = (finish - self._clock:now()):seconds()
    end
  end
end

function Executor.spin_once (self, timeout_sec)
  timeout_sec = timeout_sec or -1
  local ok, handle
  repeat
    if self._cb_iter then
      ok, handle = coroutine.resume(self._cb_iter)
    else
      self._cb_iter = coroutine.create(_wait_ready_callbacks)
      ok, handle = coroutine.resume(self._cb_iter, self, timeout_sec)
    end
    if not ok then 
      error(handle)   -- resend error
    elseif coroutine.status(self._cb_iter) == 'dead' then
      self._cb_iter = nil  -- finished
    end
  until self._cb_iter
  handle()
end

function Executor.shutdown (self, timeout_sec)
  self._is_shutdown = true
end

setmetatable(Executor, {
__call = function ()
  local o = {}
  o._nodes = {}
  o._clock = rclbind.new_clock(rclbind.ClockType.STEADY_TIME)
  o._is_shutdown = false
  -- counters
  o._sub_no = 0
  o._guard_no = 0
  o._timer_no = 0
  o._cli_no = 0
  o._srv_no = 0
  o._ev_no = 0
  setmetatable(o, Executor)
  return o
end })
