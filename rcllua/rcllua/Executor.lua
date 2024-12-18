
local rclbind = require("rcllua.rclbind")

local function _exec_ready_callbacks (executor, timeout_sec)
  local subscriptions = {}
  local timers = {}
  local services = {}
  local clients = {}
  local guards = {}
  local events = {}

  local is_new = (executor._wait_set == nil)

  for _, node in ipairs(executor._nodes) do
    for _, sub in ipairs(node._subscription__list) do
      subscriptions[#subscriptions+1] = sub
    end
    if executor._sub_no ~= #subscriptions then 
      executor._sub_no = #subscriptions
      is_new = true 
    end

    for _, timer in ipairs(node._timer__list) do
      timers[#timers+1] = timer
    end
    if executor._timer_no ~= #timers then
      executor._timer_no = #timers
      is_new = true
    end

    for _, cli in ipairs(node._client__list) do
      clients[#clients+1] = cli
    end
    if executor._cli_no ~= #clients then
      executor._cli_no = #clients
      is_new = true
    end

    for _, srv in ipairs(node._service__list) do
      services[#services+1] = srv
    end
    if executor._srv_no ~= #services then
      executor._srv_no = #services
      is_new = true
    end
  end

  if is_new then
    executor._wait_set = rclbind.new_wait_set(
      executor._sub_no,
      executor._guard_no,
      executor._timer_no,
      executor._cli_no,
      executor._srv_no,
      executor._ev_no)
  end
  local wait_set = executor._wait_set
  wait_set:clear()

  for i = 1, #subscriptions do
    wait_set:add_subscription(subscriptions[i]) 
  end
  for i = 1, #timers do
    wait_set:add_timer(timers[i])
  end
  for i = 1, #clients do
    wait_set:add_client(clients[i])
  end
  for i = 1, #services do
    wait_set:add_service(services[i])
  end

  if timeout_sec > 0 then
    -- to nanoseconds
    timeout_sec = math.floor(timeout_sec * 1E9)
  end

  wait_set:wait(timeout_sec)

  -- reuse
  subscriptions = wait_set:ready_subscriptions()
  timers = wait_set:ready_timers()
  clients = wait_set:ready_clients()
  services = wait_set:ready_services()

  -- execute
  for i = 1, #subscriptions do
    local msg, fn = table.unpack(subscriptions[i])
    fn(msg)
    coroutine.yield()
  end


  
  return true
end


local Executor = {}

function Executor.add_node (self, node)
  for i = 1, #self._nodes do
    if self._nodes[i] == node then
      return false
    end
  end
  table.insert(self._nodes, node)
  return true
end

function Executor.remove_node (self, node)
  for i = 1, #self._nodes do
    if self._nodes[i] == node then
      table.remove(self._nodes, i)
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

function Executor.spin_once (self, timeout_sec)
  local ok, res 
  repeat
    if not self._cb_iter then
      self._cb_iter = coroutine.create(_exec_ready_callbacks)
      ok, res = coroutine.resume(self._cb_iter, self, timeout_sec)
    else
      ok, res = coroutine.resume(self._cb_iter)
    end
    if not ok then 
      error(res)   -- resend error
    elseif res then
      -- _exec_ready_callbacks is finished, restart
      self._cb_iter = nil
    end
  until not res    -- exit when res = nil/false
end


function new_executor ()
  local o = {}
  o._nodes = {}
  o._is_shutdown = false
  -- counters
  o._sub_no = 0
  o._guard_no = 0
  o._timer_no = 0
  o._cli_no = 0
  o._srv_no = 0
  o._ev_no = 0
end

