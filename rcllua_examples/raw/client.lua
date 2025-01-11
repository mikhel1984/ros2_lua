-- rclbind example
-- Make simple client

local rclbind = require("rcllua.rclbind")
local std_srvs = require("std_srvs.srv")

-- Init ROS environment
rclbind.context_init(arg)

-- Make objects
local node = rclbind.new_node('raw_client')
local cli = rclbind.new_client(node, std_srvs.Trigger, '/state_trigger')
local wait_set = rclbind.new_wait_set(0, 0, 0, 1, 0, 0)  -- 4th is number of clients

-- Client callback
local function client_cb (resp)
  rclbind.simp_log(rclbind.LogLevel.INFO, 
    node:get_name(), 
    string.format('Status: %s', resp.message))
end

-- Wait for service
while not cli:service_is_available() do
  rclbind.simp_log(rclbind.LogLevel.INFO, node:get_name(), "waiting for service")
  rclbind.sleep_thread(1.0)
end


-- Send request
if cli:service_is_available() then
  local req = std_srvs.Trigger.Request()
  cli:send_request(req, client_cb)

  -- Call once
  while rclbind.context_ok() do
    wait_set:clear()
    wait_set:add_client(cli)

    -- wait for response (wait_set:wait(-1))
    if not pcall(wait_set.wait, wait_set, -1) then 
      break 
    end

    -- collect ready clinets
    local lst = wait_set:ready_clients()
    for i = 1, #lst do
      local resp, fn = table.unpack(lst[i])
      fn(resp)
    end

    break  -- call once
  end
end

-- Shutdown ROS context
rclbind.context_shutdown()
