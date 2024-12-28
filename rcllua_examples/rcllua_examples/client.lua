-- rcllua example
-- Make simple client

-- Load libraries
require "rcllua.rcllua"
require "rcllua.Node"

local std_srvs = require("std_srvs.srv")

-- Define 'class'
local MinimalClient = Node {
  name = "minimal_client",

  -- node constructor
  init = function (self)
    self.cli = self:create_client(std_srvs.Trigger, '/state_trigger')
    while not self.cli:wait_for_service(1.0) do
      self:get_logger():info('service not available, waiting...')
    end
  end,

  -- call service
  send_request = function (self)
    local req = std_srvs.Trigger.Request()
    return self.cli:call_async(req)  -- return 'Future'
  end,
}

-- Execute
rcllua:init()

local node = MinimalClient()
local future = node:send_request()

rcllua:spin_until_future_complete(node, future)
if future:done() then
  local resp = future:result()
  node:get_logger():info('Status: %s', resp.message)
end

rcllua:shutdown()
