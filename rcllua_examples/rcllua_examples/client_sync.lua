-- rcllua example
-- Make simple client with syncronous call

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
    -- wait for result here
    local resp = self.cli:call(req, 2.0)
    self:get_logger():info('Status: %s', resp.message)
    -- exit
    rcllua:shutdown()
  end,
}

-- Execute
rcllua:init()

local node = MinimalClient()
-- use 'wrap' to make coroutine
node:wrap 'send_request' ()

rcllua:spin(node)
