-- rcllua example
-- Make simple service

-- Load libraries
require "rcllua.rcllua"
require "rcllua.Node"

local std_srvs = require("std_srvs.srv")

-- Define 'class'
local MinimalService = Node {
  name = "minimal_service",

  -- node constructor
  init = function (self)
    self.is_on = false
    self.service = self:create_service(std_srvs.Trigger, '/state_trigger', self:bind 'call_trigger')
    self.timer = self:create_timer(1.0, 
      function ()
        if self.is_on then self:get_logger():info("I'm working...'") end
      end
    )
  end,
}

-- Main service function
function MinimalService.call_trigger (self, _, resp)
  self.is_on = not self.is_on
  resp.success = true
  resp.message = self.is_on and 'Node is on' or 'Node is off'
end

-- Execute
rcllua:init()

rcllua:spin(MinimalService())

rcllua:shutdown()
