-- rcllua example
-- Make simple publisher

-- Load libraries
require "rcllua.rcllua"
require "rcllua.Node"

local std_msgs = require("std_msgs.msg")

-- Define 'class'
local MinimalPublisher = Node {
  -- node name
  name = "minimal_publisher",

  -- node constructor
  init = function (self)
    self.publisher = self:create_publisher(std_msgs.String, 'topic', 10)
    self.i = 0
    self.timer = self:create_timer(0.5,  -- period, seconds
      -- define callback with lambda
      function ()
        local msg = std_msgs.String()
        msg.data = ('Hello World: %d'):format(self.i)
        self.publisher:publish(msg)
        self:get_logger():info('Publishing: %s', msg.data)
        self.i = self.i + 1
      end)
  end
}

-- Execute
rcllua:init()

rcllua:spin(MinimalPublisher())

rcllua:shutdown()
