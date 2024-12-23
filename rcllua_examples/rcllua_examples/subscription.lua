-- rcllua example
-- Make simple subscriber

-- Load libraries
require "rcllua.rcllua"
require "rcllua.Node"

local std_msgs = require("std_msgs.msg")

-- Define 'class'
local MinimalSubscriber = Node {
  -- node name
  name = "minimal_subscriber",

  -- node constructor
  init = function (self)
    self.subscription = self:create_subscription(
      std_msgs.String,
      'topic',
      self:bind 'listener_callback',  -- define callback with Node.bind
      10)
  end,

  -- callback method
  listener_callback = function (self, msg)
    self:get_logger():info('I heard: %s', msg.data)
  end
}

-- Execute
rcllua:init(arg)

rcllua:spin(MinimalSubscriber())

rcllua:shutdown()
