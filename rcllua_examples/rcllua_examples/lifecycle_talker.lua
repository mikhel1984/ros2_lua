-- rcllua example
-- Make simple publisher

-- Load libraries
require "rcllua.rcllua"
require "rcllua.LifecycleNode"

local std_msgs = require("std_msgs.msg")
local tcb_return = LifecycleNode.TransitionCallbackReturn

-- Define 'class'
local LifecyclePublisher = LifecycleNode {
  -- node name
  name = "lc_talker",

  -- node constructor
  init = function (self)
    self.publisher = nil
    self.timer = nil
    self.i = 0    
  end,
  
  publish = function (self)
    local msg = std_msgs.String()
    msg.data = ('Hello World: %d'):format(self.i)
    self.publisher:publish(msg)
    --self:get_logger():info('Publishing: %s', msg.data)
    self.i = self.i + 1
  end,
  
  on_configure = function (self, state)
    self:get_logger():info('on_configure is called')
    self.publisher = self:create_lifecycle_publisher(std_msgs.String, 'lifecycle_chatter', 10)
    self.timer = self:create_timer(0.5, self:bind "publish")
    return tcb_return.SUCCESS
  end,
  
  on_activate = function (self, state)
    self:get_logger():info('on_activate is called')
    return LifecycleNode.on_activate(self, state)
  end,
  
  on_deactivate = function (self, state)
    self:get_logger():info('on_deactivate is called')
    return LifecycleNode.on_deactivate(self, state)
  end,
  
  on_cleanup = function (self, state)
    -- TODO remove publisher and timer
    self:get_logger():info('on_cleanup is called')
    return tcb_return.SUCCESS
  end,
  
  on_shutdown = function (self, state)
    -- TODO destroy publisher and timer
    self:get_logger():info('on_shutdown is called')
    return tcb_return.SUCCESS
  end,
  
}

-- Execute
rcllua:init()

local n = LifecyclePublisher()
rcllua:spin(n)

rcllua:shutdown()
