-- rcllua example
-- Make lifecycle publisher

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
    self.managed_pub = nil
    self.timer = nil
    self.i = 0
  end,

  publish = function (self)
    local msg = std_msgs.String()
    msg.data = ('Hello World: %d'):format(self.i)
    if not self.managed_pub or not self.managed_pub:is_activated() then
      self:get_logger():info('Publisher is inactive')
      return
    else
      -- call managed entity directly
      self.managed_pub.entity:publish(msg)
    end
    self:get_logger():info('Publishing: %s', msg.data)
    self.i = self.i + 1
  end,

  on_configure = function (self, state)
    self:get_logger():info('on_configure is called')
    self.managed_pub = self:create_lifecycle_publisher(std_msgs.String, 'lifecycle_chatter', 10)
    self.timer = self:create_timer(1.0, self:bind "publish")
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

rcllua:spin(LifecyclePublisher())

rcllua:shutdown()
