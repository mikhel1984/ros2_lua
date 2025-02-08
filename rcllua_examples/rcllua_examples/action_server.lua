-- rcllua example
-- Make action server

-- Load libraries
require "rcllua.rcllua"
require "rcllua.Node"
require "rcllua.ActionServer"

local Fibonacci = require("action_tutorials_interfaces.action").Fibonacci


-- Define 'class'
local FibonacciActionServer = Node {
  name = 'fibonacci_action_server',

  -- node constructor
  init = function (self)
    self.action_server = ActionServer(
      self,
      Fibonacci,
      'fibonacci',
      self:bind 'execute_callback')
  end,

  -- main process
  execute_callback = function (self, goal_handle)
    self:get_logger():info("Executing goal..")

    local msg = Fibonacci.Feedback()
    local sequence = {0, 1}
    for i = 1, goal_handle.request.order do
      sequence[#sequence+1] = sequence[#sequence-1] + sequence[#sequence]
      msg.partial_sequence(sequence)     -- copy from table
      -- intermediate status
      self:get_logger():info("Feedback: %s", table.concat(sequence, ' '))
      goal_handle:publish_feedback(msg)
      self:wait(1.0)  -- yiel
    end

    goal_handle:succeed()

    -- final result
    local result = Fibonacci.Result()
    result.sequence = msg.partial_sequence
    return result
  end
}


-- Execute
rcllua:init()

rcllua:spin(FibonacciActionServer())

rcllua:shutdown()
