-- rcllua example
-- Make action client

-- Load libraries
require "rcllua.rcllua"
require "rcllua.Node"
require "rcllua.ActionClient"

local Fibonacci = require("action_tutorials_interfaces.action").Fibonacci

-- Print list elements
local function list_to_string (list)
  return table.concat(list, ' ')
end


-- Defnie 'class'
local FibonacciActionClient = Node {
  name = 'fibonacci_action_client',

  -- node constructor
  init = function (self)
    self.action_client = ActionClient(
      self, 
      Fibonacci, 
      'fibonacci')
  end,

  -- call to set goal
  send_goal = function (self, order)
    self.action_client:wait_for_server()

    local goal_msg = Fibonacci.Goal {order = order}
    local send_goal_future = self.action_client:send_goal_async(
      goal_msg, 
      self:bind 'feedback_cb')
    send_goal_future:add_done_callback(
      self:bind 'get_response_cb')
  end,

  -- call to get response
  get_response_cb = function (self, future)
    local handle = future:result()
    if not handle:accepted() then
      self:get_logger():info('Goal rejected')
      return
    end

    self:get_logger():info('Goal accepted')
    local result_future = handle:get_result_async()
    result_future:add_done_callback(
      self:bind 'get_result_cb')
  end,

  -- process response message
  feedback_cb = function (self, msg)
    local sequence = msg.feedback.partial_sequence
    self:get_logger():info("Feedback: %s", list_to_string(sequence))
  end, 

  -- process result
  get_result_cb = function (self, future)
    local response = future:result()
    local sequence = response.result.sequence
    self:get_logger():info("Result: %s", list_to_string(sequence))
    rcllua:shutdown()
  end
}


-- Execute
rcllua:init()

local node = FibonacciActionClient()
node:send_goal(10)
rcllua:spin(node)

