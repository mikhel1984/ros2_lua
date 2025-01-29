-- rcllua example
-- Make action client

-- Load libraries
require "rcllua.rcllua"
require "rcllua.Node"
require "rcllua.ActionClient"

local Fibonacci = require("action_tutorials_interfaces.action").Fibonacci

local function list_to_string (list)
  return table.concat(list, ' ')
end

local FibonacciActionClient = Node {
  name = 'fibonacci_action_client',

  init = function (self)
    self.action_client = ActionClient(self, Fibonacci, 'fibonacci')
  end,

  send_goal = function (self, order)
    local goal_msg = Fibonacci.Goal {order = order}
    self.action_client:wait_for_server()
    local send_goal_future = self.action_client:send_goal_async(goal_msg, self:bind 'feedback_cb')
    send_goal_future:add_done_callback(self:bind 'get_response_cb')
  end,

  get_response_cb = function (self, future)
    local handle = future:result()
    if not handle:accepted() then
      self:get_logger():info('Goal rejected')
      return
    end
    self:get_logger():info('Goal accepted')
    local result_future = handle:get_result_async()
    result_future:add_done_callback(self:bind 'get_result_cb')
  end,

  feedback_cb = function (self, msg)
    self:get_logger():info("Feedback: %s", list_to_string(msg.feedback.partial_sequence))
  end, 

  get_result_cb = function (self, future)
    local response = future:result()
    self:get_logger():info("Result: %s", list_to_string(response.result.sequence))
  end
}


-- Execute
rcllua:init()

local node = FibonacciActionClient()
node:send_goal(10)
rcllua:spin(node)

rcllua:shutdown()

