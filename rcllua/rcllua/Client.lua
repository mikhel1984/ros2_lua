
local rclbind = require("rcllua.rclbind")

local Future = {}
Future.__index = Future

function Future.done (self)
  return self._is_done
end

function Future.result (self)
  return self._result
end

function Future.__call (self)
  if self._callback and self._result then 
    self._callback(self._result)
  end
end

function new_future (fn)
  local o = {
    _is_future = true,
    _is_done = false,
    _req_id = -1,
    _result = nil,
    _callback = fn,
  }
  return setmetatable(o, Future)
end

local Client = {}
Client.__index = Client


function Client.new_client (...)
  local o = {}
  o._client = rclbind.new_client(...)
  return setmetatable(o, Client)
end

function Client.service_is_ready (self)
  return self._client:service_is_available()
end

function Client.call_async (self, req, callback)
  local future = new_future(callback)
  local future_cb = function (msg)
    future._result = msg
    future._is_done = true
  end
  future._req_id = self._client:send_request(req, future_cb)
  return future
end

function Client.wait_for_service (self, timeout_sec)
  local sleep_time = math.min(0.2, timeout_sec)
  timeout_sec = timeout_sec or math.huge
  while rclbind.context_ok() and not self._client:service_is_available() 
    and timeout_sec > 0
  do
    rclbind.sleep_thread(sleep_time)
    timeout_sec = timeout_sec - sleep_time
  end
  return self._client:service_is_available()
end

function Client.handle (self)
  return self._client
end


return Client
