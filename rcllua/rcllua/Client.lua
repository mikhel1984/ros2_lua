-- Copyright 2025 Stanislav Mikhel
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--     http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

local rclbind = require("rcllua.rclbind")

--- Future class.
local Future = {}
Future.__index = Future

--- Check if the task is done.
--  @return true when completed.
function Future.done (self)
  return self._is_done
end

--- Get task result.
--  @return current result.
function Future.result (self)
  return self._result
end

--- Execute callback if any.
function Future.__call (self)
  if self._callback and self._result then
    self._callback(self._result)
  end
end

--- Future object constructor.
--  @param fn Callback function.
--  @return new Future object.
local function new_future (fn)
  local o = {
    _is_future = true,
    _is_done = false,
    _req_id = -1,
    _result = nil,
    _callback = fn,
  }
  return setmetatable(o, Future)
end

--- Client class.
local Client = {}
Client.__index = Client

--- Client object constructor.
--  @param ... Initialization parameters defined in Node.create_client.
--  @return client (table).
function Client.new_client (...)
  local o = {}
  o._client = rclbind.new_client(...)
  return setmetatable(o, Client)
end

--- Check if the service is ready.
--  @return true when service is available.
function Client.service_is_ready (self)
  return self._client:service_is_available()
end

--- Send request to server.
--  @param req Request object.
--  @param callback Function to execute when get response (optional) fn(response) --> nil.
--  @return Future object.
function Client.call_async (self, req, callback)
  local future = new_future(callback)
  local future_cb = function (msg)
    future._result = msg
    future._is_done = true
  end
  future._req_id = self._client:send_request(req, future_cb)
  return future
end

--- Sleep until service become ready.
--  @param timeout_sec Wait time.
--  @return true if service is ready.
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

--- Get client object.
--  @return client (userdata).
function Client.handle (self)
  return self._client
end

return Client
