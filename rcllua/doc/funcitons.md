# RCLLUA methods

This page is generated based on files in *src* and *rcllua* directories.

## C part

### rclbind

**context_init (arg1)**

Initialize context state.
- *arg1* table of command line arguments

**context_ok ()**

Check context status.

Returns
- true if the context is valid.

**context_shutdown ()**

Finalize execution, free common objects.

**is_instance (arg1, arg2, arg4)**

Check message type.
- *arg1* message to check
- *arg2* interface to check
Result:
- *arg4* true when the message has the given type.

**is_timer_ready (arg1)**

Check if the timer is ready using pointer.
- *arg1* timer pointer (light userdata)

Returns
- true when ready.

**new_action_client (arg1, arg2, arg3, arg4, arg10, arg11)**

Create action client object. Save bindings to register.
- *arg1* node object
- *arg2* action type (table)
- *arg3* action name
- *arg4* QoS table (=nil)
-- goal_service_qos
-- result_service_qos
-- cancel_service_qos
-- feedback_topic_qos
-- status_topic_qos
- *arg10* cancel interface
- *arg11* state interface

Returns
- action client object

**new_action_goal_handle (arg1, arg2)**

Create action goal handle object.
- *arg1* action server
- *arg2* GoalInfo message

Returns
- goal handle object

**new_action_server (arg1, arg2, arg3, arg4, arg5, arg15, arg16)**

Create action server object. Save bindings to register.
- *arg1* node object
- *arg2* clock object
- *arg3* action type (table)
- *arg4* action name
- *arg5* parameter table (=nil)
-- goal_service_qos
-- result_service_qos
-- cancel_service_qos
-- feedback_topic_qos
-- status_topic_qos
-- result_timeout (=900)
-- goal_callback (=fn()->true)
-- handle_accepted_callback
-- cancel_callback (=fn()->true)
- *arg15* process function
- *arg16* cancel interface

Returns
- action server object

**new_client (arg1, arg2, arg3, arg4)**

Create client object. Save bindings to register.
- *arg1* node object
- *arg2* service type (table)
- *arg3* service name
- *arg4* qos profile (optional)

Returns
- client object

**new_clock (arg1)**

Create clock object.
- *arg1* clock tipe (int, optional)

Returns
- clock object.

**new_duration (arg1, arg2)**

Create duration object.
- *arg1* seconds (int, optional)
- *arg2* nanoseconds (int, optional)

Returns
- duration object

**new_duration_sec (arg1)**

Create duration object from seconds as floating point value.
- *arg1* seconds (float)

Returns
- duration object

**new_guard_condition (arg1)**

Create guard condition object.
- *arg1* callback (=nil)

Returns
- new guard condition.

**new_lifecycle (arg1, arg2, arg3, arg4)**

Create state machine object.
- *arg1* node object
- *arg2* com interface state flag (=true)
- *arg3* table with required services
- *arg4* table with required messages

Returns
- state machine object

**new_node (arg1, arg2)**

Create node object.
- *arg1* node name
- *arg2* namespace (string, optional)

Returns
- node object.

**new_publisher (arg1, arg2, arg3, arg4)**

Create publisher object.
- *arg1* node object
- *arg2* message type (table)
- *arg3* topic name
- *arg4* qos profile (optional)

Returns
- publisher object

**new_qos (arg1)**

Create QoS object.
- *arg1* QoS tipe (string, optional)

Returns
- QoS object.

**new_service (arg1, arg2, arg3, arg4, arg5, arg6)**

Create service object or wrap the existed one C structure. 
Save bindings to register.
- *arg1* node object
- *arg2* service type (table)
- *arg3* topic name
- *arg4* callback function: fn(request) -> response
- *arg5* qos profile (optional)
- *arg6* rcl service object (optional)

Returns
- service object

**new_subscription (arg1, arg2, arg3, arg4, arg5)**

Create subscription object. Save bindings to register.
- *arg1* node object
- *arg2* message type (table)
- *arg3* topic name
- *arg4* callback function: fn(message) -> nil
- *arg5* qos profile (optional)

Returns
- subscription object

**new_time (arg1, arg2, arg3)**

Create time object.
- *arg1* seconds (int, optional)
- *arg2* nanoseconds (int, optional)
- *arg3* clock tipe (int, optional)

Returns
- time object

**new_timer (arg1, arg2, arg3)**

Create new timer.
- *arg1* clock object.
- *arg2* period, sec (float)
- *arg3* callback function fn(nil) -> nil

Returns
- timer object.

**new_wait_set (arg1, arg2, arg3, arg4, arg5, arg6)**

Init WaitSet object.
- *arg1* subscriptions number
- *arg2* guard conditiona number
- *arg3* timers number
- *arg4* clients number
- *arg5* services number
- *arg6* events number

Returns
- WaitSet object.

**qos_check_compatible (arg1, arg2)**

Check QoS compatibility.
In case of warning return true and warning message.
- *arg1* publisher QoS
- *arg2* subscription QoS

Returns
- boolean
- nil or reason of uncompatibility

**service_send_response (arg1, arg2)**

Send service response.
- *arg1* table from service request receiving
- *arg2* response message

Returns
- flag of success
- error message (optional)

**simp_log (arg1, arg2, arg3)**

Write message short form into log.
- *arg1* severity (enum)
- *arg2* node name
- *arg3* message text

**sleep_thread (arg1)**

Stop execution for some time.
- *arg1* sleep duration, seconds

**timer_call (arg1)**

Call timer, use lightuserdata as timer pointer.
- *arg1* timer pointer

**uuid.new ()**

Generate UUID value.

Returns
- table form uint8[16]
- string form

**uuid.str (arg1)**

String representation for UUID.
- *arg1* message field or table with 16 uint.

Returns
- uuid as string.

**write_log (arg1, arg2, arg3, arg4, arg5, arg6)**

Write message full form into log.
- *arg1* severity (enum)
- *arg2* node name
- *arg3* message text
- *arg4* function name
- *arg5* file name
- *arg6* line number (int)

### Duration

**seconds (arg1)**

Get duration in seconds.
- *arg1* duration object

Returns
- seconds (float)

### LifecycleNode

**available_states (arg1)**

Get available system states.
- *arg1* state machine

Returns
- table of pairs
-- id
-- label

**available_transitions (arg1)**

Get available transitions.
- *arg1* state machine

Returns
- table of groups
-- transition id
-- transigion label
-- start id
-- start label
-- goal id
-- goal label

**current_state (arg1)**

Get current state.
- *arg1* state machine

Returns
- table {id, label}

**get_service (arg1, arg2)**

Get service pointer of the given type.
- *arg1* state machine
- *arg2* service name

Returns
- service as lightuserdata

**get_transition_by_label (arg1, arg2)**

Get state transition by label.
- *arg1* state machine
- *arg2* transition label

Returns
- success of operation
- transition ID or error message

**is_initialized (arg1)**

Check if the state machine is initialized.
- *arg1* state machine object

Returns
- true when initialized
- optional error message for false

**print (arg1)**

Print state machine to console.
- *arg1* state machine

**to_label (arg1, arg2)**

Get label for the given return code.
- *arg1* state machine object
- *arg2* code value

Returns
- code label or nil

**transition_graph (arg1)**

Get transition graph.
- *arg1* state machine

Returns
- table of groups
-- transition id
-- transigion label
-- start id
-- start label
-- goal id
-- goal label

**trigger_transition_by_id (arg1, arg2, arg3)**

Trigger transition by ID.
- *arg1* state machine
- *arg2* transition id
- *arg3* publish flag

**trigger_transition_by_label (arg1, arg2, arg3)**

Trigger transition by label.
- *arg1* state machine
- *arg2* transition label
- *arg3* publish flag

### ActionClient

**add_to_waitset (arg1, arg2)**

Add action client to wait set.
- *arg1* action client
- *arg2* wait set object

**get_interface (arg1, arg2)**

Get message constructor for the specific action structure.
- *arg1* action client
- *arg2* interface name

Returns
- found interface table or nil

**get_num_entities (arg1)**

Get number of enities to update wait set.
- *arg1* action client

Returns
- subcription number
- guard conditions number
- timers number
- clients number
- servers number

**is_action_server_available (arg1)**

Check if the action server is available.
- *arg1* action client

Returns
- true when server is available

**is_ready (arg1, arg2)**

Check ready entries.
- *arg1* action client
- *arg2* wait set object

Returns
- feedback message flag
- status message flag
- goal response flag
- cancel response flag
- result response flag

**send_cancel_request (arg1, arg2, arg3)**

Send request to cancel service.
- *arg1* action client object
- *arg2* request message
- *arg3* callback

Returns
- sequence id

**send_goal_request (arg1, arg2, arg3)**

Send request to goal service.
- *arg1* action client object
- *arg2* request message
- *arg3* callback

Returns
- sequence id

**send_result_request (arg1, arg2, arg3)**

Send request to result service.
- *arg1* action client object
- *arg2* request message
- *arg3* callback

Returns
- sequence id

**set_feedback_method (arg1, arg2, arg3)**

Set function to call for action server feedback.
- *arg1* action client
- *arg2* UUID object (table or message)
- *arg3* function

**take_cancel_response (arg1)**

Get response from cancel service.
- *arg1* action client

Returns
- nil or table {response, feedback, sequence}

**take_feedback (arg1)**

Get feedback message. If callback is registered then
return {message, callback} else nil.
- *arg1* action client

Returns
- nil or {message, callback}

**take_goal_response (arg1)**

Get response from goal service.
- *arg1* action client

Returns
- nil or table {response, feedback, sequence}

**take_result_response (arg1)**

Get response from result service.
- *arg1* action client

Returns
- nil or table {response, feedback, sequence}

**take_status (arg1)**

Get status message.
- *arg1* action client

Returns
- status message

### Clock

**clock_type (arg1)**

Get clock type.
- *arg1* clock object

Returns
- type value (int)

**now (arg1)**

Get current time.
- *arg1* clock object.

Returns
- time object.

**set_ros_time_override (arg1, arg2)**

Override ROS time.
- *arg1* clock object.
- *arg2* time object.

**set_ros_time_override_is_enabled (arg1, arg2)**

Set override status for the ROS time.
- *arg1* clock object.
- *arg2* override flag

### Timer

**call (arg1)**

Call timer.
- *arg1* timer object

**cancel (arg1)**

Cancel timer.
- *arg1* timer object

**is_canceled (arg1)**

Check timer status.
- *arg1* timer object

Returns
- true if the timer is canceled

**is_ready (arg1)**

Check if the timer is ready.
- *arg1* timer object.

Returns
- true when ready.

**period (arg1)**

Get timer period, in seconds.
- *arg1* timer object

Returns
- period (seconds, float)

**reset (arg1)**

Reset timer state.
- *arg1* timer object

**set_period (arg1, arg2)**

Change timer period.
- *arg1* timer object
- *arg2* new period, seconds

Returns
- old period, seconds

**time_since_last_call (arg1)**

Get time since last call, in seconds.
- *arg1* timer object

Returns
- current time (seconds, float)

**time_until_next_call (arg1)**

Get time until the next call, in seconds.
- *arg1* timer object

Returns
- rest time (seconds, float)

### Node

**get_action_client_names_and_types_by_node (arg1, arg2, arg3)**

Get action client names and types by node.
- *arg1* node object
- *arg2* remote node name
- *arg3* remote node namespace

Returns
- table of names and types

**get_action_names_and_types (arg1)**

Get action names and types by node.
- *arg1* node object

Returns
- table of names and types

**get_action_server_names_and_types_by_node (arg1, arg2, arg3)**

Get action server names and types by node.
- *arg1* node object
- *arg2* remote node name
- *arg3* remote node namespace

Returns
- table of names and types

**get_count_publishers (arg1, arg2)**

Get number of publishers.
- *arg1* node object
- *arg2* topic name

Returns
- number of publishers

**get_count_subscribers (arg1, arg2)**

Get number of subscribers.
- *arg1* node object
- *arg2* topic name

Returns
- number of subscribers

**get_fully_qualified_name (arg1)**

Get fully qualified node name.
- *arg1* node object

Returns
- node name

**get_logger_name (arg1)**

Get logger name.
- *arg1* node object

Returns
- logger name

**get_name (arg1)**

Get node name.
- *arg1* node object

Returns
- node name

**get_namespace (arg1)**

Get current namespace.
- *arg1* node object

Returns
- namespace

### Publisher

**get_logger_name (arg1)**

Get node logger name.
- *arg1* publisher object

Returns
- logger name

**get_subscription_count (arg1)**

Get number of subscriptions.
- *arg1* publisher object

Returns
- subscription number.

**get_topic_name (arg1)**

Get topic name.
- *arg1* publisher object

Returns
- topic name.

**publish (arg1, arg2)**

Send message.
- *arg1* publisher object
- *arg2* message object

**wait_for_all_acked (arg1, arg2)**

Wait untill all published message data is acknowledged.
- *arg1* publisher object
- *arg2* duration object

Returns
- false when time is out

### ActionGoalHandle

**get_status (arg1)**

Get goal status.
- *arg1* goal handle

Returns
- goal status (integer)

**is_active (arg1)**

Check if the goal is active.
- *arg1* goal handle

Returns
- true if the goal is active

**update_goal_state (arg1, arg2)**

Set new goal status.
- *arg1* goal handle
- *arg2* event (integer)

### ActionServer

**add_to_waitset (arg1, arg2)**

Add action server to wait set.
- *arg1* action server
- *arg2* wait set object

**expired_goals (arg1, arg2)**

Get list of expire goals.
- *arg1* action server
- *arg2* total number of goals

Returns
- table of goal ID's (as string)

**get_executable (arg1)**

Get registered action process.
- *arg1* action server object

Returns
- function

**get_handle_preprocessing (arg1)**

Get method that may do additional configuration and run the main process.
The method takes goal handle and run execution.
- *arg1* action server object

Returns
- function

**get_interface (arg1, arg2)**

Get message constructor for the specific action structure.
- *arg1* action server
- *arg2* interface name

Returns
- found interface table or nil

**get_num_entities (arg1)**

Get number of entries to add to wait set.
- *arg1* actoin server

Returns
- subscription number
- guard number
- timer number
- client number
- service number

**is_ready (arg1, arg2)**

Check ready entries.
- *arg1* action server
- *arg2* wait set object

Returns
- goal request flag
- cancel request flag
- result request flag
- goal expired flag

**notify_goal_done (arg1)**

Notify server about finished task.
- *arg1* action server

**process_cancel_request (arg1, arg2)**

Process cancel request, make response.
- *arg1* action server
- *arg2* cancel request object

Returns
- response message

**publish_feedback (arg1, arg2)**

Send feedback message.
- *arg1* action server
- *arg2* feedback message

**publish_status (arg1)**

Send action server status.
- *arg1* action server

**send_cancel_response (arg1, arg2, arg3)**

Send cancel response.
- *arg1* action server object
- *arg2* response message
- *arg3* header object

Returns
- true in case of success

**send_goal_response (arg1, arg2, arg3)**

Send goal response.
- *arg1* action server object
- *arg2* response message
- *arg3* header object

Returns
- true in case of success

**send_result_response (arg1, arg2, arg3)**

Send result response.
- *arg1* action server object
- *arg2* response message
- *arg3* header object

Returns
- true in case of success

**take_cancel_request (arg1)**

Take cancel request.
- *arg1* action server object

Returns
- table {request, callback, header}

**take_goal_request (arg1)**

Take goal request.
- *arg1* action server object

Returns
- table {request, callback, header}

**take_result_request (arg1)**

Take result request.
- *arg1* action server object

Returns
- table {request, callback, header}

### Time

**seconds (arg1)**

Get time in seconds.
- *arg1* time object

Returns
- seconds (float)

### WaitSet

**add_client (arg1, arg2)**

Add client for waiting.
- *arg1* WaitSet object
- *arg2* client object

Returns
- index of added client

**add_guard_condition (arg1, arg2)**

Add guard condition.
- *arg1* WaitSet object
- *arg2* guard condition object

Returns
- index of added client

**add_service (arg1, arg2)**

Add service for waiting.
- *arg1* WaitSet object
- *arg2* service object

Returns
- index of added service

**add_subscription (arg1, arg2)**

Add subscription for waiting.
- *arg1* WaitSet object
- *arg2* subscription object

Returns
- index of added subscription

**add_timer (arg1, arg2)**

Add timer for waiting.
- *arg1* WaitSet object
- *arg2* timer object

Returns
- index of added timer

**clear (arg1)**

Clear WaitSet object.
- *arg1* WaitSet object

**ready_clients (arg1)**

Collect ready client response.
- *arg1* WaitSet object

Returns
- table of tuples (response, callback)

**ready_guard_conditions (arg1)**

Collect ready guard conditions.
- *arg1* WaitSet object

Returns
- table of callbacks.

**ready_services (arg1)**

Collect ready service requests.
- *arg1* WaitSet object

Returns
- table of tuples (response, response, ...)

**ready_subscriptions (arg1)**

Collect ready subscriptions.
- *arg1* WaitSet object

Returns
- table of tuples (message, callback)

**ready_timers (arg1)**

Collect ready timers.
- *arg1* WaitSet object

Returns
- table of callback functions

**wait (arg1, arg2)**

Waiting for the next ready object.
- *arg1* WaitSet object
- *arg2* timeout  (nanoseconds)

Returns
- true when time is out

### Subscription

**get_logger_name (arg1)**

Get node logger name.
- *arg1* subscription object

Returns
- logger name

**get_topic_name (arg1)**

Get topic name.
- *arg1* subscription object

Returns
- topic name

### Service

**get_name (arg1)**

Get service name.
- *arg1* service object

Returns
- name string

**get_qos (arg1)**

Get QoS profile.
- *arg1* service object

Returns
- qos object

### Client

**remove_pending_request (arg1, arg2)**

Remove pending request.
- *arg1* client object
- *arg2* request sequence number

**send_request (arg1, arg2, arg3)**

Send request.
- *arg1* client object
- *arg2* request message
- *arg3* callback

Returns
- sequence id

**service_is_available (arg1)**

Check if the service is available.
- *arg1* client object

Returns
- true when available

## Lua part

### state_srv

**ChangeState  (node, req)**

Do change state.
- *node* LifecycleNode object.
- *req* Request with new state (ID or label).

Returns
- response with result of operation.

**GetAvailableStates  (node, req)**

Get list of available states.
- *node* LifecycleNode object.
- *req* Request.

Returns
- response with list of states.

**GetAvailableTransitions  (node, req)**

Get transitions from the current state.
- *node* LifecycleNode object.
- *req* Request.

Returns
- response with transitions from the current state.

**GetState  (node, req)**

Get current state.
- *node* LifecycleNode object.
- *req* Request.

Returns
- response with state information.

**GetTransitionGraph  (node, req)**

Get graph of transitions.
- *node* LifecycleNode object.
- *req* Request.

Returns
- response with list of transitions.

### ServerGoalHandle

**abort (self)**

Set status 'abort'.

**canceled (self)**

Set status 'canceled'.

**execute (self)**

Start process execution.

**is_active (self)**

Check if the goal handle is active.

Returns
- true when goal is active.

**is_cancel_requested (self)**

Check if the goal is canceled.

Returns
- true for canceled goal.

**publish_feedback (self, msg)**

Send feedback to client.
- *msg* Feedback message.

**status (self)**

Get goal status.

Returns
- goal status (enum).

**succeed (self)**

Set status 'succeed'.

### LifecycleNode

**add_managed_entity (self, entity)**

Add ManagedEntity object.
- *entity* Object to add.

**create_lifecycle_publisher (self, ...)**

Create publisher and add to managed entity list.
- ... Publisher parameters.

Returns
- publisher object.

**on_activate (self, state)**

Default activate callback.
- *state* Current state.

Returns
- transition result code.

**on_cleanup (self, state)**

Default cleanup callback.
- *state* Current state.

Returns
- transition result code.

**on_configure (self, state)**

Default configure callback.
- *state* Current state.

Returns
- transition result code.

**on_deactivate (self, state)**

Default deactivate callback.
- *state* Current state.

Returns
- transition result code.

**on_error (self, state)**

Default error callback.
- *state* Current state.

Returns
- transition result code.

**on_shutdown (self, state)**

Default shutdown callback.
- *state* Current state.

Returns
- transition result code.

**trigger_activate (self)**

Call activate transition.

Returns
- transition result code.

**trigger_cleanup (self)**

Call cleanup transition.

Returns
- transition result code.

**trigger_configure (self)**

Call configure transition.

Returns
- transition result code.

**trigger_deactivate (self)**

Call deactivate transition.

Returns
- transition result code.

**trigger_shutdown (self)**

Call shutdown transition.

Returns
- transition result code.

### ActionClient

**add_to_waitset (self, wait_set)**

Add action client object to wait set.

**execute (self, data)**

Process incoming data. The method works inside a Lua coroutine.
- *data* Table with incoming data.

**get_num_entities (self)**

Get number of available interfaces.

Returns
- 5 numbers.

**send_goal (self, goal, cb_feedback, uuid, timeout_sec)**

Send new goal request to action server and wait for result.
- *goal* Goal message.
- *cb_feedback* (=nil) Function to process feedback messages (optional).
- *uuid* (=nil) Task UUID (optional).
- *timeout_sec* (=nil) Time to wait (optional).

Returns
- server request or nil.

**send_goal_async (self, goal, cb_feedback, uuid)**

Send new goal request to action server.
- *goal* Goal message.
- *cb_feedback* (=nil) Function to process feedback messages (optional).
- *uuid* (=nil) Task UUID (optional).

Returns
- Future object.

**server_is_ready (self)**

Check if the action server is available.

Returns
- true if server is ready.

**take_data (self, wait_set)**

Check if there are available messages.
- *wait_set* WaitSet object.

Returns
- table with incoming data or nil.

**wait_for_server (self, timeout_sec)**

Sleep until action server become ready.
- *timeout_sec* (=inf) Wait time (optional).

Returns
- true if service is ready.

### Parameter

**from_parameter_message (msg)**

Make Parameter object from Parameter message.
- *msg* Parameter message.

Returns
- Parameter object.

**from_parameter_value (value)**

Define type based on the given value.
- *value* Parameter value.

Returns
- type index.

**get_parameter_value (self)**

Fill ParameterValue message.

Returns
- ParameterValue object.

**name (self)**

Get parameter name.

Returns
- name.

**new_parameter (name, tp, value)**

Parameter constructor.
- *name* Name string.
- *tp* (=nil) Type index.
- *value* (=nil) Parameter value.

**to_parameter_msg (self)**

Fill Parameter message.

Returns
- Parameter object.

**type (self)**

Get parameter type.

Returns
- type index.

**value (self)**

Get parameter value.

Returns
- value.

### Node

**add_waitable (self, action)**

Add action client or server to the node.
- *action* Action client or server object.

**bind (self, name)**

Wrapper for function binding.
Allows to call function fn(obj, arg1...argN) as _fn(arg1...argN).
- *name* Function name in node table.

Returns
- function for binding.

**clients (self)**

Make iterator for the node clients.

Returns
- iterator.

**create_client (self, srv, name, qos)**

Create client object.
- *srv* Service type.
- *name* Service name.
- *qos* QoS profile (optional).

Returns
- client (table).

**create_guard_condition (self, callback)**

Create guard condition object.
- *callback* Callback method (optional).

Returns
- guard condition (userdata).

**create_publisher (self, msg, topic, qos)**

Create publisher object.
- *msg* Message type.
- *topic* Topic name.
- *qos* QoS profile (optional).

Returns
- publisher (userdata).

**create_service (self, srv, name, func, qos)**

Create service object.
- *srv* Service type.
- *name* Service name.
- *func* Service function fn(request, response) --> nil.
- *qos* QoS profile (optional).

Returns
- service (userdata).

**create_subscription (self, msg, topic, qos, callback)**

Create subscription object.
- *msg* Message type.
- *topic* Topic name.
- *qos* QoS profile.
- *callback* Callback function fn(message) --> nil.

Returns
- subscription (userdata).

**create_timer (self, period, callback)**

Create timer object.
- *period* Time in seconds.
- *callback* Function to execute fn() --> nil.

Returns
- timer (userdata).

**declare_parameter (self, name, value, descriptor, ignore_override)**

Declare and initialize parameter.
- *name* Fully-qualified name of the parameter.
- *value* (=nil) Value of the parameter to declare.
- *descriptor* (=nil) Descriptor of the parameter to declare.
- *ignore_override* (=false) True if overrides should ot be taken into account.

Returns
- parameter with assigned value.

**declare_parameters (self, namespace, params, ignore_override)**

Declare a list of parameters.
- *namespace* Namespace for parameters.
- *params* List of tuples {name, value, type, ParameterDescriptor}
- *ignore_override* (=false) True if overrides should not be taken into account.

Returns
- parameter list.

**destroy_node (self)**

Free resources.

**executor (self)**

Get current Executor object.

Returns
- reference to executor.

**get_clock (self)**

Get clock object.

Returns
- node clock.

**get_fully_qualified_name (self)**

Get node fully qualified name.

Returns
- node name string.

**get_logger (self)**

Create object for logging.

Returns
- logger.

**get_name (self)**

Get node name.

Returns
- name string.

**get_namespace (self)**

Get node namespace.

Returns
- node namespace string.

**get_shortest_time (self)**

Check if there is wait time in queue.

Returns
- non-negative duration or infinity if the queue is empty.

**get_time_msg (self, t)**

Get time as builtin_interfaces.Time object.
Try to load interface first. Return get_clock():now() by default.
- *t* (=nil) rcllua time object.

Returns
- time representation in form of builtin_interfaces.Time object.

**get_waited_list (self)**

Get list of yielded threads.

Returns
- table with coroutines.

**has_parameter (self, name)**

Check if the parameter is defined.
- *name* Parameter name.

Returns
- true if the parameter is found.

**publishers (self)**

Make iterator for the node publishers.

Returns
- iterator.

**remove_client (self, client)**

Remove client from the node.
- *client* Client object.

Returns
- status of removing.

**remove_guard_condition (self, guard)**

Remove guard condition from the node.
- *guard* Guard condition object.

Returns
- status of removing.

**remove_publisher (self, pub)**

Remove publisher from the node.
- *pub* Publisher object.

Returns
- status of removing.

**remove_service (self, service)**

Remove service from the node.
- *service* Service object.

Returns
- status of removing.

**remove_subscription (self, sub)**

Remove subscription from the node.
- *sub* Subscription object.

Returns
- status of removing.

**remove_timer (self, timer)**

Remove timer from the node.
- *timer* Timer object.

Returns
- status of removing.

**services (self)**

Make iterator for the node services.

Returns
- iterator.

**set_executor (self, executor)**

Update reference to Executor object.
- *executor* New reference or nil.

**set_parameters (self, params)**

Set parameters.
- *params* The list of parameters to set.

Returns
- list of results for every set action.

**subscriptions (self)**

Make iterator for the node subscriptions.

Returns
- iterator.

**timers (self)**

Make iterator for the node timers.

Returns
- iterator.

**wait (self, condition, timeout)**

"Sleep" until the condition is fulfilled.
- *condition* Function funciton() -> bool or timeout in seconds.
- *timeout* Timeout in seconds or nil.

**waitables (self)**

Make iterator for the node action clients and services.

Returns
- iterator.

**wrap (self, name)**

Similar to 'bind' method, but it puth function to coroutine.
It allows to use 'wait' and suspend execution.
- *name* Function name in node table.

Returns
- function with coroutine inside.

### Future

**_set_result(self, value)**

Update result field.
- *value* Result.

**add_done_callback (self, func)**

Set / update callback method.
- *func* New callback function.

**done (self)**

Check if the task is done.

Returns
- true when completed.

**result (self)**

Get task result.

Returns
- current result.

### ActionServer

**add_to_waitset (self, wait_set)**

Add action server object to wait set.

**execute (self, data)**

Incoming data processing.
- *data* New requests.

**get_num_entities (self)**

Get number of entities for wait set.
- *5* numbers.

**take_data (self, wait_set)**

Check WaitSet for incoming data.
- *wait_set* WaitSet object.

Returns
- table with data or nil.

### rcllua

**init (self, tbl)**

Initialize ROS environment.
- *tbl* Initialization arguments, CLI arguments by default.

**sleep_sec (self, time)**

Stop execution for some time.
- *time* Sleep time (float).

**spin (self, node, executor)**

Run execution loop.
- *node* Node object.
- *executor* Executor object (optional)

**spin_until_future_complete (self, node, future, executor, timeout_sec)**

Run spinning until Future task is complete or time is out.
- *node* Node object.
- *future* Future object.
- *executor* Executor object (optional).
- *timeout_sec* Wait time (optional).

**tostring (self, msg)**

Get string representation for a ROS message.
- *msg* Message object.

Returns
- string representation.

### Logger

**debug (self, ...)**

Pring message with DEBUG level.
- ... Format string and parameters.

**error (self, ...)**

Pring message with ERROR level.
- ... Format string and parameters.

**fatal (self, ...)**

Pring message with FATAL level.
- ... Format string and parameters.

**info (self, ...)**

Pring message with INFO level.
- ... Format string and parameters.

**warn (self, ...)**

Pring message with WARN level.
- ... Format string and parameters.

### ClientGoalHandle

**accepted (self)**

Check if the goal is accepted.

Returns
- true when accepted

**cancel_goal (self, timeout_sec)**

Send cancel request and wait for response.
- *timeout_sec* (=nil) Time to wait.

Returns
- server response or nil.

**cancel_goal_async (self)**

Send cancel request.

Returns
- Future object.

**get_result (self, timeout_sec)**

Send result request and wait for response.
- *timeout_sec* (=nil) Time to wait.

Returns
- server response or nil.

**get_result_async (self)**

Send result request.

Returns
- Future object.

**goal_id (self)**

Get goal UUID.

Returns
- UUID object.

**stamp (self)**

Get response time.

Returns
- Time object.

### parameter_service

**_describe_parameter_callback (node, req, resp)**

Method for parameter description.
- *node* Node object.
- *req* Request with list of names.
- *resp* Response with list of descriptors.

**_get_parameter_types_callback (node, req, resp)**

Method for getting parameter types.
- *node* Node object.
- *req* Request with list of names.
- *resp* Response with list of types.

**_get_parameters_callback (node, req, resp)**

Method for getting parameter list.
- *node* Node object.
- *req* Request with list of names.
- *resp* Response with list of parameters.

**_list_parameters_callback (node, req, resp)**

Get parameters with specific prefix.
- *node* Node object.
- *req* Request with the prefix list.
- *resp* Response with names and prefixes.

**_set_parameters_atomically_callback (node, req, resp)**

Update parameters atomically.
- *node* Node object.
- *req* Request with new parameters.
- *resp* Response with result of operation.

**_set_parameters_callback (node, req, resp)**

Update parameters.
- *node* Node object.
- *req* Request with new parameters.
- *resp* Response with result of operation.

**new_service (node)**

Initialize parameter services.
- *node* Node object.

### Client

**call (self, req, timeout_sec)**

Send request to server and wait for response.
- *req* Request object.
- *timeout_sec* (=nil) Wait time (optional).

Returns
- response object or nil (in the case of time out).

**call_async (self, req, callback)**

Send request to server.
- *req* Request object.
- *callback* (=nil) Function to execute when get response (optional) fn(response) --> nil.

Returns
- Future object.

**handle (self)**

Get client object.

Returns
- client (userdata).

**new_client (...)**

Client object constructor.
- ... Initialization parameters defined in Node.create_client.

Returns
- client (table).

**remove_pending_request (self, future)**

Free request callback.
- *future* Future object.

**service_is_ready (self)**

Check if the service is ready.

Returns
- true when service is available.

**wait_for_service (self, timeout_sec)**

Sleep until service become ready.
- *timeout_sec* Wait time.

Returns
- true if service is ready.

### Executor

**add_node (self, node)**

Add Node object.
- *node* Object to add.

Returns
- true if the node is new.

**remove_node (self, node)**

Remove node object.
- *node* Object to remove.

Returns
- true if node found.

**spin (self)**

Run data spin.

**spin_once (self, timeout_sec)**

Spin until time is out or got new data.
- *timeout_sec* Wait time (optional).

**spin_until_future_complete (self, future, timeout_sec)**

Spin data until time is out or task is completed.
- *future* Future object.
- *timeout_sec* Wait time (optional).

