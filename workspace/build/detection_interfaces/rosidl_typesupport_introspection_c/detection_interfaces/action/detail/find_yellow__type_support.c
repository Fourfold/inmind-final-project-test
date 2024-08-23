// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from detection_interfaces:action/FindYellow.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "detection_interfaces/action/detail/find_yellow__rosidl_typesupport_introspection_c.h"
#include "detection_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "detection_interfaces/action/detail/find_yellow__functions.h"
#include "detection_interfaces/action/detail/find_yellow__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void detection_interfaces__action__FindYellow_Goal__rosidl_typesupport_introspection_c__FindYellow_Goal_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  detection_interfaces__action__FindYellow_Goal__init(message_memory);
}

void detection_interfaces__action__FindYellow_Goal__rosidl_typesupport_introspection_c__FindYellow_Goal_fini_function(void * message_memory)
{
  detection_interfaces__action__FindYellow_Goal__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember detection_interfaces__action__FindYellow_Goal__rosidl_typesupport_introspection_c__FindYellow_Goal_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(detection_interfaces__action__FindYellow_Goal, structure_needs_at_least_one_member),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers detection_interfaces__action__FindYellow_Goal__rosidl_typesupport_introspection_c__FindYellow_Goal_message_members = {
  "detection_interfaces__action",  // message namespace
  "FindYellow_Goal",  // message name
  1,  // number of fields
  sizeof(detection_interfaces__action__FindYellow_Goal),
  detection_interfaces__action__FindYellow_Goal__rosidl_typesupport_introspection_c__FindYellow_Goal_message_member_array,  // message members
  detection_interfaces__action__FindYellow_Goal__rosidl_typesupport_introspection_c__FindYellow_Goal_init_function,  // function to initialize message memory (memory has to be allocated)
  detection_interfaces__action__FindYellow_Goal__rosidl_typesupport_introspection_c__FindYellow_Goal_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t detection_interfaces__action__FindYellow_Goal__rosidl_typesupport_introspection_c__FindYellow_Goal_message_type_support_handle = {
  0,
  &detection_interfaces__action__FindYellow_Goal__rosidl_typesupport_introspection_c__FindYellow_Goal_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_detection_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, detection_interfaces, action, FindYellow_Goal)() {
  if (!detection_interfaces__action__FindYellow_Goal__rosidl_typesupport_introspection_c__FindYellow_Goal_message_type_support_handle.typesupport_identifier) {
    detection_interfaces__action__FindYellow_Goal__rosidl_typesupport_introspection_c__FindYellow_Goal_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &detection_interfaces__action__FindYellow_Goal__rosidl_typesupport_introspection_c__FindYellow_Goal_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "detection_interfaces/action/detail/find_yellow__rosidl_typesupport_introspection_c.h"
// already included above
// #include "detection_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "detection_interfaces/action/detail/find_yellow__functions.h"
// already included above
// #include "detection_interfaces/action/detail/find_yellow__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void detection_interfaces__action__FindYellow_Result__rosidl_typesupport_introspection_c__FindYellow_Result_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  detection_interfaces__action__FindYellow_Result__init(message_memory);
}

void detection_interfaces__action__FindYellow_Result__rosidl_typesupport_introspection_c__FindYellow_Result_fini_function(void * message_memory)
{
  detection_interfaces__action__FindYellow_Result__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember detection_interfaces__action__FindYellow_Result__rosidl_typesupport_introspection_c__FindYellow_Result_message_member_array[3] = {
  {
    "found",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(detection_interfaces__action__FindYellow_Result, found),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cx",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(detection_interfaces__action__FindYellow_Result, cx),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cy",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(detection_interfaces__action__FindYellow_Result, cy),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers detection_interfaces__action__FindYellow_Result__rosidl_typesupport_introspection_c__FindYellow_Result_message_members = {
  "detection_interfaces__action",  // message namespace
  "FindYellow_Result",  // message name
  3,  // number of fields
  sizeof(detection_interfaces__action__FindYellow_Result),
  detection_interfaces__action__FindYellow_Result__rosidl_typesupport_introspection_c__FindYellow_Result_message_member_array,  // message members
  detection_interfaces__action__FindYellow_Result__rosidl_typesupport_introspection_c__FindYellow_Result_init_function,  // function to initialize message memory (memory has to be allocated)
  detection_interfaces__action__FindYellow_Result__rosidl_typesupport_introspection_c__FindYellow_Result_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t detection_interfaces__action__FindYellow_Result__rosidl_typesupport_introspection_c__FindYellow_Result_message_type_support_handle = {
  0,
  &detection_interfaces__action__FindYellow_Result__rosidl_typesupport_introspection_c__FindYellow_Result_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_detection_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, detection_interfaces, action, FindYellow_Result)() {
  if (!detection_interfaces__action__FindYellow_Result__rosidl_typesupport_introspection_c__FindYellow_Result_message_type_support_handle.typesupport_identifier) {
    detection_interfaces__action__FindYellow_Result__rosidl_typesupport_introspection_c__FindYellow_Result_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &detection_interfaces__action__FindYellow_Result__rosidl_typesupport_introspection_c__FindYellow_Result_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "detection_interfaces/action/detail/find_yellow__rosidl_typesupport_introspection_c.h"
// already included above
// #include "detection_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "detection_interfaces/action/detail/find_yellow__functions.h"
// already included above
// #include "detection_interfaces/action/detail/find_yellow__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void detection_interfaces__action__FindYellow_Feedback__rosidl_typesupport_introspection_c__FindYellow_Feedback_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  detection_interfaces__action__FindYellow_Feedback__init(message_memory);
}

void detection_interfaces__action__FindYellow_Feedback__rosidl_typesupport_introspection_c__FindYellow_Feedback_fini_function(void * message_memory)
{
  detection_interfaces__action__FindYellow_Feedback__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember detection_interfaces__action__FindYellow_Feedback__rosidl_typesupport_introspection_c__FindYellow_Feedback_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(detection_interfaces__action__FindYellow_Feedback, structure_needs_at_least_one_member),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers detection_interfaces__action__FindYellow_Feedback__rosidl_typesupport_introspection_c__FindYellow_Feedback_message_members = {
  "detection_interfaces__action",  // message namespace
  "FindYellow_Feedback",  // message name
  1,  // number of fields
  sizeof(detection_interfaces__action__FindYellow_Feedback),
  detection_interfaces__action__FindYellow_Feedback__rosidl_typesupport_introspection_c__FindYellow_Feedback_message_member_array,  // message members
  detection_interfaces__action__FindYellow_Feedback__rosidl_typesupport_introspection_c__FindYellow_Feedback_init_function,  // function to initialize message memory (memory has to be allocated)
  detection_interfaces__action__FindYellow_Feedback__rosidl_typesupport_introspection_c__FindYellow_Feedback_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t detection_interfaces__action__FindYellow_Feedback__rosidl_typesupport_introspection_c__FindYellow_Feedback_message_type_support_handle = {
  0,
  &detection_interfaces__action__FindYellow_Feedback__rosidl_typesupport_introspection_c__FindYellow_Feedback_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_detection_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, detection_interfaces, action, FindYellow_Feedback)() {
  if (!detection_interfaces__action__FindYellow_Feedback__rosidl_typesupport_introspection_c__FindYellow_Feedback_message_type_support_handle.typesupport_identifier) {
    detection_interfaces__action__FindYellow_Feedback__rosidl_typesupport_introspection_c__FindYellow_Feedback_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &detection_interfaces__action__FindYellow_Feedback__rosidl_typesupport_introspection_c__FindYellow_Feedback_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "detection_interfaces/action/detail/find_yellow__rosidl_typesupport_introspection_c.h"
// already included above
// #include "detection_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "detection_interfaces/action/detail/find_yellow__functions.h"
// already included above
// #include "detection_interfaces/action/detail/find_yellow__struct.h"


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `goal`
#include "detection_interfaces/action/find_yellow.h"
// Member `goal`
// already included above
// #include "detection_interfaces/action/detail/find_yellow__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void detection_interfaces__action__FindYellow_SendGoal_Request__rosidl_typesupport_introspection_c__FindYellow_SendGoal_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  detection_interfaces__action__FindYellow_SendGoal_Request__init(message_memory);
}

void detection_interfaces__action__FindYellow_SendGoal_Request__rosidl_typesupport_introspection_c__FindYellow_SendGoal_Request_fini_function(void * message_memory)
{
  detection_interfaces__action__FindYellow_SendGoal_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember detection_interfaces__action__FindYellow_SendGoal_Request__rosidl_typesupport_introspection_c__FindYellow_SendGoal_Request_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(detection_interfaces__action__FindYellow_SendGoal_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "goal",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(detection_interfaces__action__FindYellow_SendGoal_Request, goal),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers detection_interfaces__action__FindYellow_SendGoal_Request__rosidl_typesupport_introspection_c__FindYellow_SendGoal_Request_message_members = {
  "detection_interfaces__action",  // message namespace
  "FindYellow_SendGoal_Request",  // message name
  2,  // number of fields
  sizeof(detection_interfaces__action__FindYellow_SendGoal_Request),
  detection_interfaces__action__FindYellow_SendGoal_Request__rosidl_typesupport_introspection_c__FindYellow_SendGoal_Request_message_member_array,  // message members
  detection_interfaces__action__FindYellow_SendGoal_Request__rosidl_typesupport_introspection_c__FindYellow_SendGoal_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  detection_interfaces__action__FindYellow_SendGoal_Request__rosidl_typesupport_introspection_c__FindYellow_SendGoal_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t detection_interfaces__action__FindYellow_SendGoal_Request__rosidl_typesupport_introspection_c__FindYellow_SendGoal_Request_message_type_support_handle = {
  0,
  &detection_interfaces__action__FindYellow_SendGoal_Request__rosidl_typesupport_introspection_c__FindYellow_SendGoal_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_detection_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, detection_interfaces, action, FindYellow_SendGoal_Request)() {
  detection_interfaces__action__FindYellow_SendGoal_Request__rosidl_typesupport_introspection_c__FindYellow_SendGoal_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  detection_interfaces__action__FindYellow_SendGoal_Request__rosidl_typesupport_introspection_c__FindYellow_SendGoal_Request_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, detection_interfaces, action, FindYellow_Goal)();
  if (!detection_interfaces__action__FindYellow_SendGoal_Request__rosidl_typesupport_introspection_c__FindYellow_SendGoal_Request_message_type_support_handle.typesupport_identifier) {
    detection_interfaces__action__FindYellow_SendGoal_Request__rosidl_typesupport_introspection_c__FindYellow_SendGoal_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &detection_interfaces__action__FindYellow_SendGoal_Request__rosidl_typesupport_introspection_c__FindYellow_SendGoal_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "detection_interfaces/action/detail/find_yellow__rosidl_typesupport_introspection_c.h"
// already included above
// #include "detection_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "detection_interfaces/action/detail/find_yellow__functions.h"
// already included above
// #include "detection_interfaces/action/detail/find_yellow__struct.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/time.h"
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void detection_interfaces__action__FindYellow_SendGoal_Response__rosidl_typesupport_introspection_c__FindYellow_SendGoal_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  detection_interfaces__action__FindYellow_SendGoal_Response__init(message_memory);
}

void detection_interfaces__action__FindYellow_SendGoal_Response__rosidl_typesupport_introspection_c__FindYellow_SendGoal_Response_fini_function(void * message_memory)
{
  detection_interfaces__action__FindYellow_SendGoal_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember detection_interfaces__action__FindYellow_SendGoal_Response__rosidl_typesupport_introspection_c__FindYellow_SendGoal_Response_message_member_array[2] = {
  {
    "accepted",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(detection_interfaces__action__FindYellow_SendGoal_Response, accepted),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "stamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(detection_interfaces__action__FindYellow_SendGoal_Response, stamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers detection_interfaces__action__FindYellow_SendGoal_Response__rosidl_typesupport_introspection_c__FindYellow_SendGoal_Response_message_members = {
  "detection_interfaces__action",  // message namespace
  "FindYellow_SendGoal_Response",  // message name
  2,  // number of fields
  sizeof(detection_interfaces__action__FindYellow_SendGoal_Response),
  detection_interfaces__action__FindYellow_SendGoal_Response__rosidl_typesupport_introspection_c__FindYellow_SendGoal_Response_message_member_array,  // message members
  detection_interfaces__action__FindYellow_SendGoal_Response__rosidl_typesupport_introspection_c__FindYellow_SendGoal_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  detection_interfaces__action__FindYellow_SendGoal_Response__rosidl_typesupport_introspection_c__FindYellow_SendGoal_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t detection_interfaces__action__FindYellow_SendGoal_Response__rosidl_typesupport_introspection_c__FindYellow_SendGoal_Response_message_type_support_handle = {
  0,
  &detection_interfaces__action__FindYellow_SendGoal_Response__rosidl_typesupport_introspection_c__FindYellow_SendGoal_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_detection_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, detection_interfaces, action, FindYellow_SendGoal_Response)() {
  detection_interfaces__action__FindYellow_SendGoal_Response__rosidl_typesupport_introspection_c__FindYellow_SendGoal_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!detection_interfaces__action__FindYellow_SendGoal_Response__rosidl_typesupport_introspection_c__FindYellow_SendGoal_Response_message_type_support_handle.typesupport_identifier) {
    detection_interfaces__action__FindYellow_SendGoal_Response__rosidl_typesupport_introspection_c__FindYellow_SendGoal_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &detection_interfaces__action__FindYellow_SendGoal_Response__rosidl_typesupport_introspection_c__FindYellow_SendGoal_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "detection_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "detection_interfaces/action/detail/find_yellow__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers detection_interfaces__action__detail__find_yellow__rosidl_typesupport_introspection_c__FindYellow_SendGoal_service_members = {
  "detection_interfaces__action",  // service namespace
  "FindYellow_SendGoal",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // detection_interfaces__action__detail__find_yellow__rosidl_typesupport_introspection_c__FindYellow_SendGoal_Request_message_type_support_handle,
  NULL  // response message
  // detection_interfaces__action__detail__find_yellow__rosidl_typesupport_introspection_c__FindYellow_SendGoal_Response_message_type_support_handle
};

static rosidl_service_type_support_t detection_interfaces__action__detail__find_yellow__rosidl_typesupport_introspection_c__FindYellow_SendGoal_service_type_support_handle = {
  0,
  &detection_interfaces__action__detail__find_yellow__rosidl_typesupport_introspection_c__FindYellow_SendGoal_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, detection_interfaces, action, FindYellow_SendGoal_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, detection_interfaces, action, FindYellow_SendGoal_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_detection_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, detection_interfaces, action, FindYellow_SendGoal)() {
  if (!detection_interfaces__action__detail__find_yellow__rosidl_typesupport_introspection_c__FindYellow_SendGoal_service_type_support_handle.typesupport_identifier) {
    detection_interfaces__action__detail__find_yellow__rosidl_typesupport_introspection_c__FindYellow_SendGoal_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)detection_interfaces__action__detail__find_yellow__rosidl_typesupport_introspection_c__FindYellow_SendGoal_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, detection_interfaces, action, FindYellow_SendGoal_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, detection_interfaces, action, FindYellow_SendGoal_Response)()->data;
  }

  return &detection_interfaces__action__detail__find_yellow__rosidl_typesupport_introspection_c__FindYellow_SendGoal_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "detection_interfaces/action/detail/find_yellow__rosidl_typesupport_introspection_c.h"
// already included above
// #include "detection_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "detection_interfaces/action/detail/find_yellow__functions.h"
// already included above
// #include "detection_interfaces/action/detail/find_yellow__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void detection_interfaces__action__FindYellow_GetResult_Request__rosidl_typesupport_introspection_c__FindYellow_GetResult_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  detection_interfaces__action__FindYellow_GetResult_Request__init(message_memory);
}

void detection_interfaces__action__FindYellow_GetResult_Request__rosidl_typesupport_introspection_c__FindYellow_GetResult_Request_fini_function(void * message_memory)
{
  detection_interfaces__action__FindYellow_GetResult_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember detection_interfaces__action__FindYellow_GetResult_Request__rosidl_typesupport_introspection_c__FindYellow_GetResult_Request_message_member_array[1] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(detection_interfaces__action__FindYellow_GetResult_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers detection_interfaces__action__FindYellow_GetResult_Request__rosidl_typesupport_introspection_c__FindYellow_GetResult_Request_message_members = {
  "detection_interfaces__action",  // message namespace
  "FindYellow_GetResult_Request",  // message name
  1,  // number of fields
  sizeof(detection_interfaces__action__FindYellow_GetResult_Request),
  detection_interfaces__action__FindYellow_GetResult_Request__rosidl_typesupport_introspection_c__FindYellow_GetResult_Request_message_member_array,  // message members
  detection_interfaces__action__FindYellow_GetResult_Request__rosidl_typesupport_introspection_c__FindYellow_GetResult_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  detection_interfaces__action__FindYellow_GetResult_Request__rosidl_typesupport_introspection_c__FindYellow_GetResult_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t detection_interfaces__action__FindYellow_GetResult_Request__rosidl_typesupport_introspection_c__FindYellow_GetResult_Request_message_type_support_handle = {
  0,
  &detection_interfaces__action__FindYellow_GetResult_Request__rosidl_typesupport_introspection_c__FindYellow_GetResult_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_detection_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, detection_interfaces, action, FindYellow_GetResult_Request)() {
  detection_interfaces__action__FindYellow_GetResult_Request__rosidl_typesupport_introspection_c__FindYellow_GetResult_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  if (!detection_interfaces__action__FindYellow_GetResult_Request__rosidl_typesupport_introspection_c__FindYellow_GetResult_Request_message_type_support_handle.typesupport_identifier) {
    detection_interfaces__action__FindYellow_GetResult_Request__rosidl_typesupport_introspection_c__FindYellow_GetResult_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &detection_interfaces__action__FindYellow_GetResult_Request__rosidl_typesupport_introspection_c__FindYellow_GetResult_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "detection_interfaces/action/detail/find_yellow__rosidl_typesupport_introspection_c.h"
// already included above
// #include "detection_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "detection_interfaces/action/detail/find_yellow__functions.h"
// already included above
// #include "detection_interfaces/action/detail/find_yellow__struct.h"


// Include directives for member types
// Member `result`
// already included above
// #include "detection_interfaces/action/find_yellow.h"
// Member `result`
// already included above
// #include "detection_interfaces/action/detail/find_yellow__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void detection_interfaces__action__FindYellow_GetResult_Response__rosidl_typesupport_introspection_c__FindYellow_GetResult_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  detection_interfaces__action__FindYellow_GetResult_Response__init(message_memory);
}

void detection_interfaces__action__FindYellow_GetResult_Response__rosidl_typesupport_introspection_c__FindYellow_GetResult_Response_fini_function(void * message_memory)
{
  detection_interfaces__action__FindYellow_GetResult_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember detection_interfaces__action__FindYellow_GetResult_Response__rosidl_typesupport_introspection_c__FindYellow_GetResult_Response_message_member_array[2] = {
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(detection_interfaces__action__FindYellow_GetResult_Response, status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "result",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(detection_interfaces__action__FindYellow_GetResult_Response, result),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers detection_interfaces__action__FindYellow_GetResult_Response__rosidl_typesupport_introspection_c__FindYellow_GetResult_Response_message_members = {
  "detection_interfaces__action",  // message namespace
  "FindYellow_GetResult_Response",  // message name
  2,  // number of fields
  sizeof(detection_interfaces__action__FindYellow_GetResult_Response),
  detection_interfaces__action__FindYellow_GetResult_Response__rosidl_typesupport_introspection_c__FindYellow_GetResult_Response_message_member_array,  // message members
  detection_interfaces__action__FindYellow_GetResult_Response__rosidl_typesupport_introspection_c__FindYellow_GetResult_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  detection_interfaces__action__FindYellow_GetResult_Response__rosidl_typesupport_introspection_c__FindYellow_GetResult_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t detection_interfaces__action__FindYellow_GetResult_Response__rosidl_typesupport_introspection_c__FindYellow_GetResult_Response_message_type_support_handle = {
  0,
  &detection_interfaces__action__FindYellow_GetResult_Response__rosidl_typesupport_introspection_c__FindYellow_GetResult_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_detection_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, detection_interfaces, action, FindYellow_GetResult_Response)() {
  detection_interfaces__action__FindYellow_GetResult_Response__rosidl_typesupport_introspection_c__FindYellow_GetResult_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, detection_interfaces, action, FindYellow_Result)();
  if (!detection_interfaces__action__FindYellow_GetResult_Response__rosidl_typesupport_introspection_c__FindYellow_GetResult_Response_message_type_support_handle.typesupport_identifier) {
    detection_interfaces__action__FindYellow_GetResult_Response__rosidl_typesupport_introspection_c__FindYellow_GetResult_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &detection_interfaces__action__FindYellow_GetResult_Response__rosidl_typesupport_introspection_c__FindYellow_GetResult_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "detection_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "detection_interfaces/action/detail/find_yellow__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers detection_interfaces__action__detail__find_yellow__rosidl_typesupport_introspection_c__FindYellow_GetResult_service_members = {
  "detection_interfaces__action",  // service namespace
  "FindYellow_GetResult",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // detection_interfaces__action__detail__find_yellow__rosidl_typesupport_introspection_c__FindYellow_GetResult_Request_message_type_support_handle,
  NULL  // response message
  // detection_interfaces__action__detail__find_yellow__rosidl_typesupport_introspection_c__FindYellow_GetResult_Response_message_type_support_handle
};

static rosidl_service_type_support_t detection_interfaces__action__detail__find_yellow__rosidl_typesupport_introspection_c__FindYellow_GetResult_service_type_support_handle = {
  0,
  &detection_interfaces__action__detail__find_yellow__rosidl_typesupport_introspection_c__FindYellow_GetResult_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, detection_interfaces, action, FindYellow_GetResult_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, detection_interfaces, action, FindYellow_GetResult_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_detection_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, detection_interfaces, action, FindYellow_GetResult)() {
  if (!detection_interfaces__action__detail__find_yellow__rosidl_typesupport_introspection_c__FindYellow_GetResult_service_type_support_handle.typesupport_identifier) {
    detection_interfaces__action__detail__find_yellow__rosidl_typesupport_introspection_c__FindYellow_GetResult_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)detection_interfaces__action__detail__find_yellow__rosidl_typesupport_introspection_c__FindYellow_GetResult_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, detection_interfaces, action, FindYellow_GetResult_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, detection_interfaces, action, FindYellow_GetResult_Response)()->data;
  }

  return &detection_interfaces__action__detail__find_yellow__rosidl_typesupport_introspection_c__FindYellow_GetResult_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "detection_interfaces/action/detail/find_yellow__rosidl_typesupport_introspection_c.h"
// already included above
// #include "detection_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "detection_interfaces/action/detail/find_yellow__functions.h"
// already included above
// #include "detection_interfaces/action/detail/find_yellow__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `feedback`
// already included above
// #include "detection_interfaces/action/find_yellow.h"
// Member `feedback`
// already included above
// #include "detection_interfaces/action/detail/find_yellow__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void detection_interfaces__action__FindYellow_FeedbackMessage__rosidl_typesupport_introspection_c__FindYellow_FeedbackMessage_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  detection_interfaces__action__FindYellow_FeedbackMessage__init(message_memory);
}

void detection_interfaces__action__FindYellow_FeedbackMessage__rosidl_typesupport_introspection_c__FindYellow_FeedbackMessage_fini_function(void * message_memory)
{
  detection_interfaces__action__FindYellow_FeedbackMessage__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember detection_interfaces__action__FindYellow_FeedbackMessage__rosidl_typesupport_introspection_c__FindYellow_FeedbackMessage_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(detection_interfaces__action__FindYellow_FeedbackMessage, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "feedback",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(detection_interfaces__action__FindYellow_FeedbackMessage, feedback),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers detection_interfaces__action__FindYellow_FeedbackMessage__rosidl_typesupport_introspection_c__FindYellow_FeedbackMessage_message_members = {
  "detection_interfaces__action",  // message namespace
  "FindYellow_FeedbackMessage",  // message name
  2,  // number of fields
  sizeof(detection_interfaces__action__FindYellow_FeedbackMessage),
  detection_interfaces__action__FindYellow_FeedbackMessage__rosidl_typesupport_introspection_c__FindYellow_FeedbackMessage_message_member_array,  // message members
  detection_interfaces__action__FindYellow_FeedbackMessage__rosidl_typesupport_introspection_c__FindYellow_FeedbackMessage_init_function,  // function to initialize message memory (memory has to be allocated)
  detection_interfaces__action__FindYellow_FeedbackMessage__rosidl_typesupport_introspection_c__FindYellow_FeedbackMessage_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t detection_interfaces__action__FindYellow_FeedbackMessage__rosidl_typesupport_introspection_c__FindYellow_FeedbackMessage_message_type_support_handle = {
  0,
  &detection_interfaces__action__FindYellow_FeedbackMessage__rosidl_typesupport_introspection_c__FindYellow_FeedbackMessage_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_detection_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, detection_interfaces, action, FindYellow_FeedbackMessage)() {
  detection_interfaces__action__FindYellow_FeedbackMessage__rosidl_typesupport_introspection_c__FindYellow_FeedbackMessage_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  detection_interfaces__action__FindYellow_FeedbackMessage__rosidl_typesupport_introspection_c__FindYellow_FeedbackMessage_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, detection_interfaces, action, FindYellow_Feedback)();
  if (!detection_interfaces__action__FindYellow_FeedbackMessage__rosidl_typesupport_introspection_c__FindYellow_FeedbackMessage_message_type_support_handle.typesupport_identifier) {
    detection_interfaces__action__FindYellow_FeedbackMessage__rosidl_typesupport_introspection_c__FindYellow_FeedbackMessage_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &detection_interfaces__action__FindYellow_FeedbackMessage__rosidl_typesupport_introspection_c__FindYellow_FeedbackMessage_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
