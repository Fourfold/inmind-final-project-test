// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_hardware_interfaces:action/GripperAction.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_HARDWARE_INTERFACES__ACTION__DETAIL__GRIPPER_ACTION__STRUCT_H_
#define ROBOT_HARDWARE_INTERFACES__ACTION__DETAIL__GRIPPER_ACTION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in action/GripperAction in the package robot_hardware_interfaces.
typedef struct robot_hardware_interfaces__action__GripperAction_Goal
{
  uint8_t structure_needs_at_least_one_member;
} robot_hardware_interfaces__action__GripperAction_Goal;

// Struct for a sequence of robot_hardware_interfaces__action__GripperAction_Goal.
typedef struct robot_hardware_interfaces__action__GripperAction_Goal__Sequence
{
  robot_hardware_interfaces__action__GripperAction_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_hardware_interfaces__action__GripperAction_Goal__Sequence;


// Constants defined in the message

/// Struct defined in action/GripperAction in the package robot_hardware_interfaces.
typedef struct robot_hardware_interfaces__action__GripperAction_Result
{
  bool result;
} robot_hardware_interfaces__action__GripperAction_Result;

// Struct for a sequence of robot_hardware_interfaces__action__GripperAction_Result.
typedef struct robot_hardware_interfaces__action__GripperAction_Result__Sequence
{
  robot_hardware_interfaces__action__GripperAction_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_hardware_interfaces__action__GripperAction_Result__Sequence;


// Constants defined in the message

/// Struct defined in action/GripperAction in the package robot_hardware_interfaces.
typedef struct robot_hardware_interfaces__action__GripperAction_Feedback
{
  int32_t feedback;
} robot_hardware_interfaces__action__GripperAction_Feedback;

// Struct for a sequence of robot_hardware_interfaces__action__GripperAction_Feedback.
typedef struct robot_hardware_interfaces__action__GripperAction_Feedback__Sequence
{
  robot_hardware_interfaces__action__GripperAction_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_hardware_interfaces__action__GripperAction_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "robot_hardware_interfaces/action/detail/gripper_action__struct.h"

/// Struct defined in action/GripperAction in the package robot_hardware_interfaces.
typedef struct robot_hardware_interfaces__action__GripperAction_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  robot_hardware_interfaces__action__GripperAction_Goal goal;
} robot_hardware_interfaces__action__GripperAction_SendGoal_Request;

// Struct for a sequence of robot_hardware_interfaces__action__GripperAction_SendGoal_Request.
typedef struct robot_hardware_interfaces__action__GripperAction_SendGoal_Request__Sequence
{
  robot_hardware_interfaces__action__GripperAction_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_hardware_interfaces__action__GripperAction_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/GripperAction in the package robot_hardware_interfaces.
typedef struct robot_hardware_interfaces__action__GripperAction_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} robot_hardware_interfaces__action__GripperAction_SendGoal_Response;

// Struct for a sequence of robot_hardware_interfaces__action__GripperAction_SendGoal_Response.
typedef struct robot_hardware_interfaces__action__GripperAction_SendGoal_Response__Sequence
{
  robot_hardware_interfaces__action__GripperAction_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_hardware_interfaces__action__GripperAction_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/GripperAction in the package robot_hardware_interfaces.
typedef struct robot_hardware_interfaces__action__GripperAction_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} robot_hardware_interfaces__action__GripperAction_GetResult_Request;

// Struct for a sequence of robot_hardware_interfaces__action__GripperAction_GetResult_Request.
typedef struct robot_hardware_interfaces__action__GripperAction_GetResult_Request__Sequence
{
  robot_hardware_interfaces__action__GripperAction_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_hardware_interfaces__action__GripperAction_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "robot_hardware_interfaces/action/detail/gripper_action__struct.h"

/// Struct defined in action/GripperAction in the package robot_hardware_interfaces.
typedef struct robot_hardware_interfaces__action__GripperAction_GetResult_Response
{
  int8_t status;
  robot_hardware_interfaces__action__GripperAction_Result result;
} robot_hardware_interfaces__action__GripperAction_GetResult_Response;

// Struct for a sequence of robot_hardware_interfaces__action__GripperAction_GetResult_Response.
typedef struct robot_hardware_interfaces__action__GripperAction_GetResult_Response__Sequence
{
  robot_hardware_interfaces__action__GripperAction_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_hardware_interfaces__action__GripperAction_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "robot_hardware_interfaces/action/detail/gripper_action__struct.h"

/// Struct defined in action/GripperAction in the package robot_hardware_interfaces.
typedef struct robot_hardware_interfaces__action__GripperAction_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  robot_hardware_interfaces__action__GripperAction_Feedback feedback;
} robot_hardware_interfaces__action__GripperAction_FeedbackMessage;

// Struct for a sequence of robot_hardware_interfaces__action__GripperAction_FeedbackMessage.
typedef struct robot_hardware_interfaces__action__GripperAction_FeedbackMessage__Sequence
{
  robot_hardware_interfaces__action__GripperAction_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_hardware_interfaces__action__GripperAction_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_HARDWARE_INTERFACES__ACTION__DETAIL__GRIPPER_ACTION__STRUCT_H_
