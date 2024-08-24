// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from detection_interfaces:action/FindObject.idl
// generated code does not contain a copyright notice

#ifndef DETECTION_INTERFACES__ACTION__DETAIL__FIND_OBJECT__STRUCT_H_
#define DETECTION_INTERFACES__ACTION__DETAIL__FIND_OBJECT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'object_type'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/FindObject in the package detection_interfaces.
typedef struct detection_interfaces__action__FindObject_Goal
{
  rosidl_runtime_c__String object_type;
} detection_interfaces__action__FindObject_Goal;

// Struct for a sequence of detection_interfaces__action__FindObject_Goal.
typedef struct detection_interfaces__action__FindObject_Goal__Sequence
{
  detection_interfaces__action__FindObject_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} detection_interfaces__action__FindObject_Goal__Sequence;


// Constants defined in the message

/// Struct defined in action/FindObject in the package detection_interfaces.
typedef struct detection_interfaces__action__FindObject_Result
{
  bool found;
  int32_t frame_width;
  int32_t cx;
  int32_t cy;
} detection_interfaces__action__FindObject_Result;

// Struct for a sequence of detection_interfaces__action__FindObject_Result.
typedef struct detection_interfaces__action__FindObject_Result__Sequence
{
  detection_interfaces__action__FindObject_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} detection_interfaces__action__FindObject_Result__Sequence;


// Constants defined in the message

/// Struct defined in action/FindObject in the package detection_interfaces.
typedef struct detection_interfaces__action__FindObject_Feedback
{
  uint8_t structure_needs_at_least_one_member;
} detection_interfaces__action__FindObject_Feedback;

// Struct for a sequence of detection_interfaces__action__FindObject_Feedback.
typedef struct detection_interfaces__action__FindObject_Feedback__Sequence
{
  detection_interfaces__action__FindObject_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} detection_interfaces__action__FindObject_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "detection_interfaces/action/detail/find_object__struct.h"

/// Struct defined in action/FindObject in the package detection_interfaces.
typedef struct detection_interfaces__action__FindObject_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  detection_interfaces__action__FindObject_Goal goal;
} detection_interfaces__action__FindObject_SendGoal_Request;

// Struct for a sequence of detection_interfaces__action__FindObject_SendGoal_Request.
typedef struct detection_interfaces__action__FindObject_SendGoal_Request__Sequence
{
  detection_interfaces__action__FindObject_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} detection_interfaces__action__FindObject_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/FindObject in the package detection_interfaces.
typedef struct detection_interfaces__action__FindObject_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} detection_interfaces__action__FindObject_SendGoal_Response;

// Struct for a sequence of detection_interfaces__action__FindObject_SendGoal_Response.
typedef struct detection_interfaces__action__FindObject_SendGoal_Response__Sequence
{
  detection_interfaces__action__FindObject_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} detection_interfaces__action__FindObject_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/FindObject in the package detection_interfaces.
typedef struct detection_interfaces__action__FindObject_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} detection_interfaces__action__FindObject_GetResult_Request;

// Struct for a sequence of detection_interfaces__action__FindObject_GetResult_Request.
typedef struct detection_interfaces__action__FindObject_GetResult_Request__Sequence
{
  detection_interfaces__action__FindObject_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} detection_interfaces__action__FindObject_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "detection_interfaces/action/detail/find_object__struct.h"

/// Struct defined in action/FindObject in the package detection_interfaces.
typedef struct detection_interfaces__action__FindObject_GetResult_Response
{
  int8_t status;
  detection_interfaces__action__FindObject_Result result;
} detection_interfaces__action__FindObject_GetResult_Response;

// Struct for a sequence of detection_interfaces__action__FindObject_GetResult_Response.
typedef struct detection_interfaces__action__FindObject_GetResult_Response__Sequence
{
  detection_interfaces__action__FindObject_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} detection_interfaces__action__FindObject_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "detection_interfaces/action/detail/find_object__struct.h"

/// Struct defined in action/FindObject in the package detection_interfaces.
typedef struct detection_interfaces__action__FindObject_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  detection_interfaces__action__FindObject_Feedback feedback;
} detection_interfaces__action__FindObject_FeedbackMessage;

// Struct for a sequence of detection_interfaces__action__FindObject_FeedbackMessage.
typedef struct detection_interfaces__action__FindObject_FeedbackMessage__Sequence
{
  detection_interfaces__action__FindObject_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} detection_interfaces__action__FindObject_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DETECTION_INTERFACES__ACTION__DETAIL__FIND_OBJECT__STRUCT_H_
