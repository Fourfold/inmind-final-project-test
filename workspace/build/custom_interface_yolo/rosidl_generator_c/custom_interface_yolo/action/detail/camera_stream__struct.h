// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_interface_yolo:action/CameraStream.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACE_YOLO__ACTION__DETAIL__CAMERA_STREAM__STRUCT_H_
#define CUSTOM_INTERFACE_YOLO__ACTION__DETAIL__CAMERA_STREAM__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in action/CameraStream in the package custom_interface_yolo.
typedef struct custom_interface_yolo__action__CameraStream_Goal
{
  uint8_t structure_needs_at_least_one_member;
} custom_interface_yolo__action__CameraStream_Goal;

// Struct for a sequence of custom_interface_yolo__action__CameraStream_Goal.
typedef struct custom_interface_yolo__action__CameraStream_Goal__Sequence
{
  custom_interface_yolo__action__CameraStream_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interface_yolo__action__CameraStream_Goal__Sequence;


// Constants defined in the message

/// Struct defined in action/CameraStream in the package custom_interface_yolo.
typedef struct custom_interface_yolo__action__CameraStream_Result
{
  /// Result definition
  bool success;
  float x;
  float y;
} custom_interface_yolo__action__CameraStream_Result;

// Struct for a sequence of custom_interface_yolo__action__CameraStream_Result.
typedef struct custom_interface_yolo__action__CameraStream_Result__Sequence
{
  custom_interface_yolo__action__CameraStream_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interface_yolo__action__CameraStream_Result__Sequence;


// Constants defined in the message

/// Struct defined in action/CameraStream in the package custom_interface_yolo.
typedef struct custom_interface_yolo__action__CameraStream_Feedback
{
  uint8_t structure_needs_at_least_one_member;
} custom_interface_yolo__action__CameraStream_Feedback;

// Struct for a sequence of custom_interface_yolo__action__CameraStream_Feedback.
typedef struct custom_interface_yolo__action__CameraStream_Feedback__Sequence
{
  custom_interface_yolo__action__CameraStream_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interface_yolo__action__CameraStream_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "custom_interface_yolo/action/detail/camera_stream__struct.h"

/// Struct defined in action/CameraStream in the package custom_interface_yolo.
typedef struct custom_interface_yolo__action__CameraStream_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  custom_interface_yolo__action__CameraStream_Goal goal;
} custom_interface_yolo__action__CameraStream_SendGoal_Request;

// Struct for a sequence of custom_interface_yolo__action__CameraStream_SendGoal_Request.
typedef struct custom_interface_yolo__action__CameraStream_SendGoal_Request__Sequence
{
  custom_interface_yolo__action__CameraStream_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interface_yolo__action__CameraStream_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/CameraStream in the package custom_interface_yolo.
typedef struct custom_interface_yolo__action__CameraStream_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} custom_interface_yolo__action__CameraStream_SendGoal_Response;

// Struct for a sequence of custom_interface_yolo__action__CameraStream_SendGoal_Response.
typedef struct custom_interface_yolo__action__CameraStream_SendGoal_Response__Sequence
{
  custom_interface_yolo__action__CameraStream_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interface_yolo__action__CameraStream_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/CameraStream in the package custom_interface_yolo.
typedef struct custom_interface_yolo__action__CameraStream_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} custom_interface_yolo__action__CameraStream_GetResult_Request;

// Struct for a sequence of custom_interface_yolo__action__CameraStream_GetResult_Request.
typedef struct custom_interface_yolo__action__CameraStream_GetResult_Request__Sequence
{
  custom_interface_yolo__action__CameraStream_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interface_yolo__action__CameraStream_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "custom_interface_yolo/action/detail/camera_stream__struct.h"

/// Struct defined in action/CameraStream in the package custom_interface_yolo.
typedef struct custom_interface_yolo__action__CameraStream_GetResult_Response
{
  int8_t status;
  custom_interface_yolo__action__CameraStream_Result result;
} custom_interface_yolo__action__CameraStream_GetResult_Response;

// Struct for a sequence of custom_interface_yolo__action__CameraStream_GetResult_Response.
typedef struct custom_interface_yolo__action__CameraStream_GetResult_Response__Sequence
{
  custom_interface_yolo__action__CameraStream_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interface_yolo__action__CameraStream_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "custom_interface_yolo/action/detail/camera_stream__struct.h"

/// Struct defined in action/CameraStream in the package custom_interface_yolo.
typedef struct custom_interface_yolo__action__CameraStream_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  custom_interface_yolo__action__CameraStream_Feedback feedback;
} custom_interface_yolo__action__CameraStream_FeedbackMessage;

// Struct for a sequence of custom_interface_yolo__action__CameraStream_FeedbackMessage.
typedef struct custom_interface_yolo__action__CameraStream_FeedbackMessage__Sequence
{
  custom_interface_yolo__action__CameraStream_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interface_yolo__action__CameraStream_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_INTERFACE_YOLO__ACTION__DETAIL__CAMERA_STREAM__STRUCT_H_
