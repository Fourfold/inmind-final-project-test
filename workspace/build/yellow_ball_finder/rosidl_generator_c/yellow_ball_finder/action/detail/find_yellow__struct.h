// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from yellow_ball_finder:action/FindYellow.idl
// generated code does not contain a copyright notice

#ifndef YELLOW_BALL_FINDER__ACTION__DETAIL__FIND_YELLOW__STRUCT_H_
#define YELLOW_BALL_FINDER__ACTION__DETAIL__FIND_YELLOW__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in action/FindYellow in the package yellow_ball_finder.
typedef struct yellow_ball_finder__action__FindYellow_Goal
{
  uint8_t structure_needs_at_least_one_member;
} yellow_ball_finder__action__FindYellow_Goal;

// Struct for a sequence of yellow_ball_finder__action__FindYellow_Goal.
typedef struct yellow_ball_finder__action__FindYellow_Goal__Sequence
{
  yellow_ball_finder__action__FindYellow_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} yellow_ball_finder__action__FindYellow_Goal__Sequence;


// Constants defined in the message

/// Struct defined in action/FindYellow in the package yellow_ball_finder.
typedef struct yellow_ball_finder__action__FindYellow_Result
{
  bool found;
  int32_t cx;
  int32_t cy;
} yellow_ball_finder__action__FindYellow_Result;

// Struct for a sequence of yellow_ball_finder__action__FindYellow_Result.
typedef struct yellow_ball_finder__action__FindYellow_Result__Sequence
{
  yellow_ball_finder__action__FindYellow_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} yellow_ball_finder__action__FindYellow_Result__Sequence;


// Constants defined in the message

/// Struct defined in action/FindYellow in the package yellow_ball_finder.
typedef struct yellow_ball_finder__action__FindYellow_Feedback
{
  uint8_t structure_needs_at_least_one_member;
} yellow_ball_finder__action__FindYellow_Feedback;

// Struct for a sequence of yellow_ball_finder__action__FindYellow_Feedback.
typedef struct yellow_ball_finder__action__FindYellow_Feedback__Sequence
{
  yellow_ball_finder__action__FindYellow_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} yellow_ball_finder__action__FindYellow_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "yellow_ball_finder/action/detail/find_yellow__struct.h"

/// Struct defined in action/FindYellow in the package yellow_ball_finder.
typedef struct yellow_ball_finder__action__FindYellow_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  yellow_ball_finder__action__FindYellow_Goal goal;
} yellow_ball_finder__action__FindYellow_SendGoal_Request;

// Struct for a sequence of yellow_ball_finder__action__FindYellow_SendGoal_Request.
typedef struct yellow_ball_finder__action__FindYellow_SendGoal_Request__Sequence
{
  yellow_ball_finder__action__FindYellow_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} yellow_ball_finder__action__FindYellow_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/FindYellow in the package yellow_ball_finder.
typedef struct yellow_ball_finder__action__FindYellow_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} yellow_ball_finder__action__FindYellow_SendGoal_Response;

// Struct for a sequence of yellow_ball_finder__action__FindYellow_SendGoal_Response.
typedef struct yellow_ball_finder__action__FindYellow_SendGoal_Response__Sequence
{
  yellow_ball_finder__action__FindYellow_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} yellow_ball_finder__action__FindYellow_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/FindYellow in the package yellow_ball_finder.
typedef struct yellow_ball_finder__action__FindYellow_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} yellow_ball_finder__action__FindYellow_GetResult_Request;

// Struct for a sequence of yellow_ball_finder__action__FindYellow_GetResult_Request.
typedef struct yellow_ball_finder__action__FindYellow_GetResult_Request__Sequence
{
  yellow_ball_finder__action__FindYellow_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} yellow_ball_finder__action__FindYellow_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "yellow_ball_finder/action/detail/find_yellow__struct.h"

/// Struct defined in action/FindYellow in the package yellow_ball_finder.
typedef struct yellow_ball_finder__action__FindYellow_GetResult_Response
{
  int8_t status;
  yellow_ball_finder__action__FindYellow_Result result;
} yellow_ball_finder__action__FindYellow_GetResult_Response;

// Struct for a sequence of yellow_ball_finder__action__FindYellow_GetResult_Response.
typedef struct yellow_ball_finder__action__FindYellow_GetResult_Response__Sequence
{
  yellow_ball_finder__action__FindYellow_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} yellow_ball_finder__action__FindYellow_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "yellow_ball_finder/action/detail/find_yellow__struct.h"

/// Struct defined in action/FindYellow in the package yellow_ball_finder.
typedef struct yellow_ball_finder__action__FindYellow_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  yellow_ball_finder__action__FindYellow_Feedback feedback;
} yellow_ball_finder__action__FindYellow_FeedbackMessage;

// Struct for a sequence of yellow_ball_finder__action__FindYellow_FeedbackMessage.
typedef struct yellow_ball_finder__action__FindYellow_FeedbackMessage__Sequence
{
  yellow_ball_finder__action__FindYellow_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} yellow_ball_finder__action__FindYellow_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // YELLOW_BALL_FINDER__ACTION__DETAIL__FIND_YELLOW__STRUCT_H_
