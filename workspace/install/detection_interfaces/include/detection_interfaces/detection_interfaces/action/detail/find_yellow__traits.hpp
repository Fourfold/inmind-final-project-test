// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from detection_interfaces:action/FindYellow.idl
// generated code does not contain a copyright notice

#ifndef DETECTION_INTERFACES__ACTION__DETAIL__FIND_YELLOW__TRAITS_HPP_
#define DETECTION_INTERFACES__ACTION__DETAIL__FIND_YELLOW__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "detection_interfaces/action/detail/find_yellow__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace detection_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const FindYellow_Goal & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FindYellow_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FindYellow_Goal & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace detection_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use detection_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const detection_interfaces::action::FindYellow_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  detection_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use detection_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const detection_interfaces::action::FindYellow_Goal & msg)
{
  return detection_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<detection_interfaces::action::FindYellow_Goal>()
{
  return "detection_interfaces::action::FindYellow_Goal";
}

template<>
inline const char * name<detection_interfaces::action::FindYellow_Goal>()
{
  return "detection_interfaces/action/FindYellow_Goal";
}

template<>
struct has_fixed_size<detection_interfaces::action::FindYellow_Goal>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<detection_interfaces::action::FindYellow_Goal>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<detection_interfaces::action::FindYellow_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace detection_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const FindYellow_Result & msg,
  std::ostream & out)
{
  out << "{";
  // member: found
  {
    out << "found: ";
    rosidl_generator_traits::value_to_yaml(msg.found, out);
    out << ", ";
  }

  // member: frame_width
  {
    out << "frame_width: ";
    rosidl_generator_traits::value_to_yaml(msg.frame_width, out);
    out << ", ";
  }

  // member: cx
  {
    out << "cx: ";
    rosidl_generator_traits::value_to_yaml(msg.cx, out);
    out << ", ";
  }

  // member: cy
  {
    out << "cy: ";
    rosidl_generator_traits::value_to_yaml(msg.cy, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FindYellow_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: found
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "found: ";
    rosidl_generator_traits::value_to_yaml(msg.found, out);
    out << "\n";
  }

  // member: frame_width
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "frame_width: ";
    rosidl_generator_traits::value_to_yaml(msg.frame_width, out);
    out << "\n";
  }

  // member: cx
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cx: ";
    rosidl_generator_traits::value_to_yaml(msg.cx, out);
    out << "\n";
  }

  // member: cy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cy: ";
    rosidl_generator_traits::value_to_yaml(msg.cy, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FindYellow_Result & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace detection_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use detection_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const detection_interfaces::action::FindYellow_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  detection_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use detection_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const detection_interfaces::action::FindYellow_Result & msg)
{
  return detection_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<detection_interfaces::action::FindYellow_Result>()
{
  return "detection_interfaces::action::FindYellow_Result";
}

template<>
inline const char * name<detection_interfaces::action::FindYellow_Result>()
{
  return "detection_interfaces/action/FindYellow_Result";
}

template<>
struct has_fixed_size<detection_interfaces::action::FindYellow_Result>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<detection_interfaces::action::FindYellow_Result>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<detection_interfaces::action::FindYellow_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace detection_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const FindYellow_Feedback & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FindYellow_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FindYellow_Feedback & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace detection_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use detection_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const detection_interfaces::action::FindYellow_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  detection_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use detection_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const detection_interfaces::action::FindYellow_Feedback & msg)
{
  return detection_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<detection_interfaces::action::FindYellow_Feedback>()
{
  return "detection_interfaces::action::FindYellow_Feedback";
}

template<>
inline const char * name<detection_interfaces::action::FindYellow_Feedback>()
{
  return "detection_interfaces/action/FindYellow_Feedback";
}

template<>
struct has_fixed_size<detection_interfaces::action::FindYellow_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<detection_interfaces::action::FindYellow_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<detection_interfaces::action::FindYellow_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "detection_interfaces/action/detail/find_yellow__traits.hpp"

namespace detection_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const FindYellow_SendGoal_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: goal
  {
    out << "goal: ";
    to_flow_style_yaml(msg.goal, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FindYellow_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal:\n";
    to_block_style_yaml(msg.goal, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FindYellow_SendGoal_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace detection_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use detection_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const detection_interfaces::action::FindYellow_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  detection_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use detection_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const detection_interfaces::action::FindYellow_SendGoal_Request & msg)
{
  return detection_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<detection_interfaces::action::FindYellow_SendGoal_Request>()
{
  return "detection_interfaces::action::FindYellow_SendGoal_Request";
}

template<>
inline const char * name<detection_interfaces::action::FindYellow_SendGoal_Request>()
{
  return "detection_interfaces/action/FindYellow_SendGoal_Request";
}

template<>
struct has_fixed_size<detection_interfaces::action::FindYellow_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<detection_interfaces::action::FindYellow_Goal>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<detection_interfaces::action::FindYellow_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<detection_interfaces::action::FindYellow_Goal>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<detection_interfaces::action::FindYellow_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace detection_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const FindYellow_SendGoal_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accepted
  {
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << ", ";
  }

  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FindYellow_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: accepted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << "\n";
  }

  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FindYellow_SendGoal_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace detection_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use detection_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const detection_interfaces::action::FindYellow_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  detection_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use detection_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const detection_interfaces::action::FindYellow_SendGoal_Response & msg)
{
  return detection_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<detection_interfaces::action::FindYellow_SendGoal_Response>()
{
  return "detection_interfaces::action::FindYellow_SendGoal_Response";
}

template<>
inline const char * name<detection_interfaces::action::FindYellow_SendGoal_Response>()
{
  return "detection_interfaces/action/FindYellow_SendGoal_Response";
}

template<>
struct has_fixed_size<detection_interfaces::action::FindYellow_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<detection_interfaces::action::FindYellow_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<detection_interfaces::action::FindYellow_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<detection_interfaces::action::FindYellow_SendGoal>()
{
  return "detection_interfaces::action::FindYellow_SendGoal";
}

template<>
inline const char * name<detection_interfaces::action::FindYellow_SendGoal>()
{
  return "detection_interfaces/action/FindYellow_SendGoal";
}

template<>
struct has_fixed_size<detection_interfaces::action::FindYellow_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<detection_interfaces::action::FindYellow_SendGoal_Request>::value &&
    has_fixed_size<detection_interfaces::action::FindYellow_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<detection_interfaces::action::FindYellow_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<detection_interfaces::action::FindYellow_SendGoal_Request>::value &&
    has_bounded_size<detection_interfaces::action::FindYellow_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<detection_interfaces::action::FindYellow_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<detection_interfaces::action::FindYellow_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<detection_interfaces::action::FindYellow_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace detection_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const FindYellow_GetResult_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FindYellow_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FindYellow_GetResult_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace detection_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use detection_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const detection_interfaces::action::FindYellow_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  detection_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use detection_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const detection_interfaces::action::FindYellow_GetResult_Request & msg)
{
  return detection_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<detection_interfaces::action::FindYellow_GetResult_Request>()
{
  return "detection_interfaces::action::FindYellow_GetResult_Request";
}

template<>
inline const char * name<detection_interfaces::action::FindYellow_GetResult_Request>()
{
  return "detection_interfaces/action/FindYellow_GetResult_Request";
}

template<>
struct has_fixed_size<detection_interfaces::action::FindYellow_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<detection_interfaces::action::FindYellow_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<detection_interfaces::action::FindYellow_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "detection_interfaces/action/detail/find_yellow__traits.hpp"

namespace detection_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const FindYellow_GetResult_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: result
  {
    out << "result: ";
    to_flow_style_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FindYellow_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result:\n";
    to_block_style_yaml(msg.result, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FindYellow_GetResult_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace detection_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use detection_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const detection_interfaces::action::FindYellow_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  detection_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use detection_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const detection_interfaces::action::FindYellow_GetResult_Response & msg)
{
  return detection_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<detection_interfaces::action::FindYellow_GetResult_Response>()
{
  return "detection_interfaces::action::FindYellow_GetResult_Response";
}

template<>
inline const char * name<detection_interfaces::action::FindYellow_GetResult_Response>()
{
  return "detection_interfaces/action/FindYellow_GetResult_Response";
}

template<>
struct has_fixed_size<detection_interfaces::action::FindYellow_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<detection_interfaces::action::FindYellow_Result>::value> {};

template<>
struct has_bounded_size<detection_interfaces::action::FindYellow_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<detection_interfaces::action::FindYellow_Result>::value> {};

template<>
struct is_message<detection_interfaces::action::FindYellow_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<detection_interfaces::action::FindYellow_GetResult>()
{
  return "detection_interfaces::action::FindYellow_GetResult";
}

template<>
inline const char * name<detection_interfaces::action::FindYellow_GetResult>()
{
  return "detection_interfaces/action/FindYellow_GetResult";
}

template<>
struct has_fixed_size<detection_interfaces::action::FindYellow_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<detection_interfaces::action::FindYellow_GetResult_Request>::value &&
    has_fixed_size<detection_interfaces::action::FindYellow_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<detection_interfaces::action::FindYellow_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<detection_interfaces::action::FindYellow_GetResult_Request>::value &&
    has_bounded_size<detection_interfaces::action::FindYellow_GetResult_Response>::value
  >
{
};

template<>
struct is_service<detection_interfaces::action::FindYellow_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<detection_interfaces::action::FindYellow_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<detection_interfaces::action::FindYellow_GetResult_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'feedback'
// already included above
// #include "detection_interfaces/action/detail/find_yellow__traits.hpp"

namespace detection_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const FindYellow_FeedbackMessage & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: feedback
  {
    out << "feedback: ";
    to_flow_style_yaml(msg.feedback, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FindYellow_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: feedback
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "feedback:\n";
    to_block_style_yaml(msg.feedback, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FindYellow_FeedbackMessage & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace detection_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use detection_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const detection_interfaces::action::FindYellow_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  detection_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use detection_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const detection_interfaces::action::FindYellow_FeedbackMessage & msg)
{
  return detection_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<detection_interfaces::action::FindYellow_FeedbackMessage>()
{
  return "detection_interfaces::action::FindYellow_FeedbackMessage";
}

template<>
inline const char * name<detection_interfaces::action::FindYellow_FeedbackMessage>()
{
  return "detection_interfaces/action/FindYellow_FeedbackMessage";
}

template<>
struct has_fixed_size<detection_interfaces::action::FindYellow_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<detection_interfaces::action::FindYellow_Feedback>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<detection_interfaces::action::FindYellow_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<detection_interfaces::action::FindYellow_Feedback>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<detection_interfaces::action::FindYellow_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<detection_interfaces::action::FindYellow>
  : std::true_type
{
};

template<>
struct is_action_goal<detection_interfaces::action::FindYellow_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<detection_interfaces::action::FindYellow_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<detection_interfaces::action::FindYellow_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // DETECTION_INTERFACES__ACTION__DETAIL__FIND_YELLOW__TRAITS_HPP_
