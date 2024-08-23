// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from yellow_ball_finder:action/FindYellow.idl
// generated code does not contain a copyright notice

#ifndef YELLOW_BALL_FINDER__ACTION__DETAIL__FIND_YELLOW__TRAITS_HPP_
#define YELLOW_BALL_FINDER__ACTION__DETAIL__FIND_YELLOW__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "yellow_ball_finder/action/detail/find_yellow__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace yellow_ball_finder
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

}  // namespace yellow_ball_finder

namespace rosidl_generator_traits
{

[[deprecated("use yellow_ball_finder::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const yellow_ball_finder::action::FindYellow_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  yellow_ball_finder::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use yellow_ball_finder::action::to_yaml() instead")]]
inline std::string to_yaml(const yellow_ball_finder::action::FindYellow_Goal & msg)
{
  return yellow_ball_finder::action::to_yaml(msg);
}

template<>
inline const char * data_type<yellow_ball_finder::action::FindYellow_Goal>()
{
  return "yellow_ball_finder::action::FindYellow_Goal";
}

template<>
inline const char * name<yellow_ball_finder::action::FindYellow_Goal>()
{
  return "yellow_ball_finder/action/FindYellow_Goal";
}

template<>
struct has_fixed_size<yellow_ball_finder::action::FindYellow_Goal>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<yellow_ball_finder::action::FindYellow_Goal>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<yellow_ball_finder::action::FindYellow_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace yellow_ball_finder
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

}  // namespace yellow_ball_finder

namespace rosidl_generator_traits
{

[[deprecated("use yellow_ball_finder::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const yellow_ball_finder::action::FindYellow_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  yellow_ball_finder::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use yellow_ball_finder::action::to_yaml() instead")]]
inline std::string to_yaml(const yellow_ball_finder::action::FindYellow_Result & msg)
{
  return yellow_ball_finder::action::to_yaml(msg);
}

template<>
inline const char * data_type<yellow_ball_finder::action::FindYellow_Result>()
{
  return "yellow_ball_finder::action::FindYellow_Result";
}

template<>
inline const char * name<yellow_ball_finder::action::FindYellow_Result>()
{
  return "yellow_ball_finder/action/FindYellow_Result";
}

template<>
struct has_fixed_size<yellow_ball_finder::action::FindYellow_Result>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<yellow_ball_finder::action::FindYellow_Result>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<yellow_ball_finder::action::FindYellow_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace yellow_ball_finder
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

}  // namespace yellow_ball_finder

namespace rosidl_generator_traits
{

[[deprecated("use yellow_ball_finder::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const yellow_ball_finder::action::FindYellow_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  yellow_ball_finder::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use yellow_ball_finder::action::to_yaml() instead")]]
inline std::string to_yaml(const yellow_ball_finder::action::FindYellow_Feedback & msg)
{
  return yellow_ball_finder::action::to_yaml(msg);
}

template<>
inline const char * data_type<yellow_ball_finder::action::FindYellow_Feedback>()
{
  return "yellow_ball_finder::action::FindYellow_Feedback";
}

template<>
inline const char * name<yellow_ball_finder::action::FindYellow_Feedback>()
{
  return "yellow_ball_finder/action/FindYellow_Feedback";
}

template<>
struct has_fixed_size<yellow_ball_finder::action::FindYellow_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<yellow_ball_finder::action::FindYellow_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<yellow_ball_finder::action::FindYellow_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "yellow_ball_finder/action/detail/find_yellow__traits.hpp"

namespace yellow_ball_finder
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

}  // namespace yellow_ball_finder

namespace rosidl_generator_traits
{

[[deprecated("use yellow_ball_finder::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const yellow_ball_finder::action::FindYellow_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  yellow_ball_finder::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use yellow_ball_finder::action::to_yaml() instead")]]
inline std::string to_yaml(const yellow_ball_finder::action::FindYellow_SendGoal_Request & msg)
{
  return yellow_ball_finder::action::to_yaml(msg);
}

template<>
inline const char * data_type<yellow_ball_finder::action::FindYellow_SendGoal_Request>()
{
  return "yellow_ball_finder::action::FindYellow_SendGoal_Request";
}

template<>
inline const char * name<yellow_ball_finder::action::FindYellow_SendGoal_Request>()
{
  return "yellow_ball_finder/action/FindYellow_SendGoal_Request";
}

template<>
struct has_fixed_size<yellow_ball_finder::action::FindYellow_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value && has_fixed_size<yellow_ball_finder::action::FindYellow_Goal>::value> {};

template<>
struct has_bounded_size<yellow_ball_finder::action::FindYellow_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value && has_bounded_size<yellow_ball_finder::action::FindYellow_Goal>::value> {};

template<>
struct is_message<yellow_ball_finder::action::FindYellow_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace yellow_ball_finder
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

}  // namespace yellow_ball_finder

namespace rosidl_generator_traits
{

[[deprecated("use yellow_ball_finder::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const yellow_ball_finder::action::FindYellow_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  yellow_ball_finder::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use yellow_ball_finder::action::to_yaml() instead")]]
inline std::string to_yaml(const yellow_ball_finder::action::FindYellow_SendGoal_Response & msg)
{
  return yellow_ball_finder::action::to_yaml(msg);
}

template<>
inline const char * data_type<yellow_ball_finder::action::FindYellow_SendGoal_Response>()
{
  return "yellow_ball_finder::action::FindYellow_SendGoal_Response";
}

template<>
inline const char * name<yellow_ball_finder::action::FindYellow_SendGoal_Response>()
{
  return "yellow_ball_finder/action/FindYellow_SendGoal_Response";
}

template<>
struct has_fixed_size<yellow_ball_finder::action::FindYellow_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<yellow_ball_finder::action::FindYellow_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<yellow_ball_finder::action::FindYellow_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<yellow_ball_finder::action::FindYellow_SendGoal>()
{
  return "yellow_ball_finder::action::FindYellow_SendGoal";
}

template<>
inline const char * name<yellow_ball_finder::action::FindYellow_SendGoal>()
{
  return "yellow_ball_finder/action/FindYellow_SendGoal";
}

template<>
struct has_fixed_size<yellow_ball_finder::action::FindYellow_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<yellow_ball_finder::action::FindYellow_SendGoal_Request>::value &&
    has_fixed_size<yellow_ball_finder::action::FindYellow_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<yellow_ball_finder::action::FindYellow_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<yellow_ball_finder::action::FindYellow_SendGoal_Request>::value &&
    has_bounded_size<yellow_ball_finder::action::FindYellow_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<yellow_ball_finder::action::FindYellow_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<yellow_ball_finder::action::FindYellow_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<yellow_ball_finder::action::FindYellow_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace yellow_ball_finder
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

}  // namespace yellow_ball_finder

namespace rosidl_generator_traits
{

[[deprecated("use yellow_ball_finder::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const yellow_ball_finder::action::FindYellow_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  yellow_ball_finder::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use yellow_ball_finder::action::to_yaml() instead")]]
inline std::string to_yaml(const yellow_ball_finder::action::FindYellow_GetResult_Request & msg)
{
  return yellow_ball_finder::action::to_yaml(msg);
}

template<>
inline const char * data_type<yellow_ball_finder::action::FindYellow_GetResult_Request>()
{
  return "yellow_ball_finder::action::FindYellow_GetResult_Request";
}

template<>
inline const char * name<yellow_ball_finder::action::FindYellow_GetResult_Request>()
{
  return "yellow_ball_finder/action/FindYellow_GetResult_Request";
}

template<>
struct has_fixed_size<yellow_ball_finder::action::FindYellow_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<yellow_ball_finder::action::FindYellow_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<yellow_ball_finder::action::FindYellow_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "yellow_ball_finder/action/detail/find_yellow__traits.hpp"

namespace yellow_ball_finder
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

}  // namespace yellow_ball_finder

namespace rosidl_generator_traits
{

[[deprecated("use yellow_ball_finder::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const yellow_ball_finder::action::FindYellow_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  yellow_ball_finder::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use yellow_ball_finder::action::to_yaml() instead")]]
inline std::string to_yaml(const yellow_ball_finder::action::FindYellow_GetResult_Response & msg)
{
  return yellow_ball_finder::action::to_yaml(msg);
}

template<>
inline const char * data_type<yellow_ball_finder::action::FindYellow_GetResult_Response>()
{
  return "yellow_ball_finder::action::FindYellow_GetResult_Response";
}

template<>
inline const char * name<yellow_ball_finder::action::FindYellow_GetResult_Response>()
{
  return "yellow_ball_finder/action/FindYellow_GetResult_Response";
}

template<>
struct has_fixed_size<yellow_ball_finder::action::FindYellow_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<yellow_ball_finder::action::FindYellow_Result>::value> {};

template<>
struct has_bounded_size<yellow_ball_finder::action::FindYellow_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<yellow_ball_finder::action::FindYellow_Result>::value> {};

template<>
struct is_message<yellow_ball_finder::action::FindYellow_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<yellow_ball_finder::action::FindYellow_GetResult>()
{
  return "yellow_ball_finder::action::FindYellow_GetResult";
}

template<>
inline const char * name<yellow_ball_finder::action::FindYellow_GetResult>()
{
  return "yellow_ball_finder/action/FindYellow_GetResult";
}

template<>
struct has_fixed_size<yellow_ball_finder::action::FindYellow_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<yellow_ball_finder::action::FindYellow_GetResult_Request>::value &&
    has_fixed_size<yellow_ball_finder::action::FindYellow_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<yellow_ball_finder::action::FindYellow_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<yellow_ball_finder::action::FindYellow_GetResult_Request>::value &&
    has_bounded_size<yellow_ball_finder::action::FindYellow_GetResult_Response>::value
  >
{
};

template<>
struct is_service<yellow_ball_finder::action::FindYellow_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<yellow_ball_finder::action::FindYellow_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<yellow_ball_finder::action::FindYellow_GetResult_Response>
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
// #include "yellow_ball_finder/action/detail/find_yellow__traits.hpp"

namespace yellow_ball_finder
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

}  // namespace yellow_ball_finder

namespace rosidl_generator_traits
{

[[deprecated("use yellow_ball_finder::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const yellow_ball_finder::action::FindYellow_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  yellow_ball_finder::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use yellow_ball_finder::action::to_yaml() instead")]]
inline std::string to_yaml(const yellow_ball_finder::action::FindYellow_FeedbackMessage & msg)
{
  return yellow_ball_finder::action::to_yaml(msg);
}

template<>
inline const char * data_type<yellow_ball_finder::action::FindYellow_FeedbackMessage>()
{
  return "yellow_ball_finder::action::FindYellow_FeedbackMessage";
}

template<>
inline const char * name<yellow_ball_finder::action::FindYellow_FeedbackMessage>()
{
  return "yellow_ball_finder/action/FindYellow_FeedbackMessage";
}

template<>
struct has_fixed_size<yellow_ball_finder::action::FindYellow_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value && has_fixed_size<yellow_ball_finder::action::FindYellow_Feedback>::value> {};

template<>
struct has_bounded_size<yellow_ball_finder::action::FindYellow_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value && has_bounded_size<yellow_ball_finder::action::FindYellow_Feedback>::value> {};

template<>
struct is_message<yellow_ball_finder::action::FindYellow_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<yellow_ball_finder::action::FindYellow>
  : std::true_type
{
};

template<>
struct is_action_goal<yellow_ball_finder::action::FindYellow_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<yellow_ball_finder::action::FindYellow_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<yellow_ball_finder::action::FindYellow_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // YELLOW_BALL_FINDER__ACTION__DETAIL__FIND_YELLOW__TRAITS_HPP_
