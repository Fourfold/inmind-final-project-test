// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from distance_interfaces:srv/GetFrontDistance.idl
// generated code does not contain a copyright notice

#ifndef DISTANCE_INTERFACES__SRV__DETAIL__GET_FRONT_DISTANCE__TRAITS_HPP_
#define DISTANCE_INTERFACES__SRV__DETAIL__GET_FRONT_DISTANCE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "distance_interfaces/srv/detail/get_front_distance__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace distance_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetFrontDistance_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetFrontDistance_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetFrontDistance_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace distance_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use distance_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const distance_interfaces::srv::GetFrontDistance_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  distance_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use distance_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const distance_interfaces::srv::GetFrontDistance_Request & msg)
{
  return distance_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<distance_interfaces::srv::GetFrontDistance_Request>()
{
  return "distance_interfaces::srv::GetFrontDistance_Request";
}

template<>
inline const char * name<distance_interfaces::srv::GetFrontDistance_Request>()
{
  return "distance_interfaces/srv/GetFrontDistance_Request";
}

template<>
struct has_fixed_size<distance_interfaces::srv::GetFrontDistance_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<distance_interfaces::srv::GetFrontDistance_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<distance_interfaces::srv::GetFrontDistance_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace distance_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetFrontDistance_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: front_distance
  {
    out << "front_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.front_distance, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetFrontDistance_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: front_distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "front_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.front_distance, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetFrontDistance_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace distance_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use distance_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const distance_interfaces::srv::GetFrontDistance_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  distance_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use distance_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const distance_interfaces::srv::GetFrontDistance_Response & msg)
{
  return distance_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<distance_interfaces::srv::GetFrontDistance_Response>()
{
  return "distance_interfaces::srv::GetFrontDistance_Response";
}

template<>
inline const char * name<distance_interfaces::srv::GetFrontDistance_Response>()
{
  return "distance_interfaces/srv/GetFrontDistance_Response";
}

template<>
struct has_fixed_size<distance_interfaces::srv::GetFrontDistance_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<distance_interfaces::srv::GetFrontDistance_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<distance_interfaces::srv::GetFrontDistance_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<distance_interfaces::srv::GetFrontDistance>()
{
  return "distance_interfaces::srv::GetFrontDistance";
}

template<>
inline const char * name<distance_interfaces::srv::GetFrontDistance>()
{
  return "distance_interfaces/srv/GetFrontDistance";
}

template<>
struct has_fixed_size<distance_interfaces::srv::GetFrontDistance>
  : std::integral_constant<
    bool,
    has_fixed_size<distance_interfaces::srv::GetFrontDistance_Request>::value &&
    has_fixed_size<distance_interfaces::srv::GetFrontDistance_Response>::value
  >
{
};

template<>
struct has_bounded_size<distance_interfaces::srv::GetFrontDistance>
  : std::integral_constant<
    bool,
    has_bounded_size<distance_interfaces::srv::GetFrontDistance_Request>::value &&
    has_bounded_size<distance_interfaces::srv::GetFrontDistance_Response>::value
  >
{
};

template<>
struct is_service<distance_interfaces::srv::GetFrontDistance>
  : std::true_type
{
};

template<>
struct is_service_request<distance_interfaces::srv::GetFrontDistance_Request>
  : std::true_type
{
};

template<>
struct is_service_response<distance_interfaces::srv::GetFrontDistance_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // DISTANCE_INTERFACES__SRV__DETAIL__GET_FRONT_DISTANCE__TRAITS_HPP_
