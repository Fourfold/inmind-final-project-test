// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from distance_interfaces:srv/GetFrontDistance.idl
// generated code does not contain a copyright notice

#ifndef DISTANCE_INTERFACES__SRV__DETAIL__GET_FRONT_DISTANCE__BUILDER_HPP_
#define DISTANCE_INTERFACES__SRV__DETAIL__GET_FRONT_DISTANCE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "distance_interfaces/srv/detail/get_front_distance__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace distance_interfaces
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::distance_interfaces::srv::GetFrontDistance_Request>()
{
  return ::distance_interfaces::srv::GetFrontDistance_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace distance_interfaces


namespace distance_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetFrontDistance_Response_front_distance
{
public:
  Init_GetFrontDistance_Response_front_distance()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::distance_interfaces::srv::GetFrontDistance_Response front_distance(::distance_interfaces::srv::GetFrontDistance_Response::_front_distance_type arg)
  {
    msg_.front_distance = std::move(arg);
    return std::move(msg_);
  }

private:
  ::distance_interfaces::srv::GetFrontDistance_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::distance_interfaces::srv::GetFrontDistance_Response>()
{
  return distance_interfaces::srv::builder::Init_GetFrontDistance_Response_front_distance();
}

}  // namespace distance_interfaces

#endif  // DISTANCE_INTERFACES__SRV__DETAIL__GET_FRONT_DISTANCE__BUILDER_HPP_
