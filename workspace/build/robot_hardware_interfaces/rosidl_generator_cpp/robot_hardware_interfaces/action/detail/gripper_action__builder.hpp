// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_hardware_interfaces:action/GripperAction.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_HARDWARE_INTERFACES__ACTION__DETAIL__GRIPPER_ACTION__BUILDER_HPP_
#define ROBOT_HARDWARE_INTERFACES__ACTION__DETAIL__GRIPPER_ACTION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_hardware_interfaces/action/detail/gripper_action__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_hardware_interfaces
{

namespace action
{


}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_hardware_interfaces::action::GripperAction_Goal>()
{
  return ::robot_hardware_interfaces::action::GripperAction_Goal(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace robot_hardware_interfaces


namespace robot_hardware_interfaces
{

namespace action
{

namespace builder
{

class Init_GripperAction_Result_result
{
public:
  Init_GripperAction_Result_result()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robot_hardware_interfaces::action::GripperAction_Result result(::robot_hardware_interfaces::action::GripperAction_Result::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_hardware_interfaces::action::GripperAction_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_hardware_interfaces::action::GripperAction_Result>()
{
  return robot_hardware_interfaces::action::builder::Init_GripperAction_Result_result();
}

}  // namespace robot_hardware_interfaces


namespace robot_hardware_interfaces
{

namespace action
{

namespace builder
{

class Init_GripperAction_Feedback_feedback
{
public:
  Init_GripperAction_Feedback_feedback()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robot_hardware_interfaces::action::GripperAction_Feedback feedback(::robot_hardware_interfaces::action::GripperAction_Feedback::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_hardware_interfaces::action::GripperAction_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_hardware_interfaces::action::GripperAction_Feedback>()
{
  return robot_hardware_interfaces::action::builder::Init_GripperAction_Feedback_feedback();
}

}  // namespace robot_hardware_interfaces


namespace robot_hardware_interfaces
{

namespace action
{

namespace builder
{

class Init_GripperAction_SendGoal_Request_goal
{
public:
  explicit Init_GripperAction_SendGoal_Request_goal(::robot_hardware_interfaces::action::GripperAction_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::robot_hardware_interfaces::action::GripperAction_SendGoal_Request goal(::robot_hardware_interfaces::action::GripperAction_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_hardware_interfaces::action::GripperAction_SendGoal_Request msg_;
};

class Init_GripperAction_SendGoal_Request_goal_id
{
public:
  Init_GripperAction_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GripperAction_SendGoal_Request_goal goal_id(::robot_hardware_interfaces::action::GripperAction_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_GripperAction_SendGoal_Request_goal(msg_);
  }

private:
  ::robot_hardware_interfaces::action::GripperAction_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_hardware_interfaces::action::GripperAction_SendGoal_Request>()
{
  return robot_hardware_interfaces::action::builder::Init_GripperAction_SendGoal_Request_goal_id();
}

}  // namespace robot_hardware_interfaces


namespace robot_hardware_interfaces
{

namespace action
{

namespace builder
{

class Init_GripperAction_SendGoal_Response_stamp
{
public:
  explicit Init_GripperAction_SendGoal_Response_stamp(::robot_hardware_interfaces::action::GripperAction_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::robot_hardware_interfaces::action::GripperAction_SendGoal_Response stamp(::robot_hardware_interfaces::action::GripperAction_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_hardware_interfaces::action::GripperAction_SendGoal_Response msg_;
};

class Init_GripperAction_SendGoal_Response_accepted
{
public:
  Init_GripperAction_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GripperAction_SendGoal_Response_stamp accepted(::robot_hardware_interfaces::action::GripperAction_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_GripperAction_SendGoal_Response_stamp(msg_);
  }

private:
  ::robot_hardware_interfaces::action::GripperAction_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_hardware_interfaces::action::GripperAction_SendGoal_Response>()
{
  return robot_hardware_interfaces::action::builder::Init_GripperAction_SendGoal_Response_accepted();
}

}  // namespace robot_hardware_interfaces


namespace robot_hardware_interfaces
{

namespace action
{

namespace builder
{

class Init_GripperAction_GetResult_Request_goal_id
{
public:
  Init_GripperAction_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robot_hardware_interfaces::action::GripperAction_GetResult_Request goal_id(::robot_hardware_interfaces::action::GripperAction_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_hardware_interfaces::action::GripperAction_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_hardware_interfaces::action::GripperAction_GetResult_Request>()
{
  return robot_hardware_interfaces::action::builder::Init_GripperAction_GetResult_Request_goal_id();
}

}  // namespace robot_hardware_interfaces


namespace robot_hardware_interfaces
{

namespace action
{

namespace builder
{

class Init_GripperAction_GetResult_Response_result
{
public:
  explicit Init_GripperAction_GetResult_Response_result(::robot_hardware_interfaces::action::GripperAction_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::robot_hardware_interfaces::action::GripperAction_GetResult_Response result(::robot_hardware_interfaces::action::GripperAction_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_hardware_interfaces::action::GripperAction_GetResult_Response msg_;
};

class Init_GripperAction_GetResult_Response_status
{
public:
  Init_GripperAction_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GripperAction_GetResult_Response_result status(::robot_hardware_interfaces::action::GripperAction_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_GripperAction_GetResult_Response_result(msg_);
  }

private:
  ::robot_hardware_interfaces::action::GripperAction_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_hardware_interfaces::action::GripperAction_GetResult_Response>()
{
  return robot_hardware_interfaces::action::builder::Init_GripperAction_GetResult_Response_status();
}

}  // namespace robot_hardware_interfaces


namespace robot_hardware_interfaces
{

namespace action
{

namespace builder
{

class Init_GripperAction_FeedbackMessage_feedback
{
public:
  explicit Init_GripperAction_FeedbackMessage_feedback(::robot_hardware_interfaces::action::GripperAction_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::robot_hardware_interfaces::action::GripperAction_FeedbackMessage feedback(::robot_hardware_interfaces::action::GripperAction_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_hardware_interfaces::action::GripperAction_FeedbackMessage msg_;
};

class Init_GripperAction_FeedbackMessage_goal_id
{
public:
  Init_GripperAction_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GripperAction_FeedbackMessage_feedback goal_id(::robot_hardware_interfaces::action::GripperAction_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_GripperAction_FeedbackMessage_feedback(msg_);
  }

private:
  ::robot_hardware_interfaces::action::GripperAction_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_hardware_interfaces::action::GripperAction_FeedbackMessage>()
{
  return robot_hardware_interfaces::action::builder::Init_GripperAction_FeedbackMessage_goal_id();
}

}  // namespace robot_hardware_interfaces

#endif  // ROBOT_HARDWARE_INTERFACES__ACTION__DETAIL__GRIPPER_ACTION__BUILDER_HPP_
