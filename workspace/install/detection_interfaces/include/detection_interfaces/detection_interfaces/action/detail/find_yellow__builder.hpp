// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from detection_interfaces:action/FindYellow.idl
// generated code does not contain a copyright notice

#ifndef DETECTION_INTERFACES__ACTION__DETAIL__FIND_YELLOW__BUILDER_HPP_
#define DETECTION_INTERFACES__ACTION__DETAIL__FIND_YELLOW__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "detection_interfaces/action/detail/find_yellow__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace detection_interfaces
{

namespace action
{


}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::detection_interfaces::action::FindYellow_Goal>()
{
  return ::detection_interfaces::action::FindYellow_Goal(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace detection_interfaces


namespace detection_interfaces
{

namespace action
{

namespace builder
{

class Init_FindYellow_Result_cy
{
public:
  explicit Init_FindYellow_Result_cy(::detection_interfaces::action::FindYellow_Result & msg)
  : msg_(msg)
  {}
  ::detection_interfaces::action::FindYellow_Result cy(::detection_interfaces::action::FindYellow_Result::_cy_type arg)
  {
    msg_.cy = std::move(arg);
    return std::move(msg_);
  }

private:
  ::detection_interfaces::action::FindYellow_Result msg_;
};

class Init_FindYellow_Result_cx
{
public:
  explicit Init_FindYellow_Result_cx(::detection_interfaces::action::FindYellow_Result & msg)
  : msg_(msg)
  {}
  Init_FindYellow_Result_cy cx(::detection_interfaces::action::FindYellow_Result::_cx_type arg)
  {
    msg_.cx = std::move(arg);
    return Init_FindYellow_Result_cy(msg_);
  }

private:
  ::detection_interfaces::action::FindYellow_Result msg_;
};

class Init_FindYellow_Result_found
{
public:
  Init_FindYellow_Result_found()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FindYellow_Result_cx found(::detection_interfaces::action::FindYellow_Result::_found_type arg)
  {
    msg_.found = std::move(arg);
    return Init_FindYellow_Result_cx(msg_);
  }

private:
  ::detection_interfaces::action::FindYellow_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::detection_interfaces::action::FindYellow_Result>()
{
  return detection_interfaces::action::builder::Init_FindYellow_Result_found();
}

}  // namespace detection_interfaces


namespace detection_interfaces
{

namespace action
{


}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::detection_interfaces::action::FindYellow_Feedback>()
{
  return ::detection_interfaces::action::FindYellow_Feedback(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace detection_interfaces


namespace detection_interfaces
{

namespace action
{

namespace builder
{

class Init_FindYellow_SendGoal_Request_goal
{
public:
  explicit Init_FindYellow_SendGoal_Request_goal(::detection_interfaces::action::FindYellow_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::detection_interfaces::action::FindYellow_SendGoal_Request goal(::detection_interfaces::action::FindYellow_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::detection_interfaces::action::FindYellow_SendGoal_Request msg_;
};

class Init_FindYellow_SendGoal_Request_goal_id
{
public:
  Init_FindYellow_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FindYellow_SendGoal_Request_goal goal_id(::detection_interfaces::action::FindYellow_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_FindYellow_SendGoal_Request_goal(msg_);
  }

private:
  ::detection_interfaces::action::FindYellow_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::detection_interfaces::action::FindYellow_SendGoal_Request>()
{
  return detection_interfaces::action::builder::Init_FindYellow_SendGoal_Request_goal_id();
}

}  // namespace detection_interfaces


namespace detection_interfaces
{

namespace action
{

namespace builder
{

class Init_FindYellow_SendGoal_Response_stamp
{
public:
  explicit Init_FindYellow_SendGoal_Response_stamp(::detection_interfaces::action::FindYellow_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::detection_interfaces::action::FindYellow_SendGoal_Response stamp(::detection_interfaces::action::FindYellow_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::detection_interfaces::action::FindYellow_SendGoal_Response msg_;
};

class Init_FindYellow_SendGoal_Response_accepted
{
public:
  Init_FindYellow_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FindYellow_SendGoal_Response_stamp accepted(::detection_interfaces::action::FindYellow_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_FindYellow_SendGoal_Response_stamp(msg_);
  }

private:
  ::detection_interfaces::action::FindYellow_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::detection_interfaces::action::FindYellow_SendGoal_Response>()
{
  return detection_interfaces::action::builder::Init_FindYellow_SendGoal_Response_accepted();
}

}  // namespace detection_interfaces


namespace detection_interfaces
{

namespace action
{

namespace builder
{

class Init_FindYellow_GetResult_Request_goal_id
{
public:
  Init_FindYellow_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::detection_interfaces::action::FindYellow_GetResult_Request goal_id(::detection_interfaces::action::FindYellow_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::detection_interfaces::action::FindYellow_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::detection_interfaces::action::FindYellow_GetResult_Request>()
{
  return detection_interfaces::action::builder::Init_FindYellow_GetResult_Request_goal_id();
}

}  // namespace detection_interfaces


namespace detection_interfaces
{

namespace action
{

namespace builder
{

class Init_FindYellow_GetResult_Response_result
{
public:
  explicit Init_FindYellow_GetResult_Response_result(::detection_interfaces::action::FindYellow_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::detection_interfaces::action::FindYellow_GetResult_Response result(::detection_interfaces::action::FindYellow_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::detection_interfaces::action::FindYellow_GetResult_Response msg_;
};

class Init_FindYellow_GetResult_Response_status
{
public:
  Init_FindYellow_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FindYellow_GetResult_Response_result status(::detection_interfaces::action::FindYellow_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_FindYellow_GetResult_Response_result(msg_);
  }

private:
  ::detection_interfaces::action::FindYellow_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::detection_interfaces::action::FindYellow_GetResult_Response>()
{
  return detection_interfaces::action::builder::Init_FindYellow_GetResult_Response_status();
}

}  // namespace detection_interfaces


namespace detection_interfaces
{

namespace action
{

namespace builder
{

class Init_FindYellow_FeedbackMessage_feedback
{
public:
  explicit Init_FindYellow_FeedbackMessage_feedback(::detection_interfaces::action::FindYellow_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::detection_interfaces::action::FindYellow_FeedbackMessage feedback(::detection_interfaces::action::FindYellow_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::detection_interfaces::action::FindYellow_FeedbackMessage msg_;
};

class Init_FindYellow_FeedbackMessage_goal_id
{
public:
  Init_FindYellow_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FindYellow_FeedbackMessage_feedback goal_id(::detection_interfaces::action::FindYellow_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_FindYellow_FeedbackMessage_feedback(msg_);
  }

private:
  ::detection_interfaces::action::FindYellow_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::detection_interfaces::action::FindYellow_FeedbackMessage>()
{
  return detection_interfaces::action::builder::Init_FindYellow_FeedbackMessage_goal_id();
}

}  // namespace detection_interfaces

#endif  // DETECTION_INTERFACES__ACTION__DETAIL__FIND_YELLOW__BUILDER_HPP_
