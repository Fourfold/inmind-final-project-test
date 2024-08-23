// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from yellow_ball_finder:action/FindYellow.idl
// generated code does not contain a copyright notice

#ifndef YELLOW_BALL_FINDER__ACTION__DETAIL__FIND_YELLOW__BUILDER_HPP_
#define YELLOW_BALL_FINDER__ACTION__DETAIL__FIND_YELLOW__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "yellow_ball_finder/action/detail/find_yellow__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace yellow_ball_finder
{

namespace action
{


}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::yellow_ball_finder::action::FindYellow_Goal>()
{
  return ::yellow_ball_finder::action::FindYellow_Goal(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace yellow_ball_finder


namespace yellow_ball_finder
{

namespace action
{

namespace builder
{

class Init_FindYellow_Result_cy
{
public:
  explicit Init_FindYellow_Result_cy(::yellow_ball_finder::action::FindYellow_Result & msg)
  : msg_(msg)
  {}
  ::yellow_ball_finder::action::FindYellow_Result cy(::yellow_ball_finder::action::FindYellow_Result::_cy_type arg)
  {
    msg_.cy = std::move(arg);
    return std::move(msg_);
  }

private:
  ::yellow_ball_finder::action::FindYellow_Result msg_;
};

class Init_FindYellow_Result_cx
{
public:
  explicit Init_FindYellow_Result_cx(::yellow_ball_finder::action::FindYellow_Result & msg)
  : msg_(msg)
  {}
  Init_FindYellow_Result_cy cx(::yellow_ball_finder::action::FindYellow_Result::_cx_type arg)
  {
    msg_.cx = std::move(arg);
    return Init_FindYellow_Result_cy(msg_);
  }

private:
  ::yellow_ball_finder::action::FindYellow_Result msg_;
};

class Init_FindYellow_Result_found
{
public:
  Init_FindYellow_Result_found()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FindYellow_Result_cx found(::yellow_ball_finder::action::FindYellow_Result::_found_type arg)
  {
    msg_.found = std::move(arg);
    return Init_FindYellow_Result_cx(msg_);
  }

private:
  ::yellow_ball_finder::action::FindYellow_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::yellow_ball_finder::action::FindYellow_Result>()
{
  return yellow_ball_finder::action::builder::Init_FindYellow_Result_found();
}

}  // namespace yellow_ball_finder


namespace yellow_ball_finder
{

namespace action
{


}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::yellow_ball_finder::action::FindYellow_Feedback>()
{
  return ::yellow_ball_finder::action::FindYellow_Feedback(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace yellow_ball_finder


namespace yellow_ball_finder
{

namespace action
{

namespace builder
{

class Init_FindYellow_SendGoal_Request_goal
{
public:
  explicit Init_FindYellow_SendGoal_Request_goal(::yellow_ball_finder::action::FindYellow_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::yellow_ball_finder::action::FindYellow_SendGoal_Request goal(::yellow_ball_finder::action::FindYellow_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::yellow_ball_finder::action::FindYellow_SendGoal_Request msg_;
};

class Init_FindYellow_SendGoal_Request_goal_id
{
public:
  Init_FindYellow_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FindYellow_SendGoal_Request_goal goal_id(::yellow_ball_finder::action::FindYellow_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_FindYellow_SendGoal_Request_goal(msg_);
  }

private:
  ::yellow_ball_finder::action::FindYellow_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::yellow_ball_finder::action::FindYellow_SendGoal_Request>()
{
  return yellow_ball_finder::action::builder::Init_FindYellow_SendGoal_Request_goal_id();
}

}  // namespace yellow_ball_finder


namespace yellow_ball_finder
{

namespace action
{

namespace builder
{

class Init_FindYellow_SendGoal_Response_stamp
{
public:
  explicit Init_FindYellow_SendGoal_Response_stamp(::yellow_ball_finder::action::FindYellow_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::yellow_ball_finder::action::FindYellow_SendGoal_Response stamp(::yellow_ball_finder::action::FindYellow_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::yellow_ball_finder::action::FindYellow_SendGoal_Response msg_;
};

class Init_FindYellow_SendGoal_Response_accepted
{
public:
  Init_FindYellow_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FindYellow_SendGoal_Response_stamp accepted(::yellow_ball_finder::action::FindYellow_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_FindYellow_SendGoal_Response_stamp(msg_);
  }

private:
  ::yellow_ball_finder::action::FindYellow_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::yellow_ball_finder::action::FindYellow_SendGoal_Response>()
{
  return yellow_ball_finder::action::builder::Init_FindYellow_SendGoal_Response_accepted();
}

}  // namespace yellow_ball_finder


namespace yellow_ball_finder
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
  ::yellow_ball_finder::action::FindYellow_GetResult_Request goal_id(::yellow_ball_finder::action::FindYellow_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::yellow_ball_finder::action::FindYellow_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::yellow_ball_finder::action::FindYellow_GetResult_Request>()
{
  return yellow_ball_finder::action::builder::Init_FindYellow_GetResult_Request_goal_id();
}

}  // namespace yellow_ball_finder


namespace yellow_ball_finder
{

namespace action
{

namespace builder
{

class Init_FindYellow_GetResult_Response_result
{
public:
  explicit Init_FindYellow_GetResult_Response_result(::yellow_ball_finder::action::FindYellow_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::yellow_ball_finder::action::FindYellow_GetResult_Response result(::yellow_ball_finder::action::FindYellow_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::yellow_ball_finder::action::FindYellow_GetResult_Response msg_;
};

class Init_FindYellow_GetResult_Response_status
{
public:
  Init_FindYellow_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FindYellow_GetResult_Response_result status(::yellow_ball_finder::action::FindYellow_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_FindYellow_GetResult_Response_result(msg_);
  }

private:
  ::yellow_ball_finder::action::FindYellow_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::yellow_ball_finder::action::FindYellow_GetResult_Response>()
{
  return yellow_ball_finder::action::builder::Init_FindYellow_GetResult_Response_status();
}

}  // namespace yellow_ball_finder


namespace yellow_ball_finder
{

namespace action
{

namespace builder
{

class Init_FindYellow_FeedbackMessage_feedback
{
public:
  explicit Init_FindYellow_FeedbackMessage_feedback(::yellow_ball_finder::action::FindYellow_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::yellow_ball_finder::action::FindYellow_FeedbackMessage feedback(::yellow_ball_finder::action::FindYellow_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::yellow_ball_finder::action::FindYellow_FeedbackMessage msg_;
};

class Init_FindYellow_FeedbackMessage_goal_id
{
public:
  Init_FindYellow_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FindYellow_FeedbackMessage_feedback goal_id(::yellow_ball_finder::action::FindYellow_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_FindYellow_FeedbackMessage_feedback(msg_);
  }

private:
  ::yellow_ball_finder::action::FindYellow_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::yellow_ball_finder::action::FindYellow_FeedbackMessage>()
{
  return yellow_ball_finder::action::builder::Init_FindYellow_FeedbackMessage_goal_id();
}

}  // namespace yellow_ball_finder

#endif  // YELLOW_BALL_FINDER__ACTION__DETAIL__FIND_YELLOW__BUILDER_HPP_
