// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from motor_feedback:msg/MotorFeedbackWithState.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_FEEDBACK__MSG__DETAIL__MOTOR_FEEDBACK_WITH_STATE__BUILDER_HPP_
#define MOTOR_FEEDBACK__MSG__DETAIL__MOTOR_FEEDBACK_WITH_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "motor_feedback/msg/detail/motor_feedback_with_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace motor_feedback
{

namespace msg
{

namespace builder
{

class Init_MotorFeedbackWithState_state
{
public:
  explicit Init_MotorFeedbackWithState_state(::motor_feedback::msg::MotorFeedbackWithState & msg)
  : msg_(msg)
  {}
  ::motor_feedback::msg::MotorFeedbackWithState state(::motor_feedback::msg::MotorFeedbackWithState::_state_type arg)
  {
    msg_.state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::motor_feedback::msg::MotorFeedbackWithState msg_;
};

class Init_MotorFeedbackWithState_effort
{
public:
  explicit Init_MotorFeedbackWithState_effort(::motor_feedback::msg::MotorFeedbackWithState & msg)
  : msg_(msg)
  {}
  Init_MotorFeedbackWithState_state effort(::motor_feedback::msg::MotorFeedbackWithState::_effort_type arg)
  {
    msg_.effort = std::move(arg);
    return Init_MotorFeedbackWithState_state(msg_);
  }

private:
  ::motor_feedback::msg::MotorFeedbackWithState msg_;
};

class Init_MotorFeedbackWithState_velocity
{
public:
  explicit Init_MotorFeedbackWithState_velocity(::motor_feedback::msg::MotorFeedbackWithState & msg)
  : msg_(msg)
  {}
  Init_MotorFeedbackWithState_effort velocity(::motor_feedback::msg::MotorFeedbackWithState::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_MotorFeedbackWithState_effort(msg_);
  }

private:
  ::motor_feedback::msg::MotorFeedbackWithState msg_;
};

class Init_MotorFeedbackWithState_position
{
public:
  explicit Init_MotorFeedbackWithState_position(::motor_feedback::msg::MotorFeedbackWithState & msg)
  : msg_(msg)
  {}
  Init_MotorFeedbackWithState_velocity position(::motor_feedback::msg::MotorFeedbackWithState::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_MotorFeedbackWithState_velocity(msg_);
  }

private:
  ::motor_feedback::msg::MotorFeedbackWithState msg_;
};

class Init_MotorFeedbackWithState_id
{
public:
  Init_MotorFeedbackWithState_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorFeedbackWithState_position id(::motor_feedback::msg::MotorFeedbackWithState::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_MotorFeedbackWithState_position(msg_);
  }

private:
  ::motor_feedback::msg::MotorFeedbackWithState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::motor_feedback::msg::MotorFeedbackWithState>()
{
  return motor_feedback::msg::builder::Init_MotorFeedbackWithState_id();
}

}  // namespace motor_feedback

#endif  // MOTOR_FEEDBACK__MSG__DETAIL__MOTOR_FEEDBACK_WITH_STATE__BUILDER_HPP_
