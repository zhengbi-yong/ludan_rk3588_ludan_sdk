// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from multi_port_motor_feedback:msg/LowState.idl
// generated code does not contain a copyright notice

#ifndef MULTI_PORT_MOTOR_FEEDBACK__MSG__DETAIL__LOW_STATE__BUILDER_HPP_
#define MULTI_PORT_MOTOR_FEEDBACK__MSG__DETAIL__LOW_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "multi_port_motor_feedback/msg/detail/low_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace multi_port_motor_feedback
{

namespace msg
{

namespace builder
{

class Init_LowState_motor_state
{
public:
  explicit Init_LowState_motor_state(::multi_port_motor_feedback::msg::LowState & msg)
  : msg_(msg)
  {}
  ::multi_port_motor_feedback::msg::LowState motor_state(::multi_port_motor_feedback::msg::LowState::_motor_state_type arg)
  {
    msg_.motor_state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::multi_port_motor_feedback::msg::LowState msg_;
};

class Init_LowState_header
{
public:
  Init_LowState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LowState_motor_state header(::multi_port_motor_feedback::msg::LowState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_LowState_motor_state(msg_);
  }

private:
  ::multi_port_motor_feedback::msg::LowState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::multi_port_motor_feedback::msg::LowState>()
{
  return multi_port_motor_feedback::msg::builder::Init_LowState_header();
}

}  // namespace multi_port_motor_feedback

#endif  // MULTI_PORT_MOTOR_FEEDBACK__MSG__DETAIL__LOW_STATE__BUILDER_HPP_
