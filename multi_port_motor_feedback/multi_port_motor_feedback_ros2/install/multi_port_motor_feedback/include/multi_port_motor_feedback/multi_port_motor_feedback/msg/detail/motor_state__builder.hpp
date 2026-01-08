// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from multi_port_motor_feedback:msg/MotorState.idl
// generated code does not contain a copyright notice

#ifndef MULTI_PORT_MOTOR_FEEDBACK__MSG__DETAIL__MOTOR_STATE__BUILDER_HPP_
#define MULTI_PORT_MOTOR_FEEDBACK__MSG__DETAIL__MOTOR_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "multi_port_motor_feedback/msg/detail/motor_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace multi_port_motor_feedback
{

namespace msg
{

namespace builder
{

class Init_MotorState_temp_rotor
{
public:
  explicit Init_MotorState_temp_rotor(::multi_port_motor_feedback::msg::MotorState & msg)
  : msg_(msg)
  {}
  ::multi_port_motor_feedback::msg::MotorState temp_rotor(::multi_port_motor_feedback::msg::MotorState::_temp_rotor_type arg)
  {
    msg_.temp_rotor = std::move(arg);
    return std::move(msg_);
  }

private:
  ::multi_port_motor_feedback::msg::MotorState msg_;
};

class Init_MotorState_temp_mos
{
public:
  explicit Init_MotorState_temp_mos(::multi_port_motor_feedback::msg::MotorState & msg)
  : msg_(msg)
  {}
  Init_MotorState_temp_rotor temp_mos(::multi_port_motor_feedback::msg::MotorState::_temp_mos_type arg)
  {
    msg_.temp_mos = std::move(arg);
    return Init_MotorState_temp_rotor(msg_);
  }

private:
  ::multi_port_motor_feedback::msg::MotorState msg_;
};

class Init_MotorState_effort
{
public:
  explicit Init_MotorState_effort(::multi_port_motor_feedback::msg::MotorState & msg)
  : msg_(msg)
  {}
  Init_MotorState_temp_mos effort(::multi_port_motor_feedback::msg::MotorState::_effort_type arg)
  {
    msg_.effort = std::move(arg);
    return Init_MotorState_temp_mos(msg_);
  }

private:
  ::multi_port_motor_feedback::msg::MotorState msg_;
};

class Init_MotorState_velocity
{
public:
  explicit Init_MotorState_velocity(::multi_port_motor_feedback::msg::MotorState & msg)
  : msg_(msg)
  {}
  Init_MotorState_effort velocity(::multi_port_motor_feedback::msg::MotorState::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_MotorState_effort(msg_);
  }

private:
  ::multi_port_motor_feedback::msg::MotorState msg_;
};

class Init_MotorState_position
{
public:
  explicit Init_MotorState_position(::multi_port_motor_feedback::msg::MotorState & msg)
  : msg_(msg)
  {}
  Init_MotorState_velocity position(::multi_port_motor_feedback::msg::MotorState::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_MotorState_velocity(msg_);
  }

private:
  ::multi_port_motor_feedback::msg::MotorState msg_;
};

class Init_MotorState_id
{
public:
  explicit Init_MotorState_id(::multi_port_motor_feedback::msg::MotorState & msg)
  : msg_(msg)
  {}
  Init_MotorState_position id(::multi_port_motor_feedback::msg::MotorState::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_MotorState_position(msg_);
  }

private:
  ::multi_port_motor_feedback::msg::MotorState msg_;
};

class Init_MotorState_state
{
public:
  Init_MotorState_state()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorState_id state(::multi_port_motor_feedback::msg::MotorState::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_MotorState_id(msg_);
  }

private:
  ::multi_port_motor_feedback::msg::MotorState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::multi_port_motor_feedback::msg::MotorState>()
{
  return multi_port_motor_feedback::msg::builder::Init_MotorState_state();
}

}  // namespace multi_port_motor_feedback

#endif  // MULTI_PORT_MOTOR_FEEDBACK__MSG__DETAIL__MOTOR_STATE__BUILDER_HPP_
