// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from motor_feedback:msg/MotorFeedback.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_FEEDBACK__MSG__DETAIL__MOTOR_FEEDBACK__BUILDER_HPP_
#define MOTOR_FEEDBACK__MSG__DETAIL__MOTOR_FEEDBACK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "motor_feedback/msg/detail/motor_feedback__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace motor_feedback
{

namespace msg
{

namespace builder
{

class Init_MotorFeedback_can_data
{
public:
  explicit Init_MotorFeedback_can_data(::motor_feedback::msg::MotorFeedback & msg)
  : msg_(msg)
  {}
  ::motor_feedback::msg::MotorFeedback can_data(::motor_feedback::msg::MotorFeedback::_can_data_type arg)
  {
    msg_.can_data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::motor_feedback::msg::MotorFeedback msg_;
};

class Init_MotorFeedback_torque_nm
{
public:
  explicit Init_MotorFeedback_torque_nm(::motor_feedback::msg::MotorFeedback & msg)
  : msg_(msg)
  {}
  Init_MotorFeedback_can_data torque_nm(::motor_feedback::msg::MotorFeedback::_torque_nm_type arg)
  {
    msg_.torque_nm = std::move(arg);
    return Init_MotorFeedback_can_data(msg_);
  }

private:
  ::motor_feedback::msg::MotorFeedback msg_;
};

class Init_MotorFeedback_velocity_rad_s
{
public:
  explicit Init_MotorFeedback_velocity_rad_s(::motor_feedback::msg::MotorFeedback & msg)
  : msg_(msg)
  {}
  Init_MotorFeedback_torque_nm velocity_rad_s(::motor_feedback::msg::MotorFeedback::_velocity_rad_s_type arg)
  {
    msg_.velocity_rad_s = std::move(arg);
    return Init_MotorFeedback_torque_nm(msg_);
  }

private:
  ::motor_feedback::msg::MotorFeedback msg_;
};

class Init_MotorFeedback_position_rad
{
public:
  explicit Init_MotorFeedback_position_rad(::motor_feedback::msg::MotorFeedback & msg)
  : msg_(msg)
  {}
  Init_MotorFeedback_velocity_rad_s position_rad(::motor_feedback::msg::MotorFeedback::_position_rad_type arg)
  {
    msg_.position_rad = std::move(arg);
    return Init_MotorFeedback_velocity_rad_s(msg_);
  }

private:
  ::motor_feedback::msg::MotorFeedback msg_;
};

class Init_MotorFeedback_temp_rotor
{
public:
  explicit Init_MotorFeedback_temp_rotor(::motor_feedback::msg::MotorFeedback & msg)
  : msg_(msg)
  {}
  Init_MotorFeedback_position_rad temp_rotor(::motor_feedback::msg::MotorFeedback::_temp_rotor_type arg)
  {
    msg_.temp_rotor = std::move(arg);
    return Init_MotorFeedback_position_rad(msg_);
  }

private:
  ::motor_feedback::msg::MotorFeedback msg_;
};

class Init_MotorFeedback_temp_mos
{
public:
  explicit Init_MotorFeedback_temp_mos(::motor_feedback::msg::MotorFeedback & msg)
  : msg_(msg)
  {}
  Init_MotorFeedback_temp_rotor temp_mos(::motor_feedback::msg::MotorFeedback::_temp_mos_type arg)
  {
    msg_.temp_mos = std::move(arg);
    return Init_MotorFeedback_temp_rotor(msg_);
  }

private:
  ::motor_feedback::msg::MotorFeedback msg_;
};

class Init_MotorFeedback_torque
{
public:
  explicit Init_MotorFeedback_torque(::motor_feedback::msg::MotorFeedback & msg)
  : msg_(msg)
  {}
  Init_MotorFeedback_temp_mos torque(::motor_feedback::msg::MotorFeedback::_torque_type arg)
  {
    msg_.torque = std::move(arg);
    return Init_MotorFeedback_temp_mos(msg_);
  }

private:
  ::motor_feedback::msg::MotorFeedback msg_;
};

class Init_MotorFeedback_velocity
{
public:
  explicit Init_MotorFeedback_velocity(::motor_feedback::msg::MotorFeedback & msg)
  : msg_(msg)
  {}
  Init_MotorFeedback_torque velocity(::motor_feedback::msg::MotorFeedback::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_MotorFeedback_torque(msg_);
  }

private:
  ::motor_feedback::msg::MotorFeedback msg_;
};

class Init_MotorFeedback_position
{
public:
  explicit Init_MotorFeedback_position(::motor_feedback::msg::MotorFeedback & msg)
  : msg_(msg)
  {}
  Init_MotorFeedback_velocity position(::motor_feedback::msg::MotorFeedback::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_MotorFeedback_velocity(msg_);
  }

private:
  ::motor_feedback::msg::MotorFeedback msg_;
};

class Init_MotorFeedback_error
{
public:
  explicit Init_MotorFeedback_error(::motor_feedback::msg::MotorFeedback & msg)
  : msg_(msg)
  {}
  Init_MotorFeedback_position error(::motor_feedback::msg::MotorFeedback::_error_type arg)
  {
    msg_.error = std::move(arg);
    return Init_MotorFeedback_position(msg_);
  }

private:
  ::motor_feedback::msg::MotorFeedback msg_;
};

class Init_MotorFeedback_can_id
{
public:
  explicit Init_MotorFeedback_can_id(::motor_feedback::msg::MotorFeedback & msg)
  : msg_(msg)
  {}
  Init_MotorFeedback_error can_id(::motor_feedback::msg::MotorFeedback::_can_id_type arg)
  {
    msg_.can_id = std::move(arg);
    return Init_MotorFeedback_error(msg_);
  }

private:
  ::motor_feedback::msg::MotorFeedback msg_;
};

class Init_MotorFeedback_motor_id
{
public:
  explicit Init_MotorFeedback_motor_id(::motor_feedback::msg::MotorFeedback & msg)
  : msg_(msg)
  {}
  Init_MotorFeedback_can_id motor_id(::motor_feedback::msg::MotorFeedback::_motor_id_type arg)
  {
    msg_.motor_id = std::move(arg);
    return Init_MotorFeedback_can_id(msg_);
  }

private:
  ::motor_feedback::msg::MotorFeedback msg_;
};

class Init_MotorFeedback_header
{
public:
  Init_MotorFeedback_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorFeedback_motor_id header(::motor_feedback::msg::MotorFeedback::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MotorFeedback_motor_id(msg_);
  }

private:
  ::motor_feedback::msg::MotorFeedback msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::motor_feedback::msg::MotorFeedback>()
{
  return motor_feedback::msg::builder::Init_MotorFeedback_header();
}

}  // namespace motor_feedback

#endif  // MOTOR_FEEDBACK__MSG__DETAIL__MOTOR_FEEDBACK__BUILDER_HPP_
