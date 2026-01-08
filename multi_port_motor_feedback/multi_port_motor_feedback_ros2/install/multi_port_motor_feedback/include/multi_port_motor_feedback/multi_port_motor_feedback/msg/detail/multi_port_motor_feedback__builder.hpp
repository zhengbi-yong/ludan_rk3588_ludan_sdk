// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from multi_port_motor_feedback:msg/MultiPortMotorFeedback.idl
// generated code does not contain a copyright notice

#ifndef MULTI_PORT_MOTOR_FEEDBACK__MSG__DETAIL__MULTI_PORT_MOTOR_FEEDBACK__BUILDER_HPP_
#define MULTI_PORT_MOTOR_FEEDBACK__MSG__DETAIL__MULTI_PORT_MOTOR_FEEDBACK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "multi_port_motor_feedback/msg/detail/multi_port_motor_feedback__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace multi_port_motor_feedback
{

namespace msg
{

namespace builder
{

class Init_MultiPortMotorFeedback_can_data
{
public:
  explicit Init_MultiPortMotorFeedback_can_data(::multi_port_motor_feedback::msg::MultiPortMotorFeedback & msg)
  : msg_(msg)
  {}
  ::multi_port_motor_feedback::msg::MultiPortMotorFeedback can_data(::multi_port_motor_feedback::msg::MultiPortMotorFeedback::_can_data_type arg)
  {
    msg_.can_data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::multi_port_motor_feedback::msg::MultiPortMotorFeedback msg_;
};

class Init_MultiPortMotorFeedback_torque_nm
{
public:
  explicit Init_MultiPortMotorFeedback_torque_nm(::multi_port_motor_feedback::msg::MultiPortMotorFeedback & msg)
  : msg_(msg)
  {}
  Init_MultiPortMotorFeedback_can_data torque_nm(::multi_port_motor_feedback::msg::MultiPortMotorFeedback::_torque_nm_type arg)
  {
    msg_.torque_nm = std::move(arg);
    return Init_MultiPortMotorFeedback_can_data(msg_);
  }

private:
  ::multi_port_motor_feedback::msg::MultiPortMotorFeedback msg_;
};

class Init_MultiPortMotorFeedback_velocity_rad_s
{
public:
  explicit Init_MultiPortMotorFeedback_velocity_rad_s(::multi_port_motor_feedback::msg::MultiPortMotorFeedback & msg)
  : msg_(msg)
  {}
  Init_MultiPortMotorFeedback_torque_nm velocity_rad_s(::multi_port_motor_feedback::msg::MultiPortMotorFeedback::_velocity_rad_s_type arg)
  {
    msg_.velocity_rad_s = std::move(arg);
    return Init_MultiPortMotorFeedback_torque_nm(msg_);
  }

private:
  ::multi_port_motor_feedback::msg::MultiPortMotorFeedback msg_;
};

class Init_MultiPortMotorFeedback_position_rad
{
public:
  explicit Init_MultiPortMotorFeedback_position_rad(::multi_port_motor_feedback::msg::MultiPortMotorFeedback & msg)
  : msg_(msg)
  {}
  Init_MultiPortMotorFeedback_velocity_rad_s position_rad(::multi_port_motor_feedback::msg::MultiPortMotorFeedback::_position_rad_type arg)
  {
    msg_.position_rad = std::move(arg);
    return Init_MultiPortMotorFeedback_velocity_rad_s(msg_);
  }

private:
  ::multi_port_motor_feedback::msg::MultiPortMotorFeedback msg_;
};

class Init_MultiPortMotorFeedback_temp_rotor
{
public:
  explicit Init_MultiPortMotorFeedback_temp_rotor(::multi_port_motor_feedback::msg::MultiPortMotorFeedback & msg)
  : msg_(msg)
  {}
  Init_MultiPortMotorFeedback_position_rad temp_rotor(::multi_port_motor_feedback::msg::MultiPortMotorFeedback::_temp_rotor_type arg)
  {
    msg_.temp_rotor = std::move(arg);
    return Init_MultiPortMotorFeedback_position_rad(msg_);
  }

private:
  ::multi_port_motor_feedback::msg::MultiPortMotorFeedback msg_;
};

class Init_MultiPortMotorFeedback_temp_mos
{
public:
  explicit Init_MultiPortMotorFeedback_temp_mos(::multi_port_motor_feedback::msg::MultiPortMotorFeedback & msg)
  : msg_(msg)
  {}
  Init_MultiPortMotorFeedback_temp_rotor temp_mos(::multi_port_motor_feedback::msg::MultiPortMotorFeedback::_temp_mos_type arg)
  {
    msg_.temp_mos = std::move(arg);
    return Init_MultiPortMotorFeedback_temp_rotor(msg_);
  }

private:
  ::multi_port_motor_feedback::msg::MultiPortMotorFeedback msg_;
};

class Init_MultiPortMotorFeedback_torque
{
public:
  explicit Init_MultiPortMotorFeedback_torque(::multi_port_motor_feedback::msg::MultiPortMotorFeedback & msg)
  : msg_(msg)
  {}
  Init_MultiPortMotorFeedback_temp_mos torque(::multi_port_motor_feedback::msg::MultiPortMotorFeedback::_torque_type arg)
  {
    msg_.torque = std::move(arg);
    return Init_MultiPortMotorFeedback_temp_mos(msg_);
  }

private:
  ::multi_port_motor_feedback::msg::MultiPortMotorFeedback msg_;
};

class Init_MultiPortMotorFeedback_velocity
{
public:
  explicit Init_MultiPortMotorFeedback_velocity(::multi_port_motor_feedback::msg::MultiPortMotorFeedback & msg)
  : msg_(msg)
  {}
  Init_MultiPortMotorFeedback_torque velocity(::multi_port_motor_feedback::msg::MultiPortMotorFeedback::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_MultiPortMotorFeedback_torque(msg_);
  }

private:
  ::multi_port_motor_feedback::msg::MultiPortMotorFeedback msg_;
};

class Init_MultiPortMotorFeedback_position
{
public:
  explicit Init_MultiPortMotorFeedback_position(::multi_port_motor_feedback::msg::MultiPortMotorFeedback & msg)
  : msg_(msg)
  {}
  Init_MultiPortMotorFeedback_velocity position(::multi_port_motor_feedback::msg::MultiPortMotorFeedback::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_MultiPortMotorFeedback_velocity(msg_);
  }

private:
  ::multi_port_motor_feedback::msg::MultiPortMotorFeedback msg_;
};

class Init_MultiPortMotorFeedback_error
{
public:
  explicit Init_MultiPortMotorFeedback_error(::multi_port_motor_feedback::msg::MultiPortMotorFeedback & msg)
  : msg_(msg)
  {}
  Init_MultiPortMotorFeedback_position error(::multi_port_motor_feedback::msg::MultiPortMotorFeedback::_error_type arg)
  {
    msg_.error = std::move(arg);
    return Init_MultiPortMotorFeedback_position(msg_);
  }

private:
  ::multi_port_motor_feedback::msg::MultiPortMotorFeedback msg_;
};

class Init_MultiPortMotorFeedback_can_id
{
public:
  explicit Init_MultiPortMotorFeedback_can_id(::multi_port_motor_feedback::msg::MultiPortMotorFeedback & msg)
  : msg_(msg)
  {}
  Init_MultiPortMotorFeedback_error can_id(::multi_port_motor_feedback::msg::MultiPortMotorFeedback::_can_id_type arg)
  {
    msg_.can_id = std::move(arg);
    return Init_MultiPortMotorFeedback_error(msg_);
  }

private:
  ::multi_port_motor_feedback::msg::MultiPortMotorFeedback msg_;
};

class Init_MultiPortMotorFeedback_motor_id
{
public:
  explicit Init_MultiPortMotorFeedback_motor_id(::multi_port_motor_feedback::msg::MultiPortMotorFeedback & msg)
  : msg_(msg)
  {}
  Init_MultiPortMotorFeedback_can_id motor_id(::multi_port_motor_feedback::msg::MultiPortMotorFeedback::_motor_id_type arg)
  {
    msg_.motor_id = std::move(arg);
    return Init_MultiPortMotorFeedback_can_id(msg_);
  }

private:
  ::multi_port_motor_feedback::msg::MultiPortMotorFeedback msg_;
};

class Init_MultiPortMotorFeedback_header
{
public:
  Init_MultiPortMotorFeedback_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MultiPortMotorFeedback_motor_id header(::multi_port_motor_feedback::msg::MultiPortMotorFeedback::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MultiPortMotorFeedback_motor_id(msg_);
  }

private:
  ::multi_port_motor_feedback::msg::MultiPortMotorFeedback msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::multi_port_motor_feedback::msg::MultiPortMotorFeedback>()
{
  return multi_port_motor_feedback::msg::builder::Init_MultiPortMotorFeedback_header();
}

}  // namespace multi_port_motor_feedback

#endif  // MULTI_PORT_MOTOR_FEEDBACK__MSG__DETAIL__MULTI_PORT_MOTOR_FEEDBACK__BUILDER_HPP_
