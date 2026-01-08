// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from multi_port_motor_feedback:msg/MotorState.idl
// generated code does not contain a copyright notice

#ifndef MULTI_PORT_MOTOR_FEEDBACK__MSG__DETAIL__MOTOR_STATE__TRAITS_HPP_
#define MULTI_PORT_MOTOR_FEEDBACK__MSG__DETAIL__MOTOR_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "multi_port_motor_feedback/msg/detail/motor_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace multi_port_motor_feedback
{

namespace msg
{

inline void to_flow_style_yaml(
  const MotorState & msg,
  std::ostream & out)
{
  out << "{";
  // member: state
  {
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << ", ";
  }

  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: position
  {
    out << "position: ";
    rosidl_generator_traits::value_to_yaml(msg.position, out);
    out << ", ";
  }

  // member: velocity
  {
    out << "velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity, out);
    out << ", ";
  }

  // member: effort
  {
    out << "effort: ";
    rosidl_generator_traits::value_to_yaml(msg.effort, out);
    out << ", ";
  }

  // member: temp_mos
  {
    out << "temp_mos: ";
    rosidl_generator_traits::value_to_yaml(msg.temp_mos, out);
    out << ", ";
  }

  // member: temp_rotor
  {
    out << "temp_rotor: ";
    rosidl_generator_traits::value_to_yaml(msg.temp_rotor, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MotorState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << "\n";
  }

  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << "\n";
  }

  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position: ";
    rosidl_generator_traits::value_to_yaml(msg.position, out);
    out << "\n";
  }

  // member: velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity, out);
    out << "\n";
  }

  // member: effort
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "effort: ";
    rosidl_generator_traits::value_to_yaml(msg.effort, out);
    out << "\n";
  }

  // member: temp_mos
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "temp_mos: ";
    rosidl_generator_traits::value_to_yaml(msg.temp_mos, out);
    out << "\n";
  }

  // member: temp_rotor
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "temp_rotor: ";
    rosidl_generator_traits::value_to_yaml(msg.temp_rotor, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MotorState & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace multi_port_motor_feedback

namespace rosidl_generator_traits
{

[[deprecated("use multi_port_motor_feedback::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const multi_port_motor_feedback::msg::MotorState & msg,
  std::ostream & out, size_t indentation = 0)
{
  multi_port_motor_feedback::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use multi_port_motor_feedback::msg::to_yaml() instead")]]
inline std::string to_yaml(const multi_port_motor_feedback::msg::MotorState & msg)
{
  return multi_port_motor_feedback::msg::to_yaml(msg);
}

template<>
inline const char * data_type<multi_port_motor_feedback::msg::MotorState>()
{
  return "multi_port_motor_feedback::msg::MotorState";
}

template<>
inline const char * name<multi_port_motor_feedback::msg::MotorState>()
{
  return "multi_port_motor_feedback/msg/MotorState";
}

template<>
struct has_fixed_size<multi_port_motor_feedback::msg::MotorState>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<multi_port_motor_feedback::msg::MotorState>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<multi_port_motor_feedback::msg::MotorState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MULTI_PORT_MOTOR_FEEDBACK__MSG__DETAIL__MOTOR_STATE__TRAITS_HPP_
