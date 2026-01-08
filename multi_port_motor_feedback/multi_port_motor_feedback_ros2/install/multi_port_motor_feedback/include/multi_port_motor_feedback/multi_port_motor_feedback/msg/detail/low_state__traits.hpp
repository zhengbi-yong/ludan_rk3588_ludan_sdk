// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from multi_port_motor_feedback:msg/LowState.idl
// generated code does not contain a copyright notice

#ifndef MULTI_PORT_MOTOR_FEEDBACK__MSG__DETAIL__LOW_STATE__TRAITS_HPP_
#define MULTI_PORT_MOTOR_FEEDBACK__MSG__DETAIL__LOW_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "multi_port_motor_feedback/msg/detail/low_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'motor_state'
#include "multi_port_motor_feedback/msg/detail/motor_state__traits.hpp"

namespace multi_port_motor_feedback
{

namespace msg
{

inline void to_flow_style_yaml(
  const LowState & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: motor_state
  {
    if (msg.motor_state.size() == 0) {
      out << "motor_state: []";
    } else {
      out << "motor_state: [";
      size_t pending_items = msg.motor_state.size();
      for (auto item : msg.motor_state) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LowState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: motor_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.motor_state.size() == 0) {
      out << "motor_state: []\n";
    } else {
      out << "motor_state:\n";
      for (auto item : msg.motor_state) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LowState & msg, bool use_flow_style = false)
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
  const multi_port_motor_feedback::msg::LowState & msg,
  std::ostream & out, size_t indentation = 0)
{
  multi_port_motor_feedback::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use multi_port_motor_feedback::msg::to_yaml() instead")]]
inline std::string to_yaml(const multi_port_motor_feedback::msg::LowState & msg)
{
  return multi_port_motor_feedback::msg::to_yaml(msg);
}

template<>
inline const char * data_type<multi_port_motor_feedback::msg::LowState>()
{
  return "multi_port_motor_feedback::msg::LowState";
}

template<>
inline const char * name<multi_port_motor_feedback::msg::LowState>()
{
  return "multi_port_motor_feedback/msg/LowState";
}

template<>
struct has_fixed_size<multi_port_motor_feedback::msg::LowState>
  : std::integral_constant<bool, has_fixed_size<multi_port_motor_feedback::msg::MotorState>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<multi_port_motor_feedback::msg::LowState>
  : std::integral_constant<bool, has_bounded_size<multi_port_motor_feedback::msg::MotorState>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<multi_port_motor_feedback::msg::LowState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MULTI_PORT_MOTOR_FEEDBACK__MSG__DETAIL__LOW_STATE__TRAITS_HPP_
