// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from motor_feedback:msg/MotorFeedbackWithState.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_FEEDBACK__MSG__DETAIL__MOTOR_FEEDBACK_WITH_STATE__TRAITS_HPP_
#define MOTOR_FEEDBACK__MSG__DETAIL__MOTOR_FEEDBACK_WITH_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "motor_feedback/msg/detail/motor_feedback_with_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace motor_feedback
{

namespace msg
{

inline void to_flow_style_yaml(
  const MotorFeedbackWithState & msg,
  std::ostream & out)
{
  out << "{";
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

  // member: state
  {
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MotorFeedbackWithState & msg,
  std::ostream & out, size_t indentation = 0)
{
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

  // member: state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MotorFeedbackWithState & msg, bool use_flow_style = false)
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

}  // namespace motor_feedback

namespace rosidl_generator_traits
{

[[deprecated("use motor_feedback::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const motor_feedback::msg::MotorFeedbackWithState & msg,
  std::ostream & out, size_t indentation = 0)
{
  motor_feedback::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use motor_feedback::msg::to_yaml() instead")]]
inline std::string to_yaml(const motor_feedback::msg::MotorFeedbackWithState & msg)
{
  return motor_feedback::msg::to_yaml(msg);
}

template<>
inline const char * data_type<motor_feedback::msg::MotorFeedbackWithState>()
{
  return "motor_feedback::msg::MotorFeedbackWithState";
}

template<>
inline const char * name<motor_feedback::msg::MotorFeedbackWithState>()
{
  return "motor_feedback/msg/MotorFeedbackWithState";
}

template<>
struct has_fixed_size<motor_feedback::msg::MotorFeedbackWithState>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<motor_feedback::msg::MotorFeedbackWithState>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<motor_feedback::msg::MotorFeedbackWithState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MOTOR_FEEDBACK__MSG__DETAIL__MOTOR_FEEDBACK_WITH_STATE__TRAITS_HPP_
