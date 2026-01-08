// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from multi_port_motor_feedback:msg/MultiPortMotorFeedback.idl
// generated code does not contain a copyright notice

#ifndef MULTI_PORT_MOTOR_FEEDBACK__MSG__DETAIL__MULTI_PORT_MOTOR_FEEDBACK__TRAITS_HPP_
#define MULTI_PORT_MOTOR_FEEDBACK__MSG__DETAIL__MULTI_PORT_MOTOR_FEEDBACK__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "multi_port_motor_feedback/msg/detail/multi_port_motor_feedback__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace multi_port_motor_feedback
{

namespace msg
{

inline void to_flow_style_yaml(
  const MultiPortMotorFeedback & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: motor_id
  {
    out << "motor_id: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_id, out);
    out << ", ";
  }

  // member: can_id
  {
    out << "can_id: ";
    rosidl_generator_traits::value_to_yaml(msg.can_id, out);
    out << ", ";
  }

  // member: error
  {
    out << "error: ";
    rosidl_generator_traits::value_to_yaml(msg.error, out);
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

  // member: torque
  {
    out << "torque: ";
    rosidl_generator_traits::value_to_yaml(msg.torque, out);
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
    out << ", ";
  }

  // member: position_rad
  {
    out << "position_rad: ";
    rosidl_generator_traits::value_to_yaml(msg.position_rad, out);
    out << ", ";
  }

  // member: velocity_rad_s
  {
    out << "velocity_rad_s: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity_rad_s, out);
    out << ", ";
  }

  // member: torque_nm
  {
    out << "torque_nm: ";
    rosidl_generator_traits::value_to_yaml(msg.torque_nm, out);
    out << ", ";
  }

  // member: can_data
  {
    if (msg.can_data.size() == 0) {
      out << "can_data: []";
    } else {
      out << "can_data: [";
      size_t pending_items = msg.can_data.size();
      for (auto item : msg.can_data) {
        rosidl_generator_traits::value_to_yaml(item, out);
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
  const MultiPortMotorFeedback & msg,
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

  // member: motor_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "motor_id: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_id, out);
    out << "\n";
  }

  // member: can_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "can_id: ";
    rosidl_generator_traits::value_to_yaml(msg.can_id, out);
    out << "\n";
  }

  // member: error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error: ";
    rosidl_generator_traits::value_to_yaml(msg.error, out);
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

  // member: torque
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "torque: ";
    rosidl_generator_traits::value_to_yaml(msg.torque, out);
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

  // member: position_rad
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position_rad: ";
    rosidl_generator_traits::value_to_yaml(msg.position_rad, out);
    out << "\n";
  }

  // member: velocity_rad_s
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity_rad_s: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity_rad_s, out);
    out << "\n";
  }

  // member: torque_nm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "torque_nm: ";
    rosidl_generator_traits::value_to_yaml(msg.torque_nm, out);
    out << "\n";
  }

  // member: can_data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.can_data.size() == 0) {
      out << "can_data: []\n";
    } else {
      out << "can_data:\n";
      for (auto item : msg.can_data) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MultiPortMotorFeedback & msg, bool use_flow_style = false)
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
  const multi_port_motor_feedback::msg::MultiPortMotorFeedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  multi_port_motor_feedback::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use multi_port_motor_feedback::msg::to_yaml() instead")]]
inline std::string to_yaml(const multi_port_motor_feedback::msg::MultiPortMotorFeedback & msg)
{
  return multi_port_motor_feedback::msg::to_yaml(msg);
}

template<>
inline const char * data_type<multi_port_motor_feedback::msg::MultiPortMotorFeedback>()
{
  return "multi_port_motor_feedback::msg::MultiPortMotorFeedback";
}

template<>
inline const char * name<multi_port_motor_feedback::msg::MultiPortMotorFeedback>()
{
  return "multi_port_motor_feedback/msg/MultiPortMotorFeedback";
}

template<>
struct has_fixed_size<multi_port_motor_feedback::msg::MultiPortMotorFeedback>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<multi_port_motor_feedback::msg::MultiPortMotorFeedback>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<multi_port_motor_feedback::msg::MultiPortMotorFeedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MULTI_PORT_MOTOR_FEEDBACK__MSG__DETAIL__MULTI_PORT_MOTOR_FEEDBACK__TRAITS_HPP_
