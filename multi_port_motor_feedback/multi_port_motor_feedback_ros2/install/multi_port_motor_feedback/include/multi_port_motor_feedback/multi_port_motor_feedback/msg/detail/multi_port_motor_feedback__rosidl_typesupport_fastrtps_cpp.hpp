// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from multi_port_motor_feedback:msg/MultiPortMotorFeedback.idl
// generated code does not contain a copyright notice

#ifndef MULTI_PORT_MOTOR_FEEDBACK__MSG__DETAIL__MULTI_PORT_MOTOR_FEEDBACK__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define MULTI_PORT_MOTOR_FEEDBACK__MSG__DETAIL__MULTI_PORT_MOTOR_FEEDBACK__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "multi_port_motor_feedback/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "multi_port_motor_feedback/msg/detail/multi_port_motor_feedback__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace multi_port_motor_feedback
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_multi_port_motor_feedback
cdr_serialize(
  const multi_port_motor_feedback::msg::MultiPortMotorFeedback & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_multi_port_motor_feedback
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  multi_port_motor_feedback::msg::MultiPortMotorFeedback & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_multi_port_motor_feedback
get_serialized_size(
  const multi_port_motor_feedback::msg::MultiPortMotorFeedback & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_multi_port_motor_feedback
max_serialized_size_MultiPortMotorFeedback(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace multi_port_motor_feedback

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_multi_port_motor_feedback
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, multi_port_motor_feedback, msg, MultiPortMotorFeedback)();

#ifdef __cplusplus
}
#endif

#endif  // MULTI_PORT_MOTOR_FEEDBACK__MSG__DETAIL__MULTI_PORT_MOTOR_FEEDBACK__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
