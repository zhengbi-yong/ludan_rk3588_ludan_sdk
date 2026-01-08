// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from multi_port_motor_feedback:msg/LowState.idl
// generated code does not contain a copyright notice

#ifndef MULTI_PORT_MOTOR_FEEDBACK__MSG__DETAIL__LOW_STATE__STRUCT_H_
#define MULTI_PORT_MOTOR_FEEDBACK__MSG__DETAIL__LOW_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'motor_state'
#include "multi_port_motor_feedback/msg/detail/motor_state__struct.h"

/// Struct defined in msg/LowState in the package multi_port_motor_feedback.
/**
  * LowState.msg
  * Robot low-level state message
  * Motor distribution: 10 + 7 + 7 + 6 = 30 motors total
 */
typedef struct multi_port_motor_feedback__msg__LowState
{
  /// Header for timestamp
  std_msgs__msg__Header header;
  /// Motor states (30 motors)
  multi_port_motor_feedback__msg__MotorState motor_state[30];
} multi_port_motor_feedback__msg__LowState;

// Struct for a sequence of multi_port_motor_feedback__msg__LowState.
typedef struct multi_port_motor_feedback__msg__LowState__Sequence
{
  multi_port_motor_feedback__msg__LowState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} multi_port_motor_feedback__msg__LowState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MULTI_PORT_MOTOR_FEEDBACK__MSG__DETAIL__LOW_STATE__STRUCT_H_
