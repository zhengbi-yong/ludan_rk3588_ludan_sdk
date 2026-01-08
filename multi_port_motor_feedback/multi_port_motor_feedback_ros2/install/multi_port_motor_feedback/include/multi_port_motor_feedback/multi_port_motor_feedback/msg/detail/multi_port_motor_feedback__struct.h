// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from multi_port_motor_feedback:msg/MultiPortMotorFeedback.idl
// generated code does not contain a copyright notice

#ifndef MULTI_PORT_MOTOR_FEEDBACK__MSG__DETAIL__MULTI_PORT_MOTOR_FEEDBACK__STRUCT_H_
#define MULTI_PORT_MOTOR_FEEDBACK__MSG__DETAIL__MULTI_PORT_MOTOR_FEEDBACK__STRUCT_H_

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

/// Struct defined in msg/MultiPortMotorFeedback in the package multi_port_motor_feedback.
/**
  * MotorFeedback.msg
  * DM Motor feedback message from CAN bus
  * Compatible with motor_controller_with_enable
 */
typedef struct multi_port_motor_feedback__msg__MultiPortMotorFeedback
{
  /// Header for timestamp and frame ID
  std_msgs__msg__Header header;
  /// Motor identification
  /// Motor ID (1-30)
  int32_t motor_id;
  /// Raw CAN ID
  int32_t can_id;
  /// Motor mode/error (DM format)
  /// Error code (4 bits from D upper)
  int32_t error;
  /// 0x0: OK/Disabled
  /// 0x1: Enabled
  /// 0x8: Over-voltage
  /// 0x9: Under-voltage
  /// 0xA: Over-current
  /// 0xB: MOS Over-temp
  /// 0xC: Coil Over-temp
  /// 0xD: Comms Lost
  /// 0xE: Overload
  /// Raw measurements (DM format, unscaled)
  /// Position raw value (16-bit signed)
  int16_t position;
  /// Velocity raw value (12-bit signed)
  int16_t velocity;
  /// Torque raw value (12-bit signed)
  int16_t torque;
  /// MOS temperature
  int8_t temp_mos;
  /// Rotor temperature
  int8_t temp_rotor;
  /// Scaled measurements (for convenience)
  /// Position in radians
  float position_rad;
  /// Velocity in rad/s
  float velocity_rad_s;
  /// Torque in Nm
  float torque_nm;
  /// Raw CAN data for debugging
  /// Raw CAN frame data (8 bytes)
  uint8_t can_data[8];
} multi_port_motor_feedback__msg__MultiPortMotorFeedback;

// Struct for a sequence of multi_port_motor_feedback__msg__MultiPortMotorFeedback.
typedef struct multi_port_motor_feedback__msg__MultiPortMotorFeedback__Sequence
{
  multi_port_motor_feedback__msg__MultiPortMotorFeedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} multi_port_motor_feedback__msg__MultiPortMotorFeedback__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MULTI_PORT_MOTOR_FEEDBACK__MSG__DETAIL__MULTI_PORT_MOTOR_FEEDBACK__STRUCT_H_
