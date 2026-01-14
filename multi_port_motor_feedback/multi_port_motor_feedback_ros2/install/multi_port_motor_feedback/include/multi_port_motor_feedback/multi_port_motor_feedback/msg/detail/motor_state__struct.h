// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from multi_port_motor_feedback:msg/MotorState.idl
// generated code does not contain a copyright notice

#ifndef MULTI_PORT_MOTOR_FEEDBACK__MSG__DETAIL__MOTOR_STATE__STRUCT_H_
#define MULTI_PORT_MOTOR_FEEDBACK__MSG__DETAIL__MOTOR_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/MotorState in the package multi_port_motor_feedback.
/**
  * MotorState.msg
  * Single motor state message
 */
typedef struct multi_port_motor_feedback__msg__MotorState
{
  /// Data validity flag
  /// True if motor data has been received at least once
  bool valid;
  /// Error code (DM motor format)
  uint8_t state;
  /// Scaled measurements
  /// Motor ID, 1-30
  int8_t id;
  float position;
  float velocity;
  /// Torque in Nm
  float effort;
  int8_t temp_mos;
  int8_t temp_rotor;
} multi_port_motor_feedback__msg__MotorState;

// Struct for a sequence of multi_port_motor_feedback__msg__MotorState.
typedef struct multi_port_motor_feedback__msg__MotorState__Sequence
{
  multi_port_motor_feedback__msg__MotorState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} multi_port_motor_feedback__msg__MotorState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MULTI_PORT_MOTOR_FEEDBACK__MSG__DETAIL__MOTOR_STATE__STRUCT_H_
