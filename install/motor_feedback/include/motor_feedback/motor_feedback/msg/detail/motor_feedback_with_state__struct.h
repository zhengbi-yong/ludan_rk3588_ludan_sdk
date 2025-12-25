// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from motor_feedback:msg/MotorFeedbackWithState.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_FEEDBACK__MSG__DETAIL__MOTOR_FEEDBACK_WITH_STATE__STRUCT_H_
#define MOTOR_FEEDBACK__MSG__DETAIL__MOTOR_FEEDBACK_WITH_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/MotorFeedbackWithState in the package motor_feedback.
typedef struct motor_feedback__msg__MotorFeedbackWithState
{
  /// id,-1,或1-14
  int8_t id;
  /// Motor position(rad/s)
  float position;
  /// Motor velocitie (rad/s)
  float velocity;
  /// Motor effort (torque in Nm)
  float effort;
  /// Motor state (error codes or status codes)
  uint8_t state;
} motor_feedback__msg__MotorFeedbackWithState;

// Struct for a sequence of motor_feedback__msg__MotorFeedbackWithState.
typedef struct motor_feedback__msg__MotorFeedbackWithState__Sequence
{
  motor_feedback__msg__MotorFeedbackWithState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} motor_feedback__msg__MotorFeedbackWithState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MOTOR_FEEDBACK__MSG__DETAIL__MOTOR_FEEDBACK_WITH_STATE__STRUCT_H_
