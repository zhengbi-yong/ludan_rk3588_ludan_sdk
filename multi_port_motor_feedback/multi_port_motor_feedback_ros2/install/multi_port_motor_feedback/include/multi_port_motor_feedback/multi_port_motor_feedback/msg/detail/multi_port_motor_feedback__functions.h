// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from multi_port_motor_feedback:msg/MultiPortMotorFeedback.idl
// generated code does not contain a copyright notice

#ifndef MULTI_PORT_MOTOR_FEEDBACK__MSG__DETAIL__MULTI_PORT_MOTOR_FEEDBACK__FUNCTIONS_H_
#define MULTI_PORT_MOTOR_FEEDBACK__MSG__DETAIL__MULTI_PORT_MOTOR_FEEDBACK__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "multi_port_motor_feedback/msg/rosidl_generator_c__visibility_control.h"

#include "multi_port_motor_feedback/msg/detail/multi_port_motor_feedback__struct.h"

/// Initialize msg/MultiPortMotorFeedback message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * multi_port_motor_feedback__msg__MultiPortMotorFeedback
 * )) before or use
 * multi_port_motor_feedback__msg__MultiPortMotorFeedback__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_multi_port_motor_feedback
bool
multi_port_motor_feedback__msg__MultiPortMotorFeedback__init(multi_port_motor_feedback__msg__MultiPortMotorFeedback * msg);

/// Finalize msg/MultiPortMotorFeedback message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_multi_port_motor_feedback
void
multi_port_motor_feedback__msg__MultiPortMotorFeedback__fini(multi_port_motor_feedback__msg__MultiPortMotorFeedback * msg);

/// Create msg/MultiPortMotorFeedback message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * multi_port_motor_feedback__msg__MultiPortMotorFeedback__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_multi_port_motor_feedback
multi_port_motor_feedback__msg__MultiPortMotorFeedback *
multi_port_motor_feedback__msg__MultiPortMotorFeedback__create();

/// Destroy msg/MultiPortMotorFeedback message.
/**
 * It calls
 * multi_port_motor_feedback__msg__MultiPortMotorFeedback__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_multi_port_motor_feedback
void
multi_port_motor_feedback__msg__MultiPortMotorFeedback__destroy(multi_port_motor_feedback__msg__MultiPortMotorFeedback * msg);

/// Check for msg/MultiPortMotorFeedback message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_multi_port_motor_feedback
bool
multi_port_motor_feedback__msg__MultiPortMotorFeedback__are_equal(const multi_port_motor_feedback__msg__MultiPortMotorFeedback * lhs, const multi_port_motor_feedback__msg__MultiPortMotorFeedback * rhs);

/// Copy a msg/MultiPortMotorFeedback message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_multi_port_motor_feedback
bool
multi_port_motor_feedback__msg__MultiPortMotorFeedback__copy(
  const multi_port_motor_feedback__msg__MultiPortMotorFeedback * input,
  multi_port_motor_feedback__msg__MultiPortMotorFeedback * output);

/// Initialize array of msg/MultiPortMotorFeedback messages.
/**
 * It allocates the memory for the number of elements and calls
 * multi_port_motor_feedback__msg__MultiPortMotorFeedback__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_multi_port_motor_feedback
bool
multi_port_motor_feedback__msg__MultiPortMotorFeedback__Sequence__init(multi_port_motor_feedback__msg__MultiPortMotorFeedback__Sequence * array, size_t size);

/// Finalize array of msg/MultiPortMotorFeedback messages.
/**
 * It calls
 * multi_port_motor_feedback__msg__MultiPortMotorFeedback__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_multi_port_motor_feedback
void
multi_port_motor_feedback__msg__MultiPortMotorFeedback__Sequence__fini(multi_port_motor_feedback__msg__MultiPortMotorFeedback__Sequence * array);

/// Create array of msg/MultiPortMotorFeedback messages.
/**
 * It allocates the memory for the array and calls
 * multi_port_motor_feedback__msg__MultiPortMotorFeedback__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_multi_port_motor_feedback
multi_port_motor_feedback__msg__MultiPortMotorFeedback__Sequence *
multi_port_motor_feedback__msg__MultiPortMotorFeedback__Sequence__create(size_t size);

/// Destroy array of msg/MultiPortMotorFeedback messages.
/**
 * It calls
 * multi_port_motor_feedback__msg__MultiPortMotorFeedback__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_multi_port_motor_feedback
void
multi_port_motor_feedback__msg__MultiPortMotorFeedback__Sequence__destroy(multi_port_motor_feedback__msg__MultiPortMotorFeedback__Sequence * array);

/// Check for msg/MultiPortMotorFeedback message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_multi_port_motor_feedback
bool
multi_port_motor_feedback__msg__MultiPortMotorFeedback__Sequence__are_equal(const multi_port_motor_feedback__msg__MultiPortMotorFeedback__Sequence * lhs, const multi_port_motor_feedback__msg__MultiPortMotorFeedback__Sequence * rhs);

/// Copy an array of msg/MultiPortMotorFeedback messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_multi_port_motor_feedback
bool
multi_port_motor_feedback__msg__MultiPortMotorFeedback__Sequence__copy(
  const multi_port_motor_feedback__msg__MultiPortMotorFeedback__Sequence * input,
  multi_port_motor_feedback__msg__MultiPortMotorFeedback__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // MULTI_PORT_MOTOR_FEEDBACK__MSG__DETAIL__MULTI_PORT_MOTOR_FEEDBACK__FUNCTIONS_H_
