// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from motor_feedback:msg/MotorFeedbackWithState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "motor_feedback/msg/detail/motor_feedback_with_state__rosidl_typesupport_introspection_c.h"
#include "motor_feedback/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "motor_feedback/msg/detail/motor_feedback_with_state__functions.h"
#include "motor_feedback/msg/detail/motor_feedback_with_state__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void motor_feedback__msg__MotorFeedbackWithState__rosidl_typesupport_introspection_c__MotorFeedbackWithState_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  motor_feedback__msg__MotorFeedbackWithState__init(message_memory);
}

void motor_feedback__msg__MotorFeedbackWithState__rosidl_typesupport_introspection_c__MotorFeedbackWithState_fini_function(void * message_memory)
{
  motor_feedback__msg__MotorFeedbackWithState__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember motor_feedback__msg__MotorFeedbackWithState__rosidl_typesupport_introspection_c__MotorFeedbackWithState_message_member_array[5] = {
  {
    "id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_feedback__msg__MotorFeedbackWithState, id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_feedback__msg__MotorFeedbackWithState, position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_feedback__msg__MotorFeedbackWithState, velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "effort",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_feedback__msg__MotorFeedbackWithState, effort),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_feedback__msg__MotorFeedbackWithState, state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers motor_feedback__msg__MotorFeedbackWithState__rosidl_typesupport_introspection_c__MotorFeedbackWithState_message_members = {
  "motor_feedback__msg",  // message namespace
  "MotorFeedbackWithState",  // message name
  5,  // number of fields
  sizeof(motor_feedback__msg__MotorFeedbackWithState),
  motor_feedback__msg__MotorFeedbackWithState__rosidl_typesupport_introspection_c__MotorFeedbackWithState_message_member_array,  // message members
  motor_feedback__msg__MotorFeedbackWithState__rosidl_typesupport_introspection_c__MotorFeedbackWithState_init_function,  // function to initialize message memory (memory has to be allocated)
  motor_feedback__msg__MotorFeedbackWithState__rosidl_typesupport_introspection_c__MotorFeedbackWithState_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t motor_feedback__msg__MotorFeedbackWithState__rosidl_typesupport_introspection_c__MotorFeedbackWithState_message_type_support_handle = {
  0,
  &motor_feedback__msg__MotorFeedbackWithState__rosidl_typesupport_introspection_c__MotorFeedbackWithState_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_motor_feedback
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, motor_feedback, msg, MotorFeedbackWithState)() {
  if (!motor_feedback__msg__MotorFeedbackWithState__rosidl_typesupport_introspection_c__MotorFeedbackWithState_message_type_support_handle.typesupport_identifier) {
    motor_feedback__msg__MotorFeedbackWithState__rosidl_typesupport_introspection_c__MotorFeedbackWithState_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &motor_feedback__msg__MotorFeedbackWithState__rosidl_typesupport_introspection_c__MotorFeedbackWithState_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
