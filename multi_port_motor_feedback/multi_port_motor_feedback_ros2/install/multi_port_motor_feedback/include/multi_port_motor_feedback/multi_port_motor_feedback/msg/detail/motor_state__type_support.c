// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from multi_port_motor_feedback:msg/MotorState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "multi_port_motor_feedback/msg/detail/motor_state__rosidl_typesupport_introspection_c.h"
#include "multi_port_motor_feedback/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "multi_port_motor_feedback/msg/detail/motor_state__functions.h"
#include "multi_port_motor_feedback/msg/detail/motor_state__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void multi_port_motor_feedback__msg__MotorState__rosidl_typesupport_introspection_c__MotorState_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  multi_port_motor_feedback__msg__MotorState__init(message_memory);
}

void multi_port_motor_feedback__msg__MotorState__rosidl_typesupport_introspection_c__MotorState_fini_function(void * message_memory)
{
  multi_port_motor_feedback__msg__MotorState__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember multi_port_motor_feedback__msg__MotorState__rosidl_typesupport_introspection_c__MotorState_message_member_array[7] = {
  {
    "state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(multi_port_motor_feedback__msg__MotorState, state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(multi_port_motor_feedback__msg__MotorState, id),  // bytes offset in struct
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
    offsetof(multi_port_motor_feedback__msg__MotorState, position),  // bytes offset in struct
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
    offsetof(multi_port_motor_feedback__msg__MotorState, velocity),  // bytes offset in struct
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
    offsetof(multi_port_motor_feedback__msg__MotorState, effort),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "temp_mos",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(multi_port_motor_feedback__msg__MotorState, temp_mos),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "temp_rotor",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(multi_port_motor_feedback__msg__MotorState, temp_rotor),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers multi_port_motor_feedback__msg__MotorState__rosidl_typesupport_introspection_c__MotorState_message_members = {
  "multi_port_motor_feedback__msg",  // message namespace
  "MotorState",  // message name
  7,  // number of fields
  sizeof(multi_port_motor_feedback__msg__MotorState),
  multi_port_motor_feedback__msg__MotorState__rosidl_typesupport_introspection_c__MotorState_message_member_array,  // message members
  multi_port_motor_feedback__msg__MotorState__rosidl_typesupport_introspection_c__MotorState_init_function,  // function to initialize message memory (memory has to be allocated)
  multi_port_motor_feedback__msg__MotorState__rosidl_typesupport_introspection_c__MotorState_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t multi_port_motor_feedback__msg__MotorState__rosidl_typesupport_introspection_c__MotorState_message_type_support_handle = {
  0,
  &multi_port_motor_feedback__msg__MotorState__rosidl_typesupport_introspection_c__MotorState_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_multi_port_motor_feedback
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, multi_port_motor_feedback, msg, MotorState)() {
  if (!multi_port_motor_feedback__msg__MotorState__rosidl_typesupport_introspection_c__MotorState_message_type_support_handle.typesupport_identifier) {
    multi_port_motor_feedback__msg__MotorState__rosidl_typesupport_introspection_c__MotorState_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &multi_port_motor_feedback__msg__MotorState__rosidl_typesupport_introspection_c__MotorState_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
