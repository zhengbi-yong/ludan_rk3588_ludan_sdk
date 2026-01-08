// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from multi_port_motor_feedback:msg/LowState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "multi_port_motor_feedback/msg/detail/low_state__rosidl_typesupport_introspection_c.h"
#include "multi_port_motor_feedback/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "multi_port_motor_feedback/msg/detail/low_state__functions.h"
#include "multi_port_motor_feedback/msg/detail/low_state__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `motor_state`
#include "multi_port_motor_feedback/msg/motor_state.h"
// Member `motor_state`
#include "multi_port_motor_feedback/msg/detail/motor_state__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void multi_port_motor_feedback__msg__LowState__rosidl_typesupport_introspection_c__LowState_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  multi_port_motor_feedback__msg__LowState__init(message_memory);
}

void multi_port_motor_feedback__msg__LowState__rosidl_typesupport_introspection_c__LowState_fini_function(void * message_memory)
{
  multi_port_motor_feedback__msg__LowState__fini(message_memory);
}

size_t multi_port_motor_feedback__msg__LowState__rosidl_typesupport_introspection_c__size_function__LowState__motor_state(
  const void * untyped_member)
{
  (void)untyped_member;
  return 30;
}

const void * multi_port_motor_feedback__msg__LowState__rosidl_typesupport_introspection_c__get_const_function__LowState__motor_state(
  const void * untyped_member, size_t index)
{
  const multi_port_motor_feedback__msg__MotorState * member =
    (const multi_port_motor_feedback__msg__MotorState *)(untyped_member);
  return &member[index];
}

void * multi_port_motor_feedback__msg__LowState__rosidl_typesupport_introspection_c__get_function__LowState__motor_state(
  void * untyped_member, size_t index)
{
  multi_port_motor_feedback__msg__MotorState * member =
    (multi_port_motor_feedback__msg__MotorState *)(untyped_member);
  return &member[index];
}

void multi_port_motor_feedback__msg__LowState__rosidl_typesupport_introspection_c__fetch_function__LowState__motor_state(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const multi_port_motor_feedback__msg__MotorState * item =
    ((const multi_port_motor_feedback__msg__MotorState *)
    multi_port_motor_feedback__msg__LowState__rosidl_typesupport_introspection_c__get_const_function__LowState__motor_state(untyped_member, index));
  multi_port_motor_feedback__msg__MotorState * value =
    (multi_port_motor_feedback__msg__MotorState *)(untyped_value);
  *value = *item;
}

void multi_port_motor_feedback__msg__LowState__rosidl_typesupport_introspection_c__assign_function__LowState__motor_state(
  void * untyped_member, size_t index, const void * untyped_value)
{
  multi_port_motor_feedback__msg__MotorState * item =
    ((multi_port_motor_feedback__msg__MotorState *)
    multi_port_motor_feedback__msg__LowState__rosidl_typesupport_introspection_c__get_function__LowState__motor_state(untyped_member, index));
  const multi_port_motor_feedback__msg__MotorState * value =
    (const multi_port_motor_feedback__msg__MotorState *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember multi_port_motor_feedback__msg__LowState__rosidl_typesupport_introspection_c__LowState_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(multi_port_motor_feedback__msg__LowState, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "motor_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    30,  // array size
    false,  // is upper bound
    offsetof(multi_port_motor_feedback__msg__LowState, motor_state),  // bytes offset in struct
    NULL,  // default value
    multi_port_motor_feedback__msg__LowState__rosidl_typesupport_introspection_c__size_function__LowState__motor_state,  // size() function pointer
    multi_port_motor_feedback__msg__LowState__rosidl_typesupport_introspection_c__get_const_function__LowState__motor_state,  // get_const(index) function pointer
    multi_port_motor_feedback__msg__LowState__rosidl_typesupport_introspection_c__get_function__LowState__motor_state,  // get(index) function pointer
    multi_port_motor_feedback__msg__LowState__rosidl_typesupport_introspection_c__fetch_function__LowState__motor_state,  // fetch(index, &value) function pointer
    multi_port_motor_feedback__msg__LowState__rosidl_typesupport_introspection_c__assign_function__LowState__motor_state,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers multi_port_motor_feedback__msg__LowState__rosidl_typesupport_introspection_c__LowState_message_members = {
  "multi_port_motor_feedback__msg",  // message namespace
  "LowState",  // message name
  2,  // number of fields
  sizeof(multi_port_motor_feedback__msg__LowState),
  multi_port_motor_feedback__msg__LowState__rosidl_typesupport_introspection_c__LowState_message_member_array,  // message members
  multi_port_motor_feedback__msg__LowState__rosidl_typesupport_introspection_c__LowState_init_function,  // function to initialize message memory (memory has to be allocated)
  multi_port_motor_feedback__msg__LowState__rosidl_typesupport_introspection_c__LowState_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t multi_port_motor_feedback__msg__LowState__rosidl_typesupport_introspection_c__LowState_message_type_support_handle = {
  0,
  &multi_port_motor_feedback__msg__LowState__rosidl_typesupport_introspection_c__LowState_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_multi_port_motor_feedback
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, multi_port_motor_feedback, msg, LowState)() {
  multi_port_motor_feedback__msg__LowState__rosidl_typesupport_introspection_c__LowState_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  multi_port_motor_feedback__msg__LowState__rosidl_typesupport_introspection_c__LowState_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, multi_port_motor_feedback, msg, MotorState)();
  if (!multi_port_motor_feedback__msg__LowState__rosidl_typesupport_introspection_c__LowState_message_type_support_handle.typesupport_identifier) {
    multi_port_motor_feedback__msg__LowState__rosidl_typesupport_introspection_c__LowState_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &multi_port_motor_feedback__msg__LowState__rosidl_typesupport_introspection_c__LowState_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
