// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from motor_feedback:msg/MotorFeedback.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "motor_feedback/msg/detail/motor_feedback__rosidl_typesupport_introspection_c.h"
#include "motor_feedback/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "motor_feedback/msg/detail/motor_feedback__functions.h"
#include "motor_feedback/msg/detail/motor_feedback__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void motor_feedback__msg__MotorFeedback__rosidl_typesupport_introspection_c__MotorFeedback_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  motor_feedback__msg__MotorFeedback__init(message_memory);
}

void motor_feedback__msg__MotorFeedback__rosidl_typesupport_introspection_c__MotorFeedback_fini_function(void * message_memory)
{
  motor_feedback__msg__MotorFeedback__fini(message_memory);
}

size_t motor_feedback__msg__MotorFeedback__rosidl_typesupport_introspection_c__size_function__MotorFeedback__can_data(
  const void * untyped_member)
{
  (void)untyped_member;
  return 8;
}

const void * motor_feedback__msg__MotorFeedback__rosidl_typesupport_introspection_c__get_const_function__MotorFeedback__can_data(
  const void * untyped_member, size_t index)
{
  const uint8_t * member =
    (const uint8_t *)(untyped_member);
  return &member[index];
}

void * motor_feedback__msg__MotorFeedback__rosidl_typesupport_introspection_c__get_function__MotorFeedback__can_data(
  void * untyped_member, size_t index)
{
  uint8_t * member =
    (uint8_t *)(untyped_member);
  return &member[index];
}

void motor_feedback__msg__MotorFeedback__rosidl_typesupport_introspection_c__fetch_function__MotorFeedback__can_data(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    motor_feedback__msg__MotorFeedback__rosidl_typesupport_introspection_c__get_const_function__MotorFeedback__can_data(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void motor_feedback__msg__MotorFeedback__rosidl_typesupport_introspection_c__assign_function__MotorFeedback__can_data(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    motor_feedback__msg__MotorFeedback__rosidl_typesupport_introspection_c__get_function__MotorFeedback__can_data(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember motor_feedback__msg__MotorFeedback__rosidl_typesupport_introspection_c__MotorFeedback_message_member_array[13] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_feedback__msg__MotorFeedback, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "motor_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_feedback__msg__MotorFeedback, motor_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "can_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_feedback__msg__MotorFeedback, can_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "error",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_feedback__msg__MotorFeedback, error),  // bytes offset in struct
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
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_feedback__msg__MotorFeedback, position),  // bytes offset in struct
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
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_feedback__msg__MotorFeedback, velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "torque",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_feedback__msg__MotorFeedback, torque),  // bytes offset in struct
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
    offsetof(motor_feedback__msg__MotorFeedback, temp_mos),  // bytes offset in struct
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
    offsetof(motor_feedback__msg__MotorFeedback, temp_rotor),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "position_rad",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_feedback__msg__MotorFeedback, position_rad),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "velocity_rad_s",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_feedback__msg__MotorFeedback, velocity_rad_s),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "torque_nm",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_feedback__msg__MotorFeedback, torque_nm),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "can_data",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    8,  // array size
    false,  // is upper bound
    offsetof(motor_feedback__msg__MotorFeedback, can_data),  // bytes offset in struct
    NULL,  // default value
    motor_feedback__msg__MotorFeedback__rosidl_typesupport_introspection_c__size_function__MotorFeedback__can_data,  // size() function pointer
    motor_feedback__msg__MotorFeedback__rosidl_typesupport_introspection_c__get_const_function__MotorFeedback__can_data,  // get_const(index) function pointer
    motor_feedback__msg__MotorFeedback__rosidl_typesupport_introspection_c__get_function__MotorFeedback__can_data,  // get(index) function pointer
    motor_feedback__msg__MotorFeedback__rosidl_typesupport_introspection_c__fetch_function__MotorFeedback__can_data,  // fetch(index, &value) function pointer
    motor_feedback__msg__MotorFeedback__rosidl_typesupport_introspection_c__assign_function__MotorFeedback__can_data,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers motor_feedback__msg__MotorFeedback__rosidl_typesupport_introspection_c__MotorFeedback_message_members = {
  "motor_feedback__msg",  // message namespace
  "MotorFeedback",  // message name
  13,  // number of fields
  sizeof(motor_feedback__msg__MotorFeedback),
  motor_feedback__msg__MotorFeedback__rosidl_typesupport_introspection_c__MotorFeedback_message_member_array,  // message members
  motor_feedback__msg__MotorFeedback__rosidl_typesupport_introspection_c__MotorFeedback_init_function,  // function to initialize message memory (memory has to be allocated)
  motor_feedback__msg__MotorFeedback__rosidl_typesupport_introspection_c__MotorFeedback_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t motor_feedback__msg__MotorFeedback__rosidl_typesupport_introspection_c__MotorFeedback_message_type_support_handle = {
  0,
  &motor_feedback__msg__MotorFeedback__rosidl_typesupport_introspection_c__MotorFeedback_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_motor_feedback
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, motor_feedback, msg, MotorFeedback)() {
  motor_feedback__msg__MotorFeedback__rosidl_typesupport_introspection_c__MotorFeedback_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!motor_feedback__msg__MotorFeedback__rosidl_typesupport_introspection_c__MotorFeedback_message_type_support_handle.typesupport_identifier) {
    motor_feedback__msg__MotorFeedback__rosidl_typesupport_introspection_c__MotorFeedback_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &motor_feedback__msg__MotorFeedback__rosidl_typesupport_introspection_c__MotorFeedback_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
