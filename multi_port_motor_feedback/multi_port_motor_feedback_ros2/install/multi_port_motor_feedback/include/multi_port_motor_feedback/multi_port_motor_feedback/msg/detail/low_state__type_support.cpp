// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from multi_port_motor_feedback:msg/LowState.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "multi_port_motor_feedback/msg/detail/low_state__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace multi_port_motor_feedback
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void LowState_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) multi_port_motor_feedback::msg::LowState(_init);
}

void LowState_fini_function(void * message_memory)
{
  auto typed_message = static_cast<multi_port_motor_feedback::msg::LowState *>(message_memory);
  typed_message->~LowState();
}

size_t size_function__LowState__motor_state(const void * untyped_member)
{
  (void)untyped_member;
  return 30;
}

const void * get_const_function__LowState__motor_state(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<multi_port_motor_feedback::msg::MotorState, 30> *>(untyped_member);
  return &member[index];
}

void * get_function__LowState__motor_state(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<multi_port_motor_feedback::msg::MotorState, 30> *>(untyped_member);
  return &member[index];
}

void fetch_function__LowState__motor_state(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const multi_port_motor_feedback::msg::MotorState *>(
    get_const_function__LowState__motor_state(untyped_member, index));
  auto & value = *reinterpret_cast<multi_port_motor_feedback::msg::MotorState *>(untyped_value);
  value = item;
}

void assign_function__LowState__motor_state(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<multi_port_motor_feedback::msg::MotorState *>(
    get_function__LowState__motor_state(untyped_member, index));
  const auto & value = *reinterpret_cast<const multi_port_motor_feedback::msg::MotorState *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember LowState_message_member_array[2] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(multi_port_motor_feedback::msg::LowState, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "motor_state",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<multi_port_motor_feedback::msg::MotorState>(),  // members of sub message
    true,  // is array
    30,  // array size
    false,  // is upper bound
    offsetof(multi_port_motor_feedback::msg::LowState, motor_state),  // bytes offset in struct
    nullptr,  // default value
    size_function__LowState__motor_state,  // size() function pointer
    get_const_function__LowState__motor_state,  // get_const(index) function pointer
    get_function__LowState__motor_state,  // get(index) function pointer
    fetch_function__LowState__motor_state,  // fetch(index, &value) function pointer
    assign_function__LowState__motor_state,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers LowState_message_members = {
  "multi_port_motor_feedback::msg",  // message namespace
  "LowState",  // message name
  2,  // number of fields
  sizeof(multi_port_motor_feedback::msg::LowState),
  LowState_message_member_array,  // message members
  LowState_init_function,  // function to initialize message memory (memory has to be allocated)
  LowState_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t LowState_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &LowState_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace multi_port_motor_feedback


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<multi_port_motor_feedback::msg::LowState>()
{
  return &::multi_port_motor_feedback::msg::rosidl_typesupport_introspection_cpp::LowState_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, multi_port_motor_feedback, msg, LowState)() {
  return &::multi_port_motor_feedback::msg::rosidl_typesupport_introspection_cpp::LowState_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
