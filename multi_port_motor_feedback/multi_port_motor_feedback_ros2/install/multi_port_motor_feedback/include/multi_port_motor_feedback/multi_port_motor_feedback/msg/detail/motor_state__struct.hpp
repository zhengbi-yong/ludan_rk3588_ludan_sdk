// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from multi_port_motor_feedback:msg/MotorState.idl
// generated code does not contain a copyright notice

#ifndef MULTI_PORT_MOTOR_FEEDBACK__MSG__DETAIL__MOTOR_STATE__STRUCT_HPP_
#define MULTI_PORT_MOTOR_FEEDBACK__MSG__DETAIL__MOTOR_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__multi_port_motor_feedback__msg__MotorState __attribute__((deprecated))
#else
# define DEPRECATED__multi_port_motor_feedback__msg__MotorState __declspec(deprecated)
#endif

namespace multi_port_motor_feedback
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MotorState_
{
  using Type = MotorState_<ContainerAllocator>;

  explicit MotorState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->valid = false;
      this->state = 0;
      this->id = 0;
      this->position = 0.0f;
      this->velocity = 0.0f;
      this->effort = 0.0f;
      this->temp_mos = 0;
      this->temp_rotor = 0;
    }
  }

  explicit MotorState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->valid = false;
      this->state = 0;
      this->id = 0;
      this->position = 0.0f;
      this->velocity = 0.0f;
      this->effort = 0.0f;
      this->temp_mos = 0;
      this->temp_rotor = 0;
    }
  }

  // field types and members
  using _valid_type =
    bool;
  _valid_type valid;
  using _state_type =
    uint8_t;
  _state_type state;
  using _id_type =
    int8_t;
  _id_type id;
  using _position_type =
    float;
  _position_type position;
  using _velocity_type =
    float;
  _velocity_type velocity;
  using _effort_type =
    float;
  _effort_type effort;
  using _temp_mos_type =
    int8_t;
  _temp_mos_type temp_mos;
  using _temp_rotor_type =
    int8_t;
  _temp_rotor_type temp_rotor;

  // setters for named parameter idiom
  Type & set__valid(
    const bool & _arg)
  {
    this->valid = _arg;
    return *this;
  }
  Type & set__state(
    const uint8_t & _arg)
  {
    this->state = _arg;
    return *this;
  }
  Type & set__id(
    const int8_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__position(
    const float & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__velocity(
    const float & _arg)
  {
    this->velocity = _arg;
    return *this;
  }
  Type & set__effort(
    const float & _arg)
  {
    this->effort = _arg;
    return *this;
  }
  Type & set__temp_mos(
    const int8_t & _arg)
  {
    this->temp_mos = _arg;
    return *this;
  }
  Type & set__temp_rotor(
    const int8_t & _arg)
  {
    this->temp_rotor = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    multi_port_motor_feedback::msg::MotorState_<ContainerAllocator> *;
  using ConstRawPtr =
    const multi_port_motor_feedback::msg::MotorState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<multi_port_motor_feedback::msg::MotorState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<multi_port_motor_feedback::msg::MotorState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      multi_port_motor_feedback::msg::MotorState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<multi_port_motor_feedback::msg::MotorState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      multi_port_motor_feedback::msg::MotorState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<multi_port_motor_feedback::msg::MotorState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<multi_port_motor_feedback::msg::MotorState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<multi_port_motor_feedback::msg::MotorState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__multi_port_motor_feedback__msg__MotorState
    std::shared_ptr<multi_port_motor_feedback::msg::MotorState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__multi_port_motor_feedback__msg__MotorState
    std::shared_ptr<multi_port_motor_feedback::msg::MotorState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotorState_ & other) const
  {
    if (this->valid != other.valid) {
      return false;
    }
    if (this->state != other.state) {
      return false;
    }
    if (this->id != other.id) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->effort != other.effort) {
      return false;
    }
    if (this->temp_mos != other.temp_mos) {
      return false;
    }
    if (this->temp_rotor != other.temp_rotor) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotorState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotorState_

// alias to use template instance with default allocator
using MotorState =
  multi_port_motor_feedback::msg::MotorState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace multi_port_motor_feedback

#endif  // MULTI_PORT_MOTOR_FEEDBACK__MSG__DETAIL__MOTOR_STATE__STRUCT_HPP_
