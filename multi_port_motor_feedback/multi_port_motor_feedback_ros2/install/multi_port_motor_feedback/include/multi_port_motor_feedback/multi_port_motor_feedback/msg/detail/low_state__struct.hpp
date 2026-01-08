// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from multi_port_motor_feedback:msg/LowState.idl
// generated code does not contain a copyright notice

#ifndef MULTI_PORT_MOTOR_FEEDBACK__MSG__DETAIL__LOW_STATE__STRUCT_HPP_
#define MULTI_PORT_MOTOR_FEEDBACK__MSG__DETAIL__LOW_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'motor_state'
#include "multi_port_motor_feedback/msg/detail/motor_state__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__multi_port_motor_feedback__msg__LowState __attribute__((deprecated))
#else
# define DEPRECATED__multi_port_motor_feedback__msg__LowState __declspec(deprecated)
#endif

namespace multi_port_motor_feedback
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LowState_
{
  using Type = LowState_<ContainerAllocator>;

  explicit LowState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->motor_state.fill(multi_port_motor_feedback::msg::MotorState_<ContainerAllocator>{_init});
    }
  }

  explicit LowState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    motor_state(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->motor_state.fill(multi_port_motor_feedback::msg::MotorState_<ContainerAllocator>{_alloc, _init});
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _motor_state_type =
    std::array<multi_port_motor_feedback::msg::MotorState_<ContainerAllocator>, 30>;
  _motor_state_type motor_state;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__motor_state(
    const std::array<multi_port_motor_feedback::msg::MotorState_<ContainerAllocator>, 30> & _arg)
  {
    this->motor_state = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    multi_port_motor_feedback::msg::LowState_<ContainerAllocator> *;
  using ConstRawPtr =
    const multi_port_motor_feedback::msg::LowState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<multi_port_motor_feedback::msg::LowState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<multi_port_motor_feedback::msg::LowState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      multi_port_motor_feedback::msg::LowState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<multi_port_motor_feedback::msg::LowState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      multi_port_motor_feedback::msg::LowState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<multi_port_motor_feedback::msg::LowState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<multi_port_motor_feedback::msg::LowState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<multi_port_motor_feedback::msg::LowState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__multi_port_motor_feedback__msg__LowState
    std::shared_ptr<multi_port_motor_feedback::msg::LowState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__multi_port_motor_feedback__msg__LowState
    std::shared_ptr<multi_port_motor_feedback::msg::LowState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LowState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->motor_state != other.motor_state) {
      return false;
    }
    return true;
  }
  bool operator!=(const LowState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LowState_

// alias to use template instance with default allocator
using LowState =
  multi_port_motor_feedback::msg::LowState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace multi_port_motor_feedback

#endif  // MULTI_PORT_MOTOR_FEEDBACK__MSG__DETAIL__LOW_STATE__STRUCT_HPP_
