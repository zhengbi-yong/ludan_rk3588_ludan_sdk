// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from multi_port_motor_feedback:msg/MultiPortMotorFeedback.idl
// generated code does not contain a copyright notice

#ifndef MULTI_PORT_MOTOR_FEEDBACK__MSG__DETAIL__MULTI_PORT_MOTOR_FEEDBACK__STRUCT_HPP_
#define MULTI_PORT_MOTOR_FEEDBACK__MSG__DETAIL__MULTI_PORT_MOTOR_FEEDBACK__STRUCT_HPP_

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

#ifndef _WIN32
# define DEPRECATED__multi_port_motor_feedback__msg__MultiPortMotorFeedback __attribute__((deprecated))
#else
# define DEPRECATED__multi_port_motor_feedback__msg__MultiPortMotorFeedback __declspec(deprecated)
#endif

namespace multi_port_motor_feedback
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MultiPortMotorFeedback_
{
  using Type = MultiPortMotorFeedback_<ContainerAllocator>;

  explicit MultiPortMotorFeedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->motor_id = 0l;
      this->can_id = 0l;
      this->error = 0l;
      this->position = 0;
      this->velocity = 0;
      this->torque = 0;
      this->temp_mos = 0;
      this->temp_rotor = 0;
      this->position_rad = 0.0f;
      this->velocity_rad_s = 0.0f;
      this->torque_nm = 0.0f;
      std::fill<typename std::array<uint8_t, 8>::iterator, uint8_t>(this->can_data.begin(), this->can_data.end(), 0);
    }
  }

  explicit MultiPortMotorFeedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    can_data(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->motor_id = 0l;
      this->can_id = 0l;
      this->error = 0l;
      this->position = 0;
      this->velocity = 0;
      this->torque = 0;
      this->temp_mos = 0;
      this->temp_rotor = 0;
      this->position_rad = 0.0f;
      this->velocity_rad_s = 0.0f;
      this->torque_nm = 0.0f;
      std::fill<typename std::array<uint8_t, 8>::iterator, uint8_t>(this->can_data.begin(), this->can_data.end(), 0);
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _motor_id_type =
    int32_t;
  _motor_id_type motor_id;
  using _can_id_type =
    int32_t;
  _can_id_type can_id;
  using _error_type =
    int32_t;
  _error_type error;
  using _position_type =
    int16_t;
  _position_type position;
  using _velocity_type =
    int16_t;
  _velocity_type velocity;
  using _torque_type =
    int16_t;
  _torque_type torque;
  using _temp_mos_type =
    int8_t;
  _temp_mos_type temp_mos;
  using _temp_rotor_type =
    int8_t;
  _temp_rotor_type temp_rotor;
  using _position_rad_type =
    float;
  _position_rad_type position_rad;
  using _velocity_rad_s_type =
    float;
  _velocity_rad_s_type velocity_rad_s;
  using _torque_nm_type =
    float;
  _torque_nm_type torque_nm;
  using _can_data_type =
    std::array<uint8_t, 8>;
  _can_data_type can_data;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__motor_id(
    const int32_t & _arg)
  {
    this->motor_id = _arg;
    return *this;
  }
  Type & set__can_id(
    const int32_t & _arg)
  {
    this->can_id = _arg;
    return *this;
  }
  Type & set__error(
    const int32_t & _arg)
  {
    this->error = _arg;
    return *this;
  }
  Type & set__position(
    const int16_t & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__velocity(
    const int16_t & _arg)
  {
    this->velocity = _arg;
    return *this;
  }
  Type & set__torque(
    const int16_t & _arg)
  {
    this->torque = _arg;
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
  Type & set__position_rad(
    const float & _arg)
  {
    this->position_rad = _arg;
    return *this;
  }
  Type & set__velocity_rad_s(
    const float & _arg)
  {
    this->velocity_rad_s = _arg;
    return *this;
  }
  Type & set__torque_nm(
    const float & _arg)
  {
    this->torque_nm = _arg;
    return *this;
  }
  Type & set__can_data(
    const std::array<uint8_t, 8> & _arg)
  {
    this->can_data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    multi_port_motor_feedback::msg::MultiPortMotorFeedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const multi_port_motor_feedback::msg::MultiPortMotorFeedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<multi_port_motor_feedback::msg::MultiPortMotorFeedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<multi_port_motor_feedback::msg::MultiPortMotorFeedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      multi_port_motor_feedback::msg::MultiPortMotorFeedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<multi_port_motor_feedback::msg::MultiPortMotorFeedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      multi_port_motor_feedback::msg::MultiPortMotorFeedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<multi_port_motor_feedback::msg::MultiPortMotorFeedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<multi_port_motor_feedback::msg::MultiPortMotorFeedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<multi_port_motor_feedback::msg::MultiPortMotorFeedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__multi_port_motor_feedback__msg__MultiPortMotorFeedback
    std::shared_ptr<multi_port_motor_feedback::msg::MultiPortMotorFeedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__multi_port_motor_feedback__msg__MultiPortMotorFeedback
    std::shared_ptr<multi_port_motor_feedback::msg::MultiPortMotorFeedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MultiPortMotorFeedback_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->motor_id != other.motor_id) {
      return false;
    }
    if (this->can_id != other.can_id) {
      return false;
    }
    if (this->error != other.error) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->torque != other.torque) {
      return false;
    }
    if (this->temp_mos != other.temp_mos) {
      return false;
    }
    if (this->temp_rotor != other.temp_rotor) {
      return false;
    }
    if (this->position_rad != other.position_rad) {
      return false;
    }
    if (this->velocity_rad_s != other.velocity_rad_s) {
      return false;
    }
    if (this->torque_nm != other.torque_nm) {
      return false;
    }
    if (this->can_data != other.can_data) {
      return false;
    }
    return true;
  }
  bool operator!=(const MultiPortMotorFeedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MultiPortMotorFeedback_

// alias to use template instance with default allocator
using MultiPortMotorFeedback =
  multi_port_motor_feedback::msg::MultiPortMotorFeedback_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace multi_port_motor_feedback

#endif  // MULTI_PORT_MOTOR_FEEDBACK__MSG__DETAIL__MULTI_PORT_MOTOR_FEEDBACK__STRUCT_HPP_
