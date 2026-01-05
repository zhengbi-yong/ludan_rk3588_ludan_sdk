// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from motor_feedback:msg/MotorFeedback.idl
// generated code does not contain a copyright notice
#include "motor_feedback/msg/detail/motor_feedback__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
motor_feedback__msg__MotorFeedback__init(motor_feedback__msg__MotorFeedback * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    motor_feedback__msg__MotorFeedback__fini(msg);
    return false;
  }
  // motor_id
  // can_id
  // error
  // position
  // velocity
  // torque
  // temp_mos
  // temp_rotor
  // position_rad
  // velocity_rad_s
  // torque_nm
  // can_data
  return true;
}

void
motor_feedback__msg__MotorFeedback__fini(motor_feedback__msg__MotorFeedback * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // motor_id
  // can_id
  // error
  // position
  // velocity
  // torque
  // temp_mos
  // temp_rotor
  // position_rad
  // velocity_rad_s
  // torque_nm
  // can_data
}

bool
motor_feedback__msg__MotorFeedback__are_equal(const motor_feedback__msg__MotorFeedback * lhs, const motor_feedback__msg__MotorFeedback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // motor_id
  if (lhs->motor_id != rhs->motor_id) {
    return false;
  }
  // can_id
  if (lhs->can_id != rhs->can_id) {
    return false;
  }
  // error
  if (lhs->error != rhs->error) {
    return false;
  }
  // position
  if (lhs->position != rhs->position) {
    return false;
  }
  // velocity
  if (lhs->velocity != rhs->velocity) {
    return false;
  }
  // torque
  if (lhs->torque != rhs->torque) {
    return false;
  }
  // temp_mos
  if (lhs->temp_mos != rhs->temp_mos) {
    return false;
  }
  // temp_rotor
  if (lhs->temp_rotor != rhs->temp_rotor) {
    return false;
  }
  // position_rad
  if (lhs->position_rad != rhs->position_rad) {
    return false;
  }
  // velocity_rad_s
  if (lhs->velocity_rad_s != rhs->velocity_rad_s) {
    return false;
  }
  // torque_nm
  if (lhs->torque_nm != rhs->torque_nm) {
    return false;
  }
  // can_data
  for (size_t i = 0; i < 8; ++i) {
    if (lhs->can_data[i] != rhs->can_data[i]) {
      return false;
    }
  }
  return true;
}

bool
motor_feedback__msg__MotorFeedback__copy(
  const motor_feedback__msg__MotorFeedback * input,
  motor_feedback__msg__MotorFeedback * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // motor_id
  output->motor_id = input->motor_id;
  // can_id
  output->can_id = input->can_id;
  // error
  output->error = input->error;
  // position
  output->position = input->position;
  // velocity
  output->velocity = input->velocity;
  // torque
  output->torque = input->torque;
  // temp_mos
  output->temp_mos = input->temp_mos;
  // temp_rotor
  output->temp_rotor = input->temp_rotor;
  // position_rad
  output->position_rad = input->position_rad;
  // velocity_rad_s
  output->velocity_rad_s = input->velocity_rad_s;
  // torque_nm
  output->torque_nm = input->torque_nm;
  // can_data
  for (size_t i = 0; i < 8; ++i) {
    output->can_data[i] = input->can_data[i];
  }
  return true;
}

motor_feedback__msg__MotorFeedback *
motor_feedback__msg__MotorFeedback__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motor_feedback__msg__MotorFeedback * msg = (motor_feedback__msg__MotorFeedback *)allocator.allocate(sizeof(motor_feedback__msg__MotorFeedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(motor_feedback__msg__MotorFeedback));
  bool success = motor_feedback__msg__MotorFeedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
motor_feedback__msg__MotorFeedback__destroy(motor_feedback__msg__MotorFeedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    motor_feedback__msg__MotorFeedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
motor_feedback__msg__MotorFeedback__Sequence__init(motor_feedback__msg__MotorFeedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motor_feedback__msg__MotorFeedback * data = NULL;

  if (size) {
    data = (motor_feedback__msg__MotorFeedback *)allocator.zero_allocate(size, sizeof(motor_feedback__msg__MotorFeedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = motor_feedback__msg__MotorFeedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        motor_feedback__msg__MotorFeedback__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
motor_feedback__msg__MotorFeedback__Sequence__fini(motor_feedback__msg__MotorFeedback__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      motor_feedback__msg__MotorFeedback__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

motor_feedback__msg__MotorFeedback__Sequence *
motor_feedback__msg__MotorFeedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motor_feedback__msg__MotorFeedback__Sequence * array = (motor_feedback__msg__MotorFeedback__Sequence *)allocator.allocate(sizeof(motor_feedback__msg__MotorFeedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = motor_feedback__msg__MotorFeedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
motor_feedback__msg__MotorFeedback__Sequence__destroy(motor_feedback__msg__MotorFeedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    motor_feedback__msg__MotorFeedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
motor_feedback__msg__MotorFeedback__Sequence__are_equal(const motor_feedback__msg__MotorFeedback__Sequence * lhs, const motor_feedback__msg__MotorFeedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!motor_feedback__msg__MotorFeedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
motor_feedback__msg__MotorFeedback__Sequence__copy(
  const motor_feedback__msg__MotorFeedback__Sequence * input,
  motor_feedback__msg__MotorFeedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(motor_feedback__msg__MotorFeedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    motor_feedback__msg__MotorFeedback * data =
      (motor_feedback__msg__MotorFeedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!motor_feedback__msg__MotorFeedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          motor_feedback__msg__MotorFeedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!motor_feedback__msg__MotorFeedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
