// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from multi_port_motor_feedback:msg/MultiPortMotorFeedback.idl
// generated code does not contain a copyright notice
#include "multi_port_motor_feedback/msg/detail/multi_port_motor_feedback__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
multi_port_motor_feedback__msg__MultiPortMotorFeedback__init(multi_port_motor_feedback__msg__MultiPortMotorFeedback * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    multi_port_motor_feedback__msg__MultiPortMotorFeedback__fini(msg);
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
multi_port_motor_feedback__msg__MultiPortMotorFeedback__fini(multi_port_motor_feedback__msg__MultiPortMotorFeedback * msg)
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
multi_port_motor_feedback__msg__MultiPortMotorFeedback__are_equal(const multi_port_motor_feedback__msg__MultiPortMotorFeedback * lhs, const multi_port_motor_feedback__msg__MultiPortMotorFeedback * rhs)
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
multi_port_motor_feedback__msg__MultiPortMotorFeedback__copy(
  const multi_port_motor_feedback__msg__MultiPortMotorFeedback * input,
  multi_port_motor_feedback__msg__MultiPortMotorFeedback * output)
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

multi_port_motor_feedback__msg__MultiPortMotorFeedback *
multi_port_motor_feedback__msg__MultiPortMotorFeedback__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  multi_port_motor_feedback__msg__MultiPortMotorFeedback * msg = (multi_port_motor_feedback__msg__MultiPortMotorFeedback *)allocator.allocate(sizeof(multi_port_motor_feedback__msg__MultiPortMotorFeedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(multi_port_motor_feedback__msg__MultiPortMotorFeedback));
  bool success = multi_port_motor_feedback__msg__MultiPortMotorFeedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
multi_port_motor_feedback__msg__MultiPortMotorFeedback__destroy(multi_port_motor_feedback__msg__MultiPortMotorFeedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    multi_port_motor_feedback__msg__MultiPortMotorFeedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
multi_port_motor_feedback__msg__MultiPortMotorFeedback__Sequence__init(multi_port_motor_feedback__msg__MultiPortMotorFeedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  multi_port_motor_feedback__msg__MultiPortMotorFeedback * data = NULL;

  if (size) {
    data = (multi_port_motor_feedback__msg__MultiPortMotorFeedback *)allocator.zero_allocate(size, sizeof(multi_port_motor_feedback__msg__MultiPortMotorFeedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = multi_port_motor_feedback__msg__MultiPortMotorFeedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        multi_port_motor_feedback__msg__MultiPortMotorFeedback__fini(&data[i - 1]);
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
multi_port_motor_feedback__msg__MultiPortMotorFeedback__Sequence__fini(multi_port_motor_feedback__msg__MultiPortMotorFeedback__Sequence * array)
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
      multi_port_motor_feedback__msg__MultiPortMotorFeedback__fini(&array->data[i]);
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

multi_port_motor_feedback__msg__MultiPortMotorFeedback__Sequence *
multi_port_motor_feedback__msg__MultiPortMotorFeedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  multi_port_motor_feedback__msg__MultiPortMotorFeedback__Sequence * array = (multi_port_motor_feedback__msg__MultiPortMotorFeedback__Sequence *)allocator.allocate(sizeof(multi_port_motor_feedback__msg__MultiPortMotorFeedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = multi_port_motor_feedback__msg__MultiPortMotorFeedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
multi_port_motor_feedback__msg__MultiPortMotorFeedback__Sequence__destroy(multi_port_motor_feedback__msg__MultiPortMotorFeedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    multi_port_motor_feedback__msg__MultiPortMotorFeedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
multi_port_motor_feedback__msg__MultiPortMotorFeedback__Sequence__are_equal(const multi_port_motor_feedback__msg__MultiPortMotorFeedback__Sequence * lhs, const multi_port_motor_feedback__msg__MultiPortMotorFeedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!multi_port_motor_feedback__msg__MultiPortMotorFeedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
multi_port_motor_feedback__msg__MultiPortMotorFeedback__Sequence__copy(
  const multi_port_motor_feedback__msg__MultiPortMotorFeedback__Sequence * input,
  multi_port_motor_feedback__msg__MultiPortMotorFeedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(multi_port_motor_feedback__msg__MultiPortMotorFeedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    multi_port_motor_feedback__msg__MultiPortMotorFeedback * data =
      (multi_port_motor_feedback__msg__MultiPortMotorFeedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!multi_port_motor_feedback__msg__MultiPortMotorFeedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          multi_port_motor_feedback__msg__MultiPortMotorFeedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!multi_port_motor_feedback__msg__MultiPortMotorFeedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
