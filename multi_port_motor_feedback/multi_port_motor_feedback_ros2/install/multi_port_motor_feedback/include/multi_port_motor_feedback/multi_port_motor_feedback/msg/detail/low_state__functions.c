// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from multi_port_motor_feedback:msg/LowState.idl
// generated code does not contain a copyright notice
#include "multi_port_motor_feedback/msg/detail/low_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `motor_state`
#include "multi_port_motor_feedback/msg/detail/motor_state__functions.h"

bool
multi_port_motor_feedback__msg__LowState__init(multi_port_motor_feedback__msg__LowState * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    multi_port_motor_feedback__msg__LowState__fini(msg);
    return false;
  }
  // motor_state
  for (size_t i = 0; i < 30; ++i) {
    if (!multi_port_motor_feedback__msg__MotorState__init(&msg->motor_state[i])) {
      multi_port_motor_feedback__msg__LowState__fini(msg);
      return false;
    }
  }
  return true;
}

void
multi_port_motor_feedback__msg__LowState__fini(multi_port_motor_feedback__msg__LowState * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // motor_state
  for (size_t i = 0; i < 30; ++i) {
    multi_port_motor_feedback__msg__MotorState__fini(&msg->motor_state[i]);
  }
}

bool
multi_port_motor_feedback__msg__LowState__are_equal(const multi_port_motor_feedback__msg__LowState * lhs, const multi_port_motor_feedback__msg__LowState * rhs)
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
  // motor_state
  for (size_t i = 0; i < 30; ++i) {
    if (!multi_port_motor_feedback__msg__MotorState__are_equal(
        &(lhs->motor_state[i]), &(rhs->motor_state[i])))
    {
      return false;
    }
  }
  return true;
}

bool
multi_port_motor_feedback__msg__LowState__copy(
  const multi_port_motor_feedback__msg__LowState * input,
  multi_port_motor_feedback__msg__LowState * output)
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
  // motor_state
  for (size_t i = 0; i < 30; ++i) {
    if (!multi_port_motor_feedback__msg__MotorState__copy(
        &(input->motor_state[i]), &(output->motor_state[i])))
    {
      return false;
    }
  }
  return true;
}

multi_port_motor_feedback__msg__LowState *
multi_port_motor_feedback__msg__LowState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  multi_port_motor_feedback__msg__LowState * msg = (multi_port_motor_feedback__msg__LowState *)allocator.allocate(sizeof(multi_port_motor_feedback__msg__LowState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(multi_port_motor_feedback__msg__LowState));
  bool success = multi_port_motor_feedback__msg__LowState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
multi_port_motor_feedback__msg__LowState__destroy(multi_port_motor_feedback__msg__LowState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    multi_port_motor_feedback__msg__LowState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
multi_port_motor_feedback__msg__LowState__Sequence__init(multi_port_motor_feedback__msg__LowState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  multi_port_motor_feedback__msg__LowState * data = NULL;

  if (size) {
    data = (multi_port_motor_feedback__msg__LowState *)allocator.zero_allocate(size, sizeof(multi_port_motor_feedback__msg__LowState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = multi_port_motor_feedback__msg__LowState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        multi_port_motor_feedback__msg__LowState__fini(&data[i - 1]);
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
multi_port_motor_feedback__msg__LowState__Sequence__fini(multi_port_motor_feedback__msg__LowState__Sequence * array)
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
      multi_port_motor_feedback__msg__LowState__fini(&array->data[i]);
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

multi_port_motor_feedback__msg__LowState__Sequence *
multi_port_motor_feedback__msg__LowState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  multi_port_motor_feedback__msg__LowState__Sequence * array = (multi_port_motor_feedback__msg__LowState__Sequence *)allocator.allocate(sizeof(multi_port_motor_feedback__msg__LowState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = multi_port_motor_feedback__msg__LowState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
multi_port_motor_feedback__msg__LowState__Sequence__destroy(multi_port_motor_feedback__msg__LowState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    multi_port_motor_feedback__msg__LowState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
multi_port_motor_feedback__msg__LowState__Sequence__are_equal(const multi_port_motor_feedback__msg__LowState__Sequence * lhs, const multi_port_motor_feedback__msg__LowState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!multi_port_motor_feedback__msg__LowState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
multi_port_motor_feedback__msg__LowState__Sequence__copy(
  const multi_port_motor_feedback__msg__LowState__Sequence * input,
  multi_port_motor_feedback__msg__LowState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(multi_port_motor_feedback__msg__LowState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    multi_port_motor_feedback__msg__LowState * data =
      (multi_port_motor_feedback__msg__LowState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!multi_port_motor_feedback__msg__LowState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          multi_port_motor_feedback__msg__LowState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!multi_port_motor_feedback__msg__LowState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
