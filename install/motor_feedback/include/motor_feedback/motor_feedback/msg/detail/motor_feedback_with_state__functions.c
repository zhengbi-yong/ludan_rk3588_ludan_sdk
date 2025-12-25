// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from motor_feedback:msg/MotorFeedbackWithState.idl
// generated code does not contain a copyright notice
#include "motor_feedback/msg/detail/motor_feedback_with_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
motor_feedback__msg__MotorFeedbackWithState__init(motor_feedback__msg__MotorFeedbackWithState * msg)
{
  if (!msg) {
    return false;
  }
  // id
  // position
  // velocity
  // effort
  // state
  return true;
}

void
motor_feedback__msg__MotorFeedbackWithState__fini(motor_feedback__msg__MotorFeedbackWithState * msg)
{
  if (!msg) {
    return;
  }
  // id
  // position
  // velocity
  // effort
  // state
}

bool
motor_feedback__msg__MotorFeedbackWithState__are_equal(const motor_feedback__msg__MotorFeedbackWithState * lhs, const motor_feedback__msg__MotorFeedbackWithState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
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
  // effort
  if (lhs->effort != rhs->effort) {
    return false;
  }
  // state
  if (lhs->state != rhs->state) {
    return false;
  }
  return true;
}

bool
motor_feedback__msg__MotorFeedbackWithState__copy(
  const motor_feedback__msg__MotorFeedbackWithState * input,
  motor_feedback__msg__MotorFeedbackWithState * output)
{
  if (!input || !output) {
    return false;
  }
  // id
  output->id = input->id;
  // position
  output->position = input->position;
  // velocity
  output->velocity = input->velocity;
  // effort
  output->effort = input->effort;
  // state
  output->state = input->state;
  return true;
}

motor_feedback__msg__MotorFeedbackWithState *
motor_feedback__msg__MotorFeedbackWithState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motor_feedback__msg__MotorFeedbackWithState * msg = (motor_feedback__msg__MotorFeedbackWithState *)allocator.allocate(sizeof(motor_feedback__msg__MotorFeedbackWithState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(motor_feedback__msg__MotorFeedbackWithState));
  bool success = motor_feedback__msg__MotorFeedbackWithState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
motor_feedback__msg__MotorFeedbackWithState__destroy(motor_feedback__msg__MotorFeedbackWithState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    motor_feedback__msg__MotorFeedbackWithState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
motor_feedback__msg__MotorFeedbackWithState__Sequence__init(motor_feedback__msg__MotorFeedbackWithState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motor_feedback__msg__MotorFeedbackWithState * data = NULL;

  if (size) {
    data = (motor_feedback__msg__MotorFeedbackWithState *)allocator.zero_allocate(size, sizeof(motor_feedback__msg__MotorFeedbackWithState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = motor_feedback__msg__MotorFeedbackWithState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        motor_feedback__msg__MotorFeedbackWithState__fini(&data[i - 1]);
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
motor_feedback__msg__MotorFeedbackWithState__Sequence__fini(motor_feedback__msg__MotorFeedbackWithState__Sequence * array)
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
      motor_feedback__msg__MotorFeedbackWithState__fini(&array->data[i]);
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

motor_feedback__msg__MotorFeedbackWithState__Sequence *
motor_feedback__msg__MotorFeedbackWithState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motor_feedback__msg__MotorFeedbackWithState__Sequence * array = (motor_feedback__msg__MotorFeedbackWithState__Sequence *)allocator.allocate(sizeof(motor_feedback__msg__MotorFeedbackWithState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = motor_feedback__msg__MotorFeedbackWithState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
motor_feedback__msg__MotorFeedbackWithState__Sequence__destroy(motor_feedback__msg__MotorFeedbackWithState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    motor_feedback__msg__MotorFeedbackWithState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
motor_feedback__msg__MotorFeedbackWithState__Sequence__are_equal(const motor_feedback__msg__MotorFeedbackWithState__Sequence * lhs, const motor_feedback__msg__MotorFeedbackWithState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!motor_feedback__msg__MotorFeedbackWithState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
motor_feedback__msg__MotorFeedbackWithState__Sequence__copy(
  const motor_feedback__msg__MotorFeedbackWithState__Sequence * input,
  motor_feedback__msg__MotorFeedbackWithState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(motor_feedback__msg__MotorFeedbackWithState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    motor_feedback__msg__MotorFeedbackWithState * data =
      (motor_feedback__msg__MotorFeedbackWithState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!motor_feedback__msg__MotorFeedbackWithState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          motor_feedback__msg__MotorFeedbackWithState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!motor_feedback__msg__MotorFeedbackWithState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
