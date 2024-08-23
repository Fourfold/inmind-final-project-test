// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from detection_interfaces:action/FindYellow.idl
// generated code does not contain a copyright notice
#include "detection_interfaces/action/detail/find_yellow__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
detection_interfaces__action__FindYellow_Goal__init(detection_interfaces__action__FindYellow_Goal * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
detection_interfaces__action__FindYellow_Goal__fini(detection_interfaces__action__FindYellow_Goal * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
detection_interfaces__action__FindYellow_Goal__are_equal(const detection_interfaces__action__FindYellow_Goal * lhs, const detection_interfaces__action__FindYellow_Goal * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
detection_interfaces__action__FindYellow_Goal__copy(
  const detection_interfaces__action__FindYellow_Goal * input,
  detection_interfaces__action__FindYellow_Goal * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

detection_interfaces__action__FindYellow_Goal *
detection_interfaces__action__FindYellow_Goal__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  detection_interfaces__action__FindYellow_Goal * msg = (detection_interfaces__action__FindYellow_Goal *)allocator.allocate(sizeof(detection_interfaces__action__FindYellow_Goal), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(detection_interfaces__action__FindYellow_Goal));
  bool success = detection_interfaces__action__FindYellow_Goal__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
detection_interfaces__action__FindYellow_Goal__destroy(detection_interfaces__action__FindYellow_Goal * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    detection_interfaces__action__FindYellow_Goal__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
detection_interfaces__action__FindYellow_Goal__Sequence__init(detection_interfaces__action__FindYellow_Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  detection_interfaces__action__FindYellow_Goal * data = NULL;

  if (size) {
    data = (detection_interfaces__action__FindYellow_Goal *)allocator.zero_allocate(size, sizeof(detection_interfaces__action__FindYellow_Goal), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = detection_interfaces__action__FindYellow_Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        detection_interfaces__action__FindYellow_Goal__fini(&data[i - 1]);
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
detection_interfaces__action__FindYellow_Goal__Sequence__fini(detection_interfaces__action__FindYellow_Goal__Sequence * array)
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
      detection_interfaces__action__FindYellow_Goal__fini(&array->data[i]);
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

detection_interfaces__action__FindYellow_Goal__Sequence *
detection_interfaces__action__FindYellow_Goal__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  detection_interfaces__action__FindYellow_Goal__Sequence * array = (detection_interfaces__action__FindYellow_Goal__Sequence *)allocator.allocate(sizeof(detection_interfaces__action__FindYellow_Goal__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = detection_interfaces__action__FindYellow_Goal__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
detection_interfaces__action__FindYellow_Goal__Sequence__destroy(detection_interfaces__action__FindYellow_Goal__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    detection_interfaces__action__FindYellow_Goal__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
detection_interfaces__action__FindYellow_Goal__Sequence__are_equal(const detection_interfaces__action__FindYellow_Goal__Sequence * lhs, const detection_interfaces__action__FindYellow_Goal__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!detection_interfaces__action__FindYellow_Goal__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
detection_interfaces__action__FindYellow_Goal__Sequence__copy(
  const detection_interfaces__action__FindYellow_Goal__Sequence * input,
  detection_interfaces__action__FindYellow_Goal__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(detection_interfaces__action__FindYellow_Goal);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    detection_interfaces__action__FindYellow_Goal * data =
      (detection_interfaces__action__FindYellow_Goal *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!detection_interfaces__action__FindYellow_Goal__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          detection_interfaces__action__FindYellow_Goal__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!detection_interfaces__action__FindYellow_Goal__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
detection_interfaces__action__FindYellow_Result__init(detection_interfaces__action__FindYellow_Result * msg)
{
  if (!msg) {
    return false;
  }
  // found
  // frame_width
  // cx
  // cy
  return true;
}

void
detection_interfaces__action__FindYellow_Result__fini(detection_interfaces__action__FindYellow_Result * msg)
{
  if (!msg) {
    return;
  }
  // found
  // frame_width
  // cx
  // cy
}

bool
detection_interfaces__action__FindYellow_Result__are_equal(const detection_interfaces__action__FindYellow_Result * lhs, const detection_interfaces__action__FindYellow_Result * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // found
  if (lhs->found != rhs->found) {
    return false;
  }
  // frame_width
  if (lhs->frame_width != rhs->frame_width) {
    return false;
  }
  // cx
  if (lhs->cx != rhs->cx) {
    return false;
  }
  // cy
  if (lhs->cy != rhs->cy) {
    return false;
  }
  return true;
}

bool
detection_interfaces__action__FindYellow_Result__copy(
  const detection_interfaces__action__FindYellow_Result * input,
  detection_interfaces__action__FindYellow_Result * output)
{
  if (!input || !output) {
    return false;
  }
  // found
  output->found = input->found;
  // frame_width
  output->frame_width = input->frame_width;
  // cx
  output->cx = input->cx;
  // cy
  output->cy = input->cy;
  return true;
}

detection_interfaces__action__FindYellow_Result *
detection_interfaces__action__FindYellow_Result__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  detection_interfaces__action__FindYellow_Result * msg = (detection_interfaces__action__FindYellow_Result *)allocator.allocate(sizeof(detection_interfaces__action__FindYellow_Result), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(detection_interfaces__action__FindYellow_Result));
  bool success = detection_interfaces__action__FindYellow_Result__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
detection_interfaces__action__FindYellow_Result__destroy(detection_interfaces__action__FindYellow_Result * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    detection_interfaces__action__FindYellow_Result__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
detection_interfaces__action__FindYellow_Result__Sequence__init(detection_interfaces__action__FindYellow_Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  detection_interfaces__action__FindYellow_Result * data = NULL;

  if (size) {
    data = (detection_interfaces__action__FindYellow_Result *)allocator.zero_allocate(size, sizeof(detection_interfaces__action__FindYellow_Result), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = detection_interfaces__action__FindYellow_Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        detection_interfaces__action__FindYellow_Result__fini(&data[i - 1]);
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
detection_interfaces__action__FindYellow_Result__Sequence__fini(detection_interfaces__action__FindYellow_Result__Sequence * array)
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
      detection_interfaces__action__FindYellow_Result__fini(&array->data[i]);
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

detection_interfaces__action__FindYellow_Result__Sequence *
detection_interfaces__action__FindYellow_Result__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  detection_interfaces__action__FindYellow_Result__Sequence * array = (detection_interfaces__action__FindYellow_Result__Sequence *)allocator.allocate(sizeof(detection_interfaces__action__FindYellow_Result__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = detection_interfaces__action__FindYellow_Result__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
detection_interfaces__action__FindYellow_Result__Sequence__destroy(detection_interfaces__action__FindYellow_Result__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    detection_interfaces__action__FindYellow_Result__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
detection_interfaces__action__FindYellow_Result__Sequence__are_equal(const detection_interfaces__action__FindYellow_Result__Sequence * lhs, const detection_interfaces__action__FindYellow_Result__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!detection_interfaces__action__FindYellow_Result__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
detection_interfaces__action__FindYellow_Result__Sequence__copy(
  const detection_interfaces__action__FindYellow_Result__Sequence * input,
  detection_interfaces__action__FindYellow_Result__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(detection_interfaces__action__FindYellow_Result);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    detection_interfaces__action__FindYellow_Result * data =
      (detection_interfaces__action__FindYellow_Result *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!detection_interfaces__action__FindYellow_Result__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          detection_interfaces__action__FindYellow_Result__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!detection_interfaces__action__FindYellow_Result__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
detection_interfaces__action__FindYellow_Feedback__init(detection_interfaces__action__FindYellow_Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
detection_interfaces__action__FindYellow_Feedback__fini(detection_interfaces__action__FindYellow_Feedback * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
detection_interfaces__action__FindYellow_Feedback__are_equal(const detection_interfaces__action__FindYellow_Feedback * lhs, const detection_interfaces__action__FindYellow_Feedback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
detection_interfaces__action__FindYellow_Feedback__copy(
  const detection_interfaces__action__FindYellow_Feedback * input,
  detection_interfaces__action__FindYellow_Feedback * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

detection_interfaces__action__FindYellow_Feedback *
detection_interfaces__action__FindYellow_Feedback__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  detection_interfaces__action__FindYellow_Feedback * msg = (detection_interfaces__action__FindYellow_Feedback *)allocator.allocate(sizeof(detection_interfaces__action__FindYellow_Feedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(detection_interfaces__action__FindYellow_Feedback));
  bool success = detection_interfaces__action__FindYellow_Feedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
detection_interfaces__action__FindYellow_Feedback__destroy(detection_interfaces__action__FindYellow_Feedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    detection_interfaces__action__FindYellow_Feedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
detection_interfaces__action__FindYellow_Feedback__Sequence__init(detection_interfaces__action__FindYellow_Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  detection_interfaces__action__FindYellow_Feedback * data = NULL;

  if (size) {
    data = (detection_interfaces__action__FindYellow_Feedback *)allocator.zero_allocate(size, sizeof(detection_interfaces__action__FindYellow_Feedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = detection_interfaces__action__FindYellow_Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        detection_interfaces__action__FindYellow_Feedback__fini(&data[i - 1]);
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
detection_interfaces__action__FindYellow_Feedback__Sequence__fini(detection_interfaces__action__FindYellow_Feedback__Sequence * array)
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
      detection_interfaces__action__FindYellow_Feedback__fini(&array->data[i]);
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

detection_interfaces__action__FindYellow_Feedback__Sequence *
detection_interfaces__action__FindYellow_Feedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  detection_interfaces__action__FindYellow_Feedback__Sequence * array = (detection_interfaces__action__FindYellow_Feedback__Sequence *)allocator.allocate(sizeof(detection_interfaces__action__FindYellow_Feedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = detection_interfaces__action__FindYellow_Feedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
detection_interfaces__action__FindYellow_Feedback__Sequence__destroy(detection_interfaces__action__FindYellow_Feedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    detection_interfaces__action__FindYellow_Feedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
detection_interfaces__action__FindYellow_Feedback__Sequence__are_equal(const detection_interfaces__action__FindYellow_Feedback__Sequence * lhs, const detection_interfaces__action__FindYellow_Feedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!detection_interfaces__action__FindYellow_Feedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
detection_interfaces__action__FindYellow_Feedback__Sequence__copy(
  const detection_interfaces__action__FindYellow_Feedback__Sequence * input,
  detection_interfaces__action__FindYellow_Feedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(detection_interfaces__action__FindYellow_Feedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    detection_interfaces__action__FindYellow_Feedback * data =
      (detection_interfaces__action__FindYellow_Feedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!detection_interfaces__action__FindYellow_Feedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          detection_interfaces__action__FindYellow_Feedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!detection_interfaces__action__FindYellow_Feedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `goal`
// already included above
// #include "detection_interfaces/action/detail/find_yellow__functions.h"

bool
detection_interfaces__action__FindYellow_SendGoal_Request__init(detection_interfaces__action__FindYellow_SendGoal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    detection_interfaces__action__FindYellow_SendGoal_Request__fini(msg);
    return false;
  }
  // goal
  if (!detection_interfaces__action__FindYellow_Goal__init(&msg->goal)) {
    detection_interfaces__action__FindYellow_SendGoal_Request__fini(msg);
    return false;
  }
  return true;
}

void
detection_interfaces__action__FindYellow_SendGoal_Request__fini(detection_interfaces__action__FindYellow_SendGoal_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // goal
  detection_interfaces__action__FindYellow_Goal__fini(&msg->goal);
}

bool
detection_interfaces__action__FindYellow_SendGoal_Request__are_equal(const detection_interfaces__action__FindYellow_SendGoal_Request * lhs, const detection_interfaces__action__FindYellow_SendGoal_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // goal
  if (!detection_interfaces__action__FindYellow_Goal__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
detection_interfaces__action__FindYellow_SendGoal_Request__copy(
  const detection_interfaces__action__FindYellow_SendGoal_Request * input,
  detection_interfaces__action__FindYellow_SendGoal_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // goal
  if (!detection_interfaces__action__FindYellow_Goal__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

detection_interfaces__action__FindYellow_SendGoal_Request *
detection_interfaces__action__FindYellow_SendGoal_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  detection_interfaces__action__FindYellow_SendGoal_Request * msg = (detection_interfaces__action__FindYellow_SendGoal_Request *)allocator.allocate(sizeof(detection_interfaces__action__FindYellow_SendGoal_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(detection_interfaces__action__FindYellow_SendGoal_Request));
  bool success = detection_interfaces__action__FindYellow_SendGoal_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
detection_interfaces__action__FindYellow_SendGoal_Request__destroy(detection_interfaces__action__FindYellow_SendGoal_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    detection_interfaces__action__FindYellow_SendGoal_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
detection_interfaces__action__FindYellow_SendGoal_Request__Sequence__init(detection_interfaces__action__FindYellow_SendGoal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  detection_interfaces__action__FindYellow_SendGoal_Request * data = NULL;

  if (size) {
    data = (detection_interfaces__action__FindYellow_SendGoal_Request *)allocator.zero_allocate(size, sizeof(detection_interfaces__action__FindYellow_SendGoal_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = detection_interfaces__action__FindYellow_SendGoal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        detection_interfaces__action__FindYellow_SendGoal_Request__fini(&data[i - 1]);
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
detection_interfaces__action__FindYellow_SendGoal_Request__Sequence__fini(detection_interfaces__action__FindYellow_SendGoal_Request__Sequence * array)
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
      detection_interfaces__action__FindYellow_SendGoal_Request__fini(&array->data[i]);
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

detection_interfaces__action__FindYellow_SendGoal_Request__Sequence *
detection_interfaces__action__FindYellow_SendGoal_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  detection_interfaces__action__FindYellow_SendGoal_Request__Sequence * array = (detection_interfaces__action__FindYellow_SendGoal_Request__Sequence *)allocator.allocate(sizeof(detection_interfaces__action__FindYellow_SendGoal_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = detection_interfaces__action__FindYellow_SendGoal_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
detection_interfaces__action__FindYellow_SendGoal_Request__Sequence__destroy(detection_interfaces__action__FindYellow_SendGoal_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    detection_interfaces__action__FindYellow_SendGoal_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
detection_interfaces__action__FindYellow_SendGoal_Request__Sequence__are_equal(const detection_interfaces__action__FindYellow_SendGoal_Request__Sequence * lhs, const detection_interfaces__action__FindYellow_SendGoal_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!detection_interfaces__action__FindYellow_SendGoal_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
detection_interfaces__action__FindYellow_SendGoal_Request__Sequence__copy(
  const detection_interfaces__action__FindYellow_SendGoal_Request__Sequence * input,
  detection_interfaces__action__FindYellow_SendGoal_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(detection_interfaces__action__FindYellow_SendGoal_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    detection_interfaces__action__FindYellow_SendGoal_Request * data =
      (detection_interfaces__action__FindYellow_SendGoal_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!detection_interfaces__action__FindYellow_SendGoal_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          detection_interfaces__action__FindYellow_SendGoal_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!detection_interfaces__action__FindYellow_SendGoal_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
detection_interfaces__action__FindYellow_SendGoal_Response__init(detection_interfaces__action__FindYellow_SendGoal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    detection_interfaces__action__FindYellow_SendGoal_Response__fini(msg);
    return false;
  }
  return true;
}

void
detection_interfaces__action__FindYellow_SendGoal_Response__fini(detection_interfaces__action__FindYellow_SendGoal_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

bool
detection_interfaces__action__FindYellow_SendGoal_Response__are_equal(const detection_interfaces__action__FindYellow_SendGoal_Response * lhs, const detection_interfaces__action__FindYellow_SendGoal_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // accepted
  if (lhs->accepted != rhs->accepted) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  return true;
}

bool
detection_interfaces__action__FindYellow_SendGoal_Response__copy(
  const detection_interfaces__action__FindYellow_SendGoal_Response * input,
  detection_interfaces__action__FindYellow_SendGoal_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // accepted
  output->accepted = input->accepted;
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  return true;
}

detection_interfaces__action__FindYellow_SendGoal_Response *
detection_interfaces__action__FindYellow_SendGoal_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  detection_interfaces__action__FindYellow_SendGoal_Response * msg = (detection_interfaces__action__FindYellow_SendGoal_Response *)allocator.allocate(sizeof(detection_interfaces__action__FindYellow_SendGoal_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(detection_interfaces__action__FindYellow_SendGoal_Response));
  bool success = detection_interfaces__action__FindYellow_SendGoal_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
detection_interfaces__action__FindYellow_SendGoal_Response__destroy(detection_interfaces__action__FindYellow_SendGoal_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    detection_interfaces__action__FindYellow_SendGoal_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
detection_interfaces__action__FindYellow_SendGoal_Response__Sequence__init(detection_interfaces__action__FindYellow_SendGoal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  detection_interfaces__action__FindYellow_SendGoal_Response * data = NULL;

  if (size) {
    data = (detection_interfaces__action__FindYellow_SendGoal_Response *)allocator.zero_allocate(size, sizeof(detection_interfaces__action__FindYellow_SendGoal_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = detection_interfaces__action__FindYellow_SendGoal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        detection_interfaces__action__FindYellow_SendGoal_Response__fini(&data[i - 1]);
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
detection_interfaces__action__FindYellow_SendGoal_Response__Sequence__fini(detection_interfaces__action__FindYellow_SendGoal_Response__Sequence * array)
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
      detection_interfaces__action__FindYellow_SendGoal_Response__fini(&array->data[i]);
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

detection_interfaces__action__FindYellow_SendGoal_Response__Sequence *
detection_interfaces__action__FindYellow_SendGoal_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  detection_interfaces__action__FindYellow_SendGoal_Response__Sequence * array = (detection_interfaces__action__FindYellow_SendGoal_Response__Sequence *)allocator.allocate(sizeof(detection_interfaces__action__FindYellow_SendGoal_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = detection_interfaces__action__FindYellow_SendGoal_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
detection_interfaces__action__FindYellow_SendGoal_Response__Sequence__destroy(detection_interfaces__action__FindYellow_SendGoal_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    detection_interfaces__action__FindYellow_SendGoal_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
detection_interfaces__action__FindYellow_SendGoal_Response__Sequence__are_equal(const detection_interfaces__action__FindYellow_SendGoal_Response__Sequence * lhs, const detection_interfaces__action__FindYellow_SendGoal_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!detection_interfaces__action__FindYellow_SendGoal_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
detection_interfaces__action__FindYellow_SendGoal_Response__Sequence__copy(
  const detection_interfaces__action__FindYellow_SendGoal_Response__Sequence * input,
  detection_interfaces__action__FindYellow_SendGoal_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(detection_interfaces__action__FindYellow_SendGoal_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    detection_interfaces__action__FindYellow_SendGoal_Response * data =
      (detection_interfaces__action__FindYellow_SendGoal_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!detection_interfaces__action__FindYellow_SendGoal_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          detection_interfaces__action__FindYellow_SendGoal_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!detection_interfaces__action__FindYellow_SendGoal_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"

bool
detection_interfaces__action__FindYellow_GetResult_Request__init(detection_interfaces__action__FindYellow_GetResult_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    detection_interfaces__action__FindYellow_GetResult_Request__fini(msg);
    return false;
  }
  return true;
}

void
detection_interfaces__action__FindYellow_GetResult_Request__fini(detection_interfaces__action__FindYellow_GetResult_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
}

bool
detection_interfaces__action__FindYellow_GetResult_Request__are_equal(const detection_interfaces__action__FindYellow_GetResult_Request * lhs, const detection_interfaces__action__FindYellow_GetResult_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  return true;
}

bool
detection_interfaces__action__FindYellow_GetResult_Request__copy(
  const detection_interfaces__action__FindYellow_GetResult_Request * input,
  detection_interfaces__action__FindYellow_GetResult_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  return true;
}

detection_interfaces__action__FindYellow_GetResult_Request *
detection_interfaces__action__FindYellow_GetResult_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  detection_interfaces__action__FindYellow_GetResult_Request * msg = (detection_interfaces__action__FindYellow_GetResult_Request *)allocator.allocate(sizeof(detection_interfaces__action__FindYellow_GetResult_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(detection_interfaces__action__FindYellow_GetResult_Request));
  bool success = detection_interfaces__action__FindYellow_GetResult_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
detection_interfaces__action__FindYellow_GetResult_Request__destroy(detection_interfaces__action__FindYellow_GetResult_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    detection_interfaces__action__FindYellow_GetResult_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
detection_interfaces__action__FindYellow_GetResult_Request__Sequence__init(detection_interfaces__action__FindYellow_GetResult_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  detection_interfaces__action__FindYellow_GetResult_Request * data = NULL;

  if (size) {
    data = (detection_interfaces__action__FindYellow_GetResult_Request *)allocator.zero_allocate(size, sizeof(detection_interfaces__action__FindYellow_GetResult_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = detection_interfaces__action__FindYellow_GetResult_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        detection_interfaces__action__FindYellow_GetResult_Request__fini(&data[i - 1]);
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
detection_interfaces__action__FindYellow_GetResult_Request__Sequence__fini(detection_interfaces__action__FindYellow_GetResult_Request__Sequence * array)
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
      detection_interfaces__action__FindYellow_GetResult_Request__fini(&array->data[i]);
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

detection_interfaces__action__FindYellow_GetResult_Request__Sequence *
detection_interfaces__action__FindYellow_GetResult_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  detection_interfaces__action__FindYellow_GetResult_Request__Sequence * array = (detection_interfaces__action__FindYellow_GetResult_Request__Sequence *)allocator.allocate(sizeof(detection_interfaces__action__FindYellow_GetResult_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = detection_interfaces__action__FindYellow_GetResult_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
detection_interfaces__action__FindYellow_GetResult_Request__Sequence__destroy(detection_interfaces__action__FindYellow_GetResult_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    detection_interfaces__action__FindYellow_GetResult_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
detection_interfaces__action__FindYellow_GetResult_Request__Sequence__are_equal(const detection_interfaces__action__FindYellow_GetResult_Request__Sequence * lhs, const detection_interfaces__action__FindYellow_GetResult_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!detection_interfaces__action__FindYellow_GetResult_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
detection_interfaces__action__FindYellow_GetResult_Request__Sequence__copy(
  const detection_interfaces__action__FindYellow_GetResult_Request__Sequence * input,
  detection_interfaces__action__FindYellow_GetResult_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(detection_interfaces__action__FindYellow_GetResult_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    detection_interfaces__action__FindYellow_GetResult_Request * data =
      (detection_interfaces__action__FindYellow_GetResult_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!detection_interfaces__action__FindYellow_GetResult_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          detection_interfaces__action__FindYellow_GetResult_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!detection_interfaces__action__FindYellow_GetResult_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `result`
// already included above
// #include "detection_interfaces/action/detail/find_yellow__functions.h"

bool
detection_interfaces__action__FindYellow_GetResult_Response__init(detection_interfaces__action__FindYellow_GetResult_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // result
  if (!detection_interfaces__action__FindYellow_Result__init(&msg->result)) {
    detection_interfaces__action__FindYellow_GetResult_Response__fini(msg);
    return false;
  }
  return true;
}

void
detection_interfaces__action__FindYellow_GetResult_Response__fini(detection_interfaces__action__FindYellow_GetResult_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // result
  detection_interfaces__action__FindYellow_Result__fini(&msg->result);
}

bool
detection_interfaces__action__FindYellow_GetResult_Response__are_equal(const detection_interfaces__action__FindYellow_GetResult_Response * lhs, const detection_interfaces__action__FindYellow_GetResult_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // result
  if (!detection_interfaces__action__FindYellow_Result__are_equal(
      &(lhs->result), &(rhs->result)))
  {
    return false;
  }
  return true;
}

bool
detection_interfaces__action__FindYellow_GetResult_Response__copy(
  const detection_interfaces__action__FindYellow_GetResult_Response * input,
  detection_interfaces__action__FindYellow_GetResult_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  // result
  if (!detection_interfaces__action__FindYellow_Result__copy(
      &(input->result), &(output->result)))
  {
    return false;
  }
  return true;
}

detection_interfaces__action__FindYellow_GetResult_Response *
detection_interfaces__action__FindYellow_GetResult_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  detection_interfaces__action__FindYellow_GetResult_Response * msg = (detection_interfaces__action__FindYellow_GetResult_Response *)allocator.allocate(sizeof(detection_interfaces__action__FindYellow_GetResult_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(detection_interfaces__action__FindYellow_GetResult_Response));
  bool success = detection_interfaces__action__FindYellow_GetResult_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
detection_interfaces__action__FindYellow_GetResult_Response__destroy(detection_interfaces__action__FindYellow_GetResult_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    detection_interfaces__action__FindYellow_GetResult_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
detection_interfaces__action__FindYellow_GetResult_Response__Sequence__init(detection_interfaces__action__FindYellow_GetResult_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  detection_interfaces__action__FindYellow_GetResult_Response * data = NULL;

  if (size) {
    data = (detection_interfaces__action__FindYellow_GetResult_Response *)allocator.zero_allocate(size, sizeof(detection_interfaces__action__FindYellow_GetResult_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = detection_interfaces__action__FindYellow_GetResult_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        detection_interfaces__action__FindYellow_GetResult_Response__fini(&data[i - 1]);
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
detection_interfaces__action__FindYellow_GetResult_Response__Sequence__fini(detection_interfaces__action__FindYellow_GetResult_Response__Sequence * array)
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
      detection_interfaces__action__FindYellow_GetResult_Response__fini(&array->data[i]);
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

detection_interfaces__action__FindYellow_GetResult_Response__Sequence *
detection_interfaces__action__FindYellow_GetResult_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  detection_interfaces__action__FindYellow_GetResult_Response__Sequence * array = (detection_interfaces__action__FindYellow_GetResult_Response__Sequence *)allocator.allocate(sizeof(detection_interfaces__action__FindYellow_GetResult_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = detection_interfaces__action__FindYellow_GetResult_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
detection_interfaces__action__FindYellow_GetResult_Response__Sequence__destroy(detection_interfaces__action__FindYellow_GetResult_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    detection_interfaces__action__FindYellow_GetResult_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
detection_interfaces__action__FindYellow_GetResult_Response__Sequence__are_equal(const detection_interfaces__action__FindYellow_GetResult_Response__Sequence * lhs, const detection_interfaces__action__FindYellow_GetResult_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!detection_interfaces__action__FindYellow_GetResult_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
detection_interfaces__action__FindYellow_GetResult_Response__Sequence__copy(
  const detection_interfaces__action__FindYellow_GetResult_Response__Sequence * input,
  detection_interfaces__action__FindYellow_GetResult_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(detection_interfaces__action__FindYellow_GetResult_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    detection_interfaces__action__FindYellow_GetResult_Response * data =
      (detection_interfaces__action__FindYellow_GetResult_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!detection_interfaces__action__FindYellow_GetResult_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          detection_interfaces__action__FindYellow_GetResult_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!detection_interfaces__action__FindYellow_GetResult_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `feedback`
// already included above
// #include "detection_interfaces/action/detail/find_yellow__functions.h"

bool
detection_interfaces__action__FindYellow_FeedbackMessage__init(detection_interfaces__action__FindYellow_FeedbackMessage * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    detection_interfaces__action__FindYellow_FeedbackMessage__fini(msg);
    return false;
  }
  // feedback
  if (!detection_interfaces__action__FindYellow_Feedback__init(&msg->feedback)) {
    detection_interfaces__action__FindYellow_FeedbackMessage__fini(msg);
    return false;
  }
  return true;
}

void
detection_interfaces__action__FindYellow_FeedbackMessage__fini(detection_interfaces__action__FindYellow_FeedbackMessage * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // feedback
  detection_interfaces__action__FindYellow_Feedback__fini(&msg->feedback);
}

bool
detection_interfaces__action__FindYellow_FeedbackMessage__are_equal(const detection_interfaces__action__FindYellow_FeedbackMessage * lhs, const detection_interfaces__action__FindYellow_FeedbackMessage * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // feedback
  if (!detection_interfaces__action__FindYellow_Feedback__are_equal(
      &(lhs->feedback), &(rhs->feedback)))
  {
    return false;
  }
  return true;
}

bool
detection_interfaces__action__FindYellow_FeedbackMessage__copy(
  const detection_interfaces__action__FindYellow_FeedbackMessage * input,
  detection_interfaces__action__FindYellow_FeedbackMessage * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // feedback
  if (!detection_interfaces__action__FindYellow_Feedback__copy(
      &(input->feedback), &(output->feedback)))
  {
    return false;
  }
  return true;
}

detection_interfaces__action__FindYellow_FeedbackMessage *
detection_interfaces__action__FindYellow_FeedbackMessage__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  detection_interfaces__action__FindYellow_FeedbackMessage * msg = (detection_interfaces__action__FindYellow_FeedbackMessage *)allocator.allocate(sizeof(detection_interfaces__action__FindYellow_FeedbackMessage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(detection_interfaces__action__FindYellow_FeedbackMessage));
  bool success = detection_interfaces__action__FindYellow_FeedbackMessage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
detection_interfaces__action__FindYellow_FeedbackMessage__destroy(detection_interfaces__action__FindYellow_FeedbackMessage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    detection_interfaces__action__FindYellow_FeedbackMessage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
detection_interfaces__action__FindYellow_FeedbackMessage__Sequence__init(detection_interfaces__action__FindYellow_FeedbackMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  detection_interfaces__action__FindYellow_FeedbackMessage * data = NULL;

  if (size) {
    data = (detection_interfaces__action__FindYellow_FeedbackMessage *)allocator.zero_allocate(size, sizeof(detection_interfaces__action__FindYellow_FeedbackMessage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = detection_interfaces__action__FindYellow_FeedbackMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        detection_interfaces__action__FindYellow_FeedbackMessage__fini(&data[i - 1]);
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
detection_interfaces__action__FindYellow_FeedbackMessage__Sequence__fini(detection_interfaces__action__FindYellow_FeedbackMessage__Sequence * array)
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
      detection_interfaces__action__FindYellow_FeedbackMessage__fini(&array->data[i]);
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

detection_interfaces__action__FindYellow_FeedbackMessage__Sequence *
detection_interfaces__action__FindYellow_FeedbackMessage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  detection_interfaces__action__FindYellow_FeedbackMessage__Sequence * array = (detection_interfaces__action__FindYellow_FeedbackMessage__Sequence *)allocator.allocate(sizeof(detection_interfaces__action__FindYellow_FeedbackMessage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = detection_interfaces__action__FindYellow_FeedbackMessage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
detection_interfaces__action__FindYellow_FeedbackMessage__Sequence__destroy(detection_interfaces__action__FindYellow_FeedbackMessage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    detection_interfaces__action__FindYellow_FeedbackMessage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
detection_interfaces__action__FindYellow_FeedbackMessage__Sequence__are_equal(const detection_interfaces__action__FindYellow_FeedbackMessage__Sequence * lhs, const detection_interfaces__action__FindYellow_FeedbackMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!detection_interfaces__action__FindYellow_FeedbackMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
detection_interfaces__action__FindYellow_FeedbackMessage__Sequence__copy(
  const detection_interfaces__action__FindYellow_FeedbackMessage__Sequence * input,
  detection_interfaces__action__FindYellow_FeedbackMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(detection_interfaces__action__FindYellow_FeedbackMessage);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    detection_interfaces__action__FindYellow_FeedbackMessage * data =
      (detection_interfaces__action__FindYellow_FeedbackMessage *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!detection_interfaces__action__FindYellow_FeedbackMessage__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          detection_interfaces__action__FindYellow_FeedbackMessage__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!detection_interfaces__action__FindYellow_FeedbackMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
