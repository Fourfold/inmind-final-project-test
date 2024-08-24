// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from distance_interfaces:srv/GetFrontDistance.idl
// generated code does not contain a copyright notice
#include "distance_interfaces/srv/detail/get_front_distance__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
distance_interfaces__srv__GetFrontDistance_Request__init(distance_interfaces__srv__GetFrontDistance_Request * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
distance_interfaces__srv__GetFrontDistance_Request__fini(distance_interfaces__srv__GetFrontDistance_Request * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
distance_interfaces__srv__GetFrontDistance_Request__are_equal(const distance_interfaces__srv__GetFrontDistance_Request * lhs, const distance_interfaces__srv__GetFrontDistance_Request * rhs)
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
distance_interfaces__srv__GetFrontDistance_Request__copy(
  const distance_interfaces__srv__GetFrontDistance_Request * input,
  distance_interfaces__srv__GetFrontDistance_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

distance_interfaces__srv__GetFrontDistance_Request *
distance_interfaces__srv__GetFrontDistance_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  distance_interfaces__srv__GetFrontDistance_Request * msg = (distance_interfaces__srv__GetFrontDistance_Request *)allocator.allocate(sizeof(distance_interfaces__srv__GetFrontDistance_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(distance_interfaces__srv__GetFrontDistance_Request));
  bool success = distance_interfaces__srv__GetFrontDistance_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
distance_interfaces__srv__GetFrontDistance_Request__destroy(distance_interfaces__srv__GetFrontDistance_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    distance_interfaces__srv__GetFrontDistance_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
distance_interfaces__srv__GetFrontDistance_Request__Sequence__init(distance_interfaces__srv__GetFrontDistance_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  distance_interfaces__srv__GetFrontDistance_Request * data = NULL;

  if (size) {
    data = (distance_interfaces__srv__GetFrontDistance_Request *)allocator.zero_allocate(size, sizeof(distance_interfaces__srv__GetFrontDistance_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = distance_interfaces__srv__GetFrontDistance_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        distance_interfaces__srv__GetFrontDistance_Request__fini(&data[i - 1]);
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
distance_interfaces__srv__GetFrontDistance_Request__Sequence__fini(distance_interfaces__srv__GetFrontDistance_Request__Sequence * array)
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
      distance_interfaces__srv__GetFrontDistance_Request__fini(&array->data[i]);
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

distance_interfaces__srv__GetFrontDistance_Request__Sequence *
distance_interfaces__srv__GetFrontDistance_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  distance_interfaces__srv__GetFrontDistance_Request__Sequence * array = (distance_interfaces__srv__GetFrontDistance_Request__Sequence *)allocator.allocate(sizeof(distance_interfaces__srv__GetFrontDistance_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = distance_interfaces__srv__GetFrontDistance_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
distance_interfaces__srv__GetFrontDistance_Request__Sequence__destroy(distance_interfaces__srv__GetFrontDistance_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    distance_interfaces__srv__GetFrontDistance_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
distance_interfaces__srv__GetFrontDistance_Request__Sequence__are_equal(const distance_interfaces__srv__GetFrontDistance_Request__Sequence * lhs, const distance_interfaces__srv__GetFrontDistance_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!distance_interfaces__srv__GetFrontDistance_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
distance_interfaces__srv__GetFrontDistance_Request__Sequence__copy(
  const distance_interfaces__srv__GetFrontDistance_Request__Sequence * input,
  distance_interfaces__srv__GetFrontDistance_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(distance_interfaces__srv__GetFrontDistance_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    distance_interfaces__srv__GetFrontDistance_Request * data =
      (distance_interfaces__srv__GetFrontDistance_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!distance_interfaces__srv__GetFrontDistance_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          distance_interfaces__srv__GetFrontDistance_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!distance_interfaces__srv__GetFrontDistance_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
distance_interfaces__srv__GetFrontDistance_Response__init(distance_interfaces__srv__GetFrontDistance_Response * msg)
{
  if (!msg) {
    return false;
  }
  // front_distance
  return true;
}

void
distance_interfaces__srv__GetFrontDistance_Response__fini(distance_interfaces__srv__GetFrontDistance_Response * msg)
{
  if (!msg) {
    return;
  }
  // front_distance
}

bool
distance_interfaces__srv__GetFrontDistance_Response__are_equal(const distance_interfaces__srv__GetFrontDistance_Response * lhs, const distance_interfaces__srv__GetFrontDistance_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // front_distance
  if (lhs->front_distance != rhs->front_distance) {
    return false;
  }
  return true;
}

bool
distance_interfaces__srv__GetFrontDistance_Response__copy(
  const distance_interfaces__srv__GetFrontDistance_Response * input,
  distance_interfaces__srv__GetFrontDistance_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // front_distance
  output->front_distance = input->front_distance;
  return true;
}

distance_interfaces__srv__GetFrontDistance_Response *
distance_interfaces__srv__GetFrontDistance_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  distance_interfaces__srv__GetFrontDistance_Response * msg = (distance_interfaces__srv__GetFrontDistance_Response *)allocator.allocate(sizeof(distance_interfaces__srv__GetFrontDistance_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(distance_interfaces__srv__GetFrontDistance_Response));
  bool success = distance_interfaces__srv__GetFrontDistance_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
distance_interfaces__srv__GetFrontDistance_Response__destroy(distance_interfaces__srv__GetFrontDistance_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    distance_interfaces__srv__GetFrontDistance_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
distance_interfaces__srv__GetFrontDistance_Response__Sequence__init(distance_interfaces__srv__GetFrontDistance_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  distance_interfaces__srv__GetFrontDistance_Response * data = NULL;

  if (size) {
    data = (distance_interfaces__srv__GetFrontDistance_Response *)allocator.zero_allocate(size, sizeof(distance_interfaces__srv__GetFrontDistance_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = distance_interfaces__srv__GetFrontDistance_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        distance_interfaces__srv__GetFrontDistance_Response__fini(&data[i - 1]);
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
distance_interfaces__srv__GetFrontDistance_Response__Sequence__fini(distance_interfaces__srv__GetFrontDistance_Response__Sequence * array)
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
      distance_interfaces__srv__GetFrontDistance_Response__fini(&array->data[i]);
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

distance_interfaces__srv__GetFrontDistance_Response__Sequence *
distance_interfaces__srv__GetFrontDistance_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  distance_interfaces__srv__GetFrontDistance_Response__Sequence * array = (distance_interfaces__srv__GetFrontDistance_Response__Sequence *)allocator.allocate(sizeof(distance_interfaces__srv__GetFrontDistance_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = distance_interfaces__srv__GetFrontDistance_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
distance_interfaces__srv__GetFrontDistance_Response__Sequence__destroy(distance_interfaces__srv__GetFrontDistance_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    distance_interfaces__srv__GetFrontDistance_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
distance_interfaces__srv__GetFrontDistance_Response__Sequence__are_equal(const distance_interfaces__srv__GetFrontDistance_Response__Sequence * lhs, const distance_interfaces__srv__GetFrontDistance_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!distance_interfaces__srv__GetFrontDistance_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
distance_interfaces__srv__GetFrontDistance_Response__Sequence__copy(
  const distance_interfaces__srv__GetFrontDistance_Response__Sequence * input,
  distance_interfaces__srv__GetFrontDistance_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(distance_interfaces__srv__GetFrontDistance_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    distance_interfaces__srv__GetFrontDistance_Response * data =
      (distance_interfaces__srv__GetFrontDistance_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!distance_interfaces__srv__GetFrontDistance_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          distance_interfaces__srv__GetFrontDistance_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!distance_interfaces__srv__GetFrontDistance_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
