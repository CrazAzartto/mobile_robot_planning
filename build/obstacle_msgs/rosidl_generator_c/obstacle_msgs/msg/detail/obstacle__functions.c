// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from obstacle_msgs:msg/Obstacle.idl
// generated code does not contain a copyright notice
#include "obstacle_msgs/msg/detail/obstacle__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `label`
#include "rosidl_runtime_c/string_functions.h"

bool
obstacle_msgs__msg__Obstacle__init(obstacle_msgs__msg__Obstacle * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    obstacle_msgs__msg__Obstacle__fini(msg);
    return false;
  }
  // x
  // y
  // z
  // width
  // height
  // depth
  // bbox_x
  // bbox_y
  // bbox_w
  // bbox_h
  // label
  if (!rosidl_runtime_c__String__init(&msg->label)) {
    obstacle_msgs__msg__Obstacle__fini(msg);
    return false;
  }
  // confidence
  // is_dynamic
  return true;
}

void
obstacle_msgs__msg__Obstacle__fini(obstacle_msgs__msg__Obstacle * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // x
  // y
  // z
  // width
  // height
  // depth
  // bbox_x
  // bbox_y
  // bbox_w
  // bbox_h
  // label
  rosidl_runtime_c__String__fini(&msg->label);
  // confidence
  // is_dynamic
}

bool
obstacle_msgs__msg__Obstacle__are_equal(const obstacle_msgs__msg__Obstacle * lhs, const obstacle_msgs__msg__Obstacle * rhs)
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
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  // z
  if (lhs->z != rhs->z) {
    return false;
  }
  // width
  if (lhs->width != rhs->width) {
    return false;
  }
  // height
  if (lhs->height != rhs->height) {
    return false;
  }
  // depth
  if (lhs->depth != rhs->depth) {
    return false;
  }
  // bbox_x
  if (lhs->bbox_x != rhs->bbox_x) {
    return false;
  }
  // bbox_y
  if (lhs->bbox_y != rhs->bbox_y) {
    return false;
  }
  // bbox_w
  if (lhs->bbox_w != rhs->bbox_w) {
    return false;
  }
  // bbox_h
  if (lhs->bbox_h != rhs->bbox_h) {
    return false;
  }
  // label
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->label), &(rhs->label)))
  {
    return false;
  }
  // confidence
  if (lhs->confidence != rhs->confidence) {
    return false;
  }
  // is_dynamic
  if (lhs->is_dynamic != rhs->is_dynamic) {
    return false;
  }
  return true;
}

bool
obstacle_msgs__msg__Obstacle__copy(
  const obstacle_msgs__msg__Obstacle * input,
  obstacle_msgs__msg__Obstacle * output)
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
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // z
  output->z = input->z;
  // width
  output->width = input->width;
  // height
  output->height = input->height;
  // depth
  output->depth = input->depth;
  // bbox_x
  output->bbox_x = input->bbox_x;
  // bbox_y
  output->bbox_y = input->bbox_y;
  // bbox_w
  output->bbox_w = input->bbox_w;
  // bbox_h
  output->bbox_h = input->bbox_h;
  // label
  if (!rosidl_runtime_c__String__copy(
      &(input->label), &(output->label)))
  {
    return false;
  }
  // confidence
  output->confidence = input->confidence;
  // is_dynamic
  output->is_dynamic = input->is_dynamic;
  return true;
}

obstacle_msgs__msg__Obstacle *
obstacle_msgs__msg__Obstacle__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  obstacle_msgs__msg__Obstacle * msg = (obstacle_msgs__msg__Obstacle *)allocator.allocate(sizeof(obstacle_msgs__msg__Obstacle), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(obstacle_msgs__msg__Obstacle));
  bool success = obstacle_msgs__msg__Obstacle__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
obstacle_msgs__msg__Obstacle__destroy(obstacle_msgs__msg__Obstacle * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    obstacle_msgs__msg__Obstacle__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
obstacle_msgs__msg__Obstacle__Sequence__init(obstacle_msgs__msg__Obstacle__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  obstacle_msgs__msg__Obstacle * data = NULL;

  if (size) {
    data = (obstacle_msgs__msg__Obstacle *)allocator.zero_allocate(size, sizeof(obstacle_msgs__msg__Obstacle), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = obstacle_msgs__msg__Obstacle__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        obstacle_msgs__msg__Obstacle__fini(&data[i - 1]);
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
obstacle_msgs__msg__Obstacle__Sequence__fini(obstacle_msgs__msg__Obstacle__Sequence * array)
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
      obstacle_msgs__msg__Obstacle__fini(&array->data[i]);
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

obstacle_msgs__msg__Obstacle__Sequence *
obstacle_msgs__msg__Obstacle__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  obstacle_msgs__msg__Obstacle__Sequence * array = (obstacle_msgs__msg__Obstacle__Sequence *)allocator.allocate(sizeof(obstacle_msgs__msg__Obstacle__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = obstacle_msgs__msg__Obstacle__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
obstacle_msgs__msg__Obstacle__Sequence__destroy(obstacle_msgs__msg__Obstacle__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    obstacle_msgs__msg__Obstacle__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
obstacle_msgs__msg__Obstacle__Sequence__are_equal(const obstacle_msgs__msg__Obstacle__Sequence * lhs, const obstacle_msgs__msg__Obstacle__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!obstacle_msgs__msg__Obstacle__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
obstacle_msgs__msg__Obstacle__Sequence__copy(
  const obstacle_msgs__msg__Obstacle__Sequence * input,
  obstacle_msgs__msg__Obstacle__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(obstacle_msgs__msg__Obstacle);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    obstacle_msgs__msg__Obstacle * data =
      (obstacle_msgs__msg__Obstacle *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!obstacle_msgs__msg__Obstacle__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          obstacle_msgs__msg__Obstacle__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!obstacle_msgs__msg__Obstacle__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
