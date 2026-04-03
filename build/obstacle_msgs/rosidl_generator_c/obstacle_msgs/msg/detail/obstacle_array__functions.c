// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from obstacle_msgs:msg/ObstacleArray.idl
// generated code does not contain a copyright notice
#include "obstacle_msgs/msg/detail/obstacle_array__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `obstacles`
#include "obstacle_msgs/msg/detail/obstacle__functions.h"

bool
obstacle_msgs__msg__ObstacleArray__init(obstacle_msgs__msg__ObstacleArray * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    obstacle_msgs__msg__ObstacleArray__fini(msg);
    return false;
  }
  // obstacles
  if (!obstacle_msgs__msg__Obstacle__Sequence__init(&msg->obstacles, 0)) {
    obstacle_msgs__msg__ObstacleArray__fini(msg);
    return false;
  }
  return true;
}

void
obstacle_msgs__msg__ObstacleArray__fini(obstacle_msgs__msg__ObstacleArray * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // obstacles
  obstacle_msgs__msg__Obstacle__Sequence__fini(&msg->obstacles);
}

bool
obstacle_msgs__msg__ObstacleArray__are_equal(const obstacle_msgs__msg__ObstacleArray * lhs, const obstacle_msgs__msg__ObstacleArray * rhs)
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
  // obstacles
  if (!obstacle_msgs__msg__Obstacle__Sequence__are_equal(
      &(lhs->obstacles), &(rhs->obstacles)))
  {
    return false;
  }
  return true;
}

bool
obstacle_msgs__msg__ObstacleArray__copy(
  const obstacle_msgs__msg__ObstacleArray * input,
  obstacle_msgs__msg__ObstacleArray * output)
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
  // obstacles
  if (!obstacle_msgs__msg__Obstacle__Sequence__copy(
      &(input->obstacles), &(output->obstacles)))
  {
    return false;
  }
  return true;
}

obstacle_msgs__msg__ObstacleArray *
obstacle_msgs__msg__ObstacleArray__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  obstacle_msgs__msg__ObstacleArray * msg = (obstacle_msgs__msg__ObstacleArray *)allocator.allocate(sizeof(obstacle_msgs__msg__ObstacleArray), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(obstacle_msgs__msg__ObstacleArray));
  bool success = obstacle_msgs__msg__ObstacleArray__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
obstacle_msgs__msg__ObstacleArray__destroy(obstacle_msgs__msg__ObstacleArray * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    obstacle_msgs__msg__ObstacleArray__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
obstacle_msgs__msg__ObstacleArray__Sequence__init(obstacle_msgs__msg__ObstacleArray__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  obstacle_msgs__msg__ObstacleArray * data = NULL;

  if (size) {
    data = (obstacle_msgs__msg__ObstacleArray *)allocator.zero_allocate(size, sizeof(obstacle_msgs__msg__ObstacleArray), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = obstacle_msgs__msg__ObstacleArray__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        obstacle_msgs__msg__ObstacleArray__fini(&data[i - 1]);
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
obstacle_msgs__msg__ObstacleArray__Sequence__fini(obstacle_msgs__msg__ObstacleArray__Sequence * array)
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
      obstacle_msgs__msg__ObstacleArray__fini(&array->data[i]);
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

obstacle_msgs__msg__ObstacleArray__Sequence *
obstacle_msgs__msg__ObstacleArray__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  obstacle_msgs__msg__ObstacleArray__Sequence * array = (obstacle_msgs__msg__ObstacleArray__Sequence *)allocator.allocate(sizeof(obstacle_msgs__msg__ObstacleArray__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = obstacle_msgs__msg__ObstacleArray__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
obstacle_msgs__msg__ObstacleArray__Sequence__destroy(obstacle_msgs__msg__ObstacleArray__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    obstacle_msgs__msg__ObstacleArray__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
obstacle_msgs__msg__ObstacleArray__Sequence__are_equal(const obstacle_msgs__msg__ObstacleArray__Sequence * lhs, const obstacle_msgs__msg__ObstacleArray__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!obstacle_msgs__msg__ObstacleArray__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
obstacle_msgs__msg__ObstacleArray__Sequence__copy(
  const obstacle_msgs__msg__ObstacleArray__Sequence * input,
  obstacle_msgs__msg__ObstacleArray__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(obstacle_msgs__msg__ObstacleArray);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    obstacle_msgs__msg__ObstacleArray * data =
      (obstacle_msgs__msg__ObstacleArray *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!obstacle_msgs__msg__ObstacleArray__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          obstacle_msgs__msg__ObstacleArray__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!obstacle_msgs__msg__ObstacleArray__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
