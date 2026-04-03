// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from obstacle_msgs:msg/ObstacleArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "obstacle_msgs/msg/obstacle_array.h"


#ifndef OBSTACLE_MSGS__MSG__DETAIL__OBSTACLE_ARRAY__STRUCT_H_
#define OBSTACLE_MSGS__MSG__DETAIL__OBSTACLE_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'obstacles'
#include "obstacle_msgs/msg/detail/obstacle__struct.h"

/// Struct defined in msg/ObstacleArray in the package obstacle_msgs.
/**
  * Array of detected obstacles with common header
 */
typedef struct obstacle_msgs__msg__ObstacleArray
{
  std_msgs__msg__Header header;
  obstacle_msgs__msg__Obstacle__Sequence obstacles;
} obstacle_msgs__msg__ObstacleArray;

// Struct for a sequence of obstacle_msgs__msg__ObstacleArray.
typedef struct obstacle_msgs__msg__ObstacleArray__Sequence
{
  obstacle_msgs__msg__ObstacleArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} obstacle_msgs__msg__ObstacleArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // OBSTACLE_MSGS__MSG__DETAIL__OBSTACLE_ARRAY__STRUCT_H_
