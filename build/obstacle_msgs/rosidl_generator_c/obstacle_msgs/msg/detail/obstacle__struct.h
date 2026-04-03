// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from obstacle_msgs:msg/Obstacle.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "obstacle_msgs/msg/obstacle.h"


#ifndef OBSTACLE_MSGS__MSG__DETAIL__OBSTACLE__STRUCT_H_
#define OBSTACLE_MSGS__MSG__DETAIL__OBSTACLE__STRUCT_H_

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
// Member 'label'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Obstacle in the package obstacle_msgs.
/**
  * Single obstacle detected by camera/LiDAR fusion
 */
typedef struct obstacle_msgs__msg__Obstacle
{
  std_msgs__msg__Header header;
  /// Position in robot frame (meters)
  float x;
  float y;
  float z;
  /// Bounding box dimensions (meters)
  float width;
  float height;
  float depth;
  /// Image bounding box (pixels)
  float bbox_x;
  float bbox_y;
  float bbox_w;
  float bbox_h;
  /// Color label / class from HSV segmentation
  rosidl_runtime_c__String label;
  /// Detection confidence [0.0, 1.0]
  float confidence;
  /// True if identified as moving obstacle
  bool is_dynamic;
} obstacle_msgs__msg__Obstacle;

// Struct for a sequence of obstacle_msgs__msg__Obstacle.
typedef struct obstacle_msgs__msg__Obstacle__Sequence
{
  obstacle_msgs__msg__Obstacle * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} obstacle_msgs__msg__Obstacle__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // OBSTACLE_MSGS__MSG__DETAIL__OBSTACLE__STRUCT_H_
