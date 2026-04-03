// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from obstacle_msgs:msg/ObstacleArray.idl
// generated code does not contain a copyright notice
#ifndef OBSTACLE_MSGS__MSG__DETAIL__OBSTACLE_ARRAY__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define OBSTACLE_MSGS__MSG__DETAIL__OBSTACLE_ARRAY__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "obstacle_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "obstacle_msgs/msg/detail/obstacle_array__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_obstacle_msgs
bool cdr_serialize_obstacle_msgs__msg__ObstacleArray(
  const obstacle_msgs__msg__ObstacleArray * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_obstacle_msgs
bool cdr_deserialize_obstacle_msgs__msg__ObstacleArray(
  eprosima::fastcdr::Cdr &,
  obstacle_msgs__msg__ObstacleArray * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_obstacle_msgs
size_t get_serialized_size_obstacle_msgs__msg__ObstacleArray(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_obstacle_msgs
size_t max_serialized_size_obstacle_msgs__msg__ObstacleArray(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_obstacle_msgs
bool cdr_serialize_key_obstacle_msgs__msg__ObstacleArray(
  const obstacle_msgs__msg__ObstacleArray * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_obstacle_msgs
size_t get_serialized_size_key_obstacle_msgs__msg__ObstacleArray(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_obstacle_msgs
size_t max_serialized_size_key_obstacle_msgs__msg__ObstacleArray(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_obstacle_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, obstacle_msgs, msg, ObstacleArray)();

#ifdef __cplusplus
}
#endif

#endif  // OBSTACLE_MSGS__MSG__DETAIL__OBSTACLE_ARRAY__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
