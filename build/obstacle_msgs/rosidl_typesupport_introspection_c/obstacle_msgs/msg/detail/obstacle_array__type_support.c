// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from obstacle_msgs:msg/ObstacleArray.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "obstacle_msgs/msg/detail/obstacle_array__rosidl_typesupport_introspection_c.h"
#include "obstacle_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "obstacle_msgs/msg/detail/obstacle_array__functions.h"
#include "obstacle_msgs/msg/detail/obstacle_array__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `obstacles`
#include "obstacle_msgs/msg/obstacle.h"
// Member `obstacles`
#include "obstacle_msgs/msg/detail/obstacle__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void obstacle_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__ObstacleArray_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  obstacle_msgs__msg__ObstacleArray__init(message_memory);
}

void obstacle_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__ObstacleArray_fini_function(void * message_memory)
{
  obstacle_msgs__msg__ObstacleArray__fini(message_memory);
}

size_t obstacle_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__size_function__ObstacleArray__obstacles(
  const void * untyped_member)
{
  const obstacle_msgs__msg__Obstacle__Sequence * member =
    (const obstacle_msgs__msg__Obstacle__Sequence *)(untyped_member);
  return member->size;
}

const void * obstacle_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__get_const_function__ObstacleArray__obstacles(
  const void * untyped_member, size_t index)
{
  const obstacle_msgs__msg__Obstacle__Sequence * member =
    (const obstacle_msgs__msg__Obstacle__Sequence *)(untyped_member);
  return &member->data[index];
}

void * obstacle_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__get_function__ObstacleArray__obstacles(
  void * untyped_member, size_t index)
{
  obstacle_msgs__msg__Obstacle__Sequence * member =
    (obstacle_msgs__msg__Obstacle__Sequence *)(untyped_member);
  return &member->data[index];
}

void obstacle_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__fetch_function__ObstacleArray__obstacles(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const obstacle_msgs__msg__Obstacle * item =
    ((const obstacle_msgs__msg__Obstacle *)
    obstacle_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__get_const_function__ObstacleArray__obstacles(untyped_member, index));
  obstacle_msgs__msg__Obstacle * value =
    (obstacle_msgs__msg__Obstacle *)(untyped_value);
  *value = *item;
}

void obstacle_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__assign_function__ObstacleArray__obstacles(
  void * untyped_member, size_t index, const void * untyped_value)
{
  obstacle_msgs__msg__Obstacle * item =
    ((obstacle_msgs__msg__Obstacle *)
    obstacle_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__get_function__ObstacleArray__obstacles(untyped_member, index));
  const obstacle_msgs__msg__Obstacle * value =
    (const obstacle_msgs__msg__Obstacle *)(untyped_value);
  *item = *value;
}

bool obstacle_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__resize_function__ObstacleArray__obstacles(
  void * untyped_member, size_t size)
{
  obstacle_msgs__msg__Obstacle__Sequence * member =
    (obstacle_msgs__msg__Obstacle__Sequence *)(untyped_member);
  obstacle_msgs__msg__Obstacle__Sequence__fini(member);
  return obstacle_msgs__msg__Obstacle__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember obstacle_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__ObstacleArray_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(obstacle_msgs__msg__ObstacleArray, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "obstacles",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(obstacle_msgs__msg__ObstacleArray, obstacles),  // bytes offset in struct
    NULL,  // default value
    obstacle_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__size_function__ObstacleArray__obstacles,  // size() function pointer
    obstacle_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__get_const_function__ObstacleArray__obstacles,  // get_const(index) function pointer
    obstacle_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__get_function__ObstacleArray__obstacles,  // get(index) function pointer
    obstacle_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__fetch_function__ObstacleArray__obstacles,  // fetch(index, &value) function pointer
    obstacle_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__assign_function__ObstacleArray__obstacles,  // assign(index, value) function pointer
    obstacle_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__resize_function__ObstacleArray__obstacles  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers obstacle_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__ObstacleArray_message_members = {
  "obstacle_msgs__msg",  // message namespace
  "ObstacleArray",  // message name
  2,  // number of fields
  sizeof(obstacle_msgs__msg__ObstacleArray),
  false,  // has_any_key_member_
  obstacle_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__ObstacleArray_message_member_array,  // message members
  obstacle_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__ObstacleArray_init_function,  // function to initialize message memory (memory has to be allocated)
  obstacle_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__ObstacleArray_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t obstacle_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__ObstacleArray_message_type_support_handle = {
  0,
  &obstacle_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__ObstacleArray_message_members,
  get_message_typesupport_handle_function,
  &obstacle_msgs__msg__ObstacleArray__get_type_hash,
  &obstacle_msgs__msg__ObstacleArray__get_type_description,
  &obstacle_msgs__msg__ObstacleArray__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_obstacle_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, obstacle_msgs, msg, ObstacleArray)() {
  obstacle_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__ObstacleArray_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  obstacle_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__ObstacleArray_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, obstacle_msgs, msg, Obstacle)();
  if (!obstacle_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__ObstacleArray_message_type_support_handle.typesupport_identifier) {
    obstacle_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__ObstacleArray_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &obstacle_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__ObstacleArray_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
