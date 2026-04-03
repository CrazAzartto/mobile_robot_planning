// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from obstacle_msgs:msg/Obstacle.idl
// generated code does not contain a copyright notice

#include "obstacle_msgs/msg/detail/obstacle__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_obstacle_msgs
const rosidl_type_hash_t *
obstacle_msgs__msg__Obstacle__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa6, 0x03, 0x85, 0x28, 0xad, 0x7b, 0x85, 0xb3,
      0xe8, 0xef, 0x96, 0xe2, 0xe6, 0x8b, 0x80, 0x69,
      0x43, 0x97, 0x78, 0x6e, 0xc6, 0xd9, 0x45, 0xf0,
      0xa9, 0x89, 0xb9, 0x1a, 0xa3, 0x84, 0x01, 0xb1,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "std_msgs/msg/detail/header__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char obstacle_msgs__msg__Obstacle__TYPE_NAME[] = "obstacle_msgs/msg/Obstacle";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char obstacle_msgs__msg__Obstacle__FIELD_NAME__header[] = "header";
static char obstacle_msgs__msg__Obstacle__FIELD_NAME__x[] = "x";
static char obstacle_msgs__msg__Obstacle__FIELD_NAME__y[] = "y";
static char obstacle_msgs__msg__Obstacle__FIELD_NAME__z[] = "z";
static char obstacle_msgs__msg__Obstacle__FIELD_NAME__width[] = "width";
static char obstacle_msgs__msg__Obstacle__FIELD_NAME__height[] = "height";
static char obstacle_msgs__msg__Obstacle__FIELD_NAME__depth[] = "depth";
static char obstacle_msgs__msg__Obstacle__FIELD_NAME__bbox_x[] = "bbox_x";
static char obstacle_msgs__msg__Obstacle__FIELD_NAME__bbox_y[] = "bbox_y";
static char obstacle_msgs__msg__Obstacle__FIELD_NAME__bbox_w[] = "bbox_w";
static char obstacle_msgs__msg__Obstacle__FIELD_NAME__bbox_h[] = "bbox_h";
static char obstacle_msgs__msg__Obstacle__FIELD_NAME__label[] = "label";
static char obstacle_msgs__msg__Obstacle__FIELD_NAME__confidence[] = "confidence";
static char obstacle_msgs__msg__Obstacle__FIELD_NAME__is_dynamic[] = "is_dynamic";

static rosidl_runtime_c__type_description__Field obstacle_msgs__msg__Obstacle__FIELDS[] = {
  {
    {obstacle_msgs__msg__Obstacle__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {obstacle_msgs__msg__Obstacle__FIELD_NAME__x, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {obstacle_msgs__msg__Obstacle__FIELD_NAME__y, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {obstacle_msgs__msg__Obstacle__FIELD_NAME__z, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {obstacle_msgs__msg__Obstacle__FIELD_NAME__width, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {obstacle_msgs__msg__Obstacle__FIELD_NAME__height, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {obstacle_msgs__msg__Obstacle__FIELD_NAME__depth, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {obstacle_msgs__msg__Obstacle__FIELD_NAME__bbox_x, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {obstacle_msgs__msg__Obstacle__FIELD_NAME__bbox_y, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {obstacle_msgs__msg__Obstacle__FIELD_NAME__bbox_w, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {obstacle_msgs__msg__Obstacle__FIELD_NAME__bbox_h, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {obstacle_msgs__msg__Obstacle__FIELD_NAME__label, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {obstacle_msgs__msg__Obstacle__FIELD_NAME__confidence, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {obstacle_msgs__msg__Obstacle__FIELD_NAME__is_dynamic, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription obstacle_msgs__msg__Obstacle__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
obstacle_msgs__msg__Obstacle__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {obstacle_msgs__msg__Obstacle__TYPE_NAME, 26, 26},
      {obstacle_msgs__msg__Obstacle__FIELDS, 14, 14},
    },
    {obstacle_msgs__msg__Obstacle__REFERENCED_TYPE_DESCRIPTIONS, 2, 2},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Single obstacle detected by camera/LiDAR fusion\n"
  "std_msgs/Header header\n"
  "\n"
  "float32 x          # Position in robot frame (meters)\n"
  "float32 y\n"
  "float32 z\n"
  "\n"
  "float32 width      # Bounding box dimensions (meters)\n"
  "float32 height\n"
  "float32 depth\n"
  "\n"
  "float32 bbox_x     # Image bounding box (pixels)\n"
  "float32 bbox_y\n"
  "float32 bbox_w\n"
  "float32 bbox_h\n"
  "\n"
  "string  label      # Color label / class from HSV segmentation\n"
  "float32 confidence # Detection confidence [0.0, 1.0]\n"
  "bool    is_dynamic # True if identified as moving obstacle";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
obstacle_msgs__msg__Obstacle__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {obstacle_msgs__msg__Obstacle__TYPE_NAME, 26, 26},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 503, 503},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
obstacle_msgs__msg__Obstacle__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[3];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 3, 3};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *obstacle_msgs__msg__Obstacle__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
