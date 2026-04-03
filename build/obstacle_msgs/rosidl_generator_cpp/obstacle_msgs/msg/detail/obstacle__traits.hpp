// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from obstacle_msgs:msg/Obstacle.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "obstacle_msgs/msg/obstacle.hpp"


#ifndef OBSTACLE_MSGS__MSG__DETAIL__OBSTACLE__TRAITS_HPP_
#define OBSTACLE_MSGS__MSG__DETAIL__OBSTACLE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "obstacle_msgs/msg/detail/obstacle__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace obstacle_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Obstacle & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: z
  {
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << ", ";
  }

  // member: width
  {
    out << "width: ";
    rosidl_generator_traits::value_to_yaml(msg.width, out);
    out << ", ";
  }

  // member: height
  {
    out << "height: ";
    rosidl_generator_traits::value_to_yaml(msg.height, out);
    out << ", ";
  }

  // member: depth
  {
    out << "depth: ";
    rosidl_generator_traits::value_to_yaml(msg.depth, out);
    out << ", ";
  }

  // member: bbox_x
  {
    out << "bbox_x: ";
    rosidl_generator_traits::value_to_yaml(msg.bbox_x, out);
    out << ", ";
  }

  // member: bbox_y
  {
    out << "bbox_y: ";
    rosidl_generator_traits::value_to_yaml(msg.bbox_y, out);
    out << ", ";
  }

  // member: bbox_w
  {
    out << "bbox_w: ";
    rosidl_generator_traits::value_to_yaml(msg.bbox_w, out);
    out << ", ";
  }

  // member: bbox_h
  {
    out << "bbox_h: ";
    rosidl_generator_traits::value_to_yaml(msg.bbox_h, out);
    out << ", ";
  }

  // member: label
  {
    out << "label: ";
    rosidl_generator_traits::value_to_yaml(msg.label, out);
    out << ", ";
  }

  // member: confidence
  {
    out << "confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence, out);
    out << ", ";
  }

  // member: is_dynamic
  {
    out << "is_dynamic: ";
    rosidl_generator_traits::value_to_yaml(msg.is_dynamic, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Obstacle & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << "\n";
  }

  // member: width
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "width: ";
    rosidl_generator_traits::value_to_yaml(msg.width, out);
    out << "\n";
  }

  // member: height
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "height: ";
    rosidl_generator_traits::value_to_yaml(msg.height, out);
    out << "\n";
  }

  // member: depth
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "depth: ";
    rosidl_generator_traits::value_to_yaml(msg.depth, out);
    out << "\n";
  }

  // member: bbox_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bbox_x: ";
    rosidl_generator_traits::value_to_yaml(msg.bbox_x, out);
    out << "\n";
  }

  // member: bbox_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bbox_y: ";
    rosidl_generator_traits::value_to_yaml(msg.bbox_y, out);
    out << "\n";
  }

  // member: bbox_w
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bbox_w: ";
    rosidl_generator_traits::value_to_yaml(msg.bbox_w, out);
    out << "\n";
  }

  // member: bbox_h
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bbox_h: ";
    rosidl_generator_traits::value_to_yaml(msg.bbox_h, out);
    out << "\n";
  }

  // member: label
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "label: ";
    rosidl_generator_traits::value_to_yaml(msg.label, out);
    out << "\n";
  }

  // member: confidence
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence, out);
    out << "\n";
  }

  // member: is_dynamic
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_dynamic: ";
    rosidl_generator_traits::value_to_yaml(msg.is_dynamic, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Obstacle & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace obstacle_msgs

namespace rosidl_generator_traits
{

[[deprecated("use obstacle_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const obstacle_msgs::msg::Obstacle & msg,
  std::ostream & out, size_t indentation = 0)
{
  obstacle_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use obstacle_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const obstacle_msgs::msg::Obstacle & msg)
{
  return obstacle_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<obstacle_msgs::msg::Obstacle>()
{
  return "obstacle_msgs::msg::Obstacle";
}

template<>
inline const char * name<obstacle_msgs::msg::Obstacle>()
{
  return "obstacle_msgs/msg/Obstacle";
}

template<>
struct has_fixed_size<obstacle_msgs::msg::Obstacle>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<obstacle_msgs::msg::Obstacle>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<obstacle_msgs::msg::Obstacle>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // OBSTACLE_MSGS__MSG__DETAIL__OBSTACLE__TRAITS_HPP_
