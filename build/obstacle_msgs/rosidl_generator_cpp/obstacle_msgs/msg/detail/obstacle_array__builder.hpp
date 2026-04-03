// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from obstacle_msgs:msg/ObstacleArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "obstacle_msgs/msg/obstacle_array.hpp"


#ifndef OBSTACLE_MSGS__MSG__DETAIL__OBSTACLE_ARRAY__BUILDER_HPP_
#define OBSTACLE_MSGS__MSG__DETAIL__OBSTACLE_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "obstacle_msgs/msg/detail/obstacle_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace obstacle_msgs
{

namespace msg
{

namespace builder
{

class Init_ObstacleArray_obstacles
{
public:
  explicit Init_ObstacleArray_obstacles(::obstacle_msgs::msg::ObstacleArray & msg)
  : msg_(msg)
  {}
  ::obstacle_msgs::msg::ObstacleArray obstacles(::obstacle_msgs::msg::ObstacleArray::_obstacles_type arg)
  {
    msg_.obstacles = std::move(arg);
    return std::move(msg_);
  }

private:
  ::obstacle_msgs::msg::ObstacleArray msg_;
};

class Init_ObstacleArray_header
{
public:
  Init_ObstacleArray_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ObstacleArray_obstacles header(::obstacle_msgs::msg::ObstacleArray::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ObstacleArray_obstacles(msg_);
  }

private:
  ::obstacle_msgs::msg::ObstacleArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::obstacle_msgs::msg::ObstacleArray>()
{
  return obstacle_msgs::msg::builder::Init_ObstacleArray_header();
}

}  // namespace obstacle_msgs

#endif  // OBSTACLE_MSGS__MSG__DETAIL__OBSTACLE_ARRAY__BUILDER_HPP_
