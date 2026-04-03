// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from obstacle_msgs:msg/Obstacle.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "obstacle_msgs/msg/obstacle.hpp"


#ifndef OBSTACLE_MSGS__MSG__DETAIL__OBSTACLE__BUILDER_HPP_
#define OBSTACLE_MSGS__MSG__DETAIL__OBSTACLE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "obstacle_msgs/msg/detail/obstacle__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace obstacle_msgs
{

namespace msg
{

namespace builder
{

class Init_Obstacle_is_dynamic
{
public:
  explicit Init_Obstacle_is_dynamic(::obstacle_msgs::msg::Obstacle & msg)
  : msg_(msg)
  {}
  ::obstacle_msgs::msg::Obstacle is_dynamic(::obstacle_msgs::msg::Obstacle::_is_dynamic_type arg)
  {
    msg_.is_dynamic = std::move(arg);
    return std::move(msg_);
  }

private:
  ::obstacle_msgs::msg::Obstacle msg_;
};

class Init_Obstacle_confidence
{
public:
  explicit Init_Obstacle_confidence(::obstacle_msgs::msg::Obstacle & msg)
  : msg_(msg)
  {}
  Init_Obstacle_is_dynamic confidence(::obstacle_msgs::msg::Obstacle::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return Init_Obstacle_is_dynamic(msg_);
  }

private:
  ::obstacle_msgs::msg::Obstacle msg_;
};

class Init_Obstacle_label
{
public:
  explicit Init_Obstacle_label(::obstacle_msgs::msg::Obstacle & msg)
  : msg_(msg)
  {}
  Init_Obstacle_confidence label(::obstacle_msgs::msg::Obstacle::_label_type arg)
  {
    msg_.label = std::move(arg);
    return Init_Obstacle_confidence(msg_);
  }

private:
  ::obstacle_msgs::msg::Obstacle msg_;
};

class Init_Obstacle_bbox_h
{
public:
  explicit Init_Obstacle_bbox_h(::obstacle_msgs::msg::Obstacle & msg)
  : msg_(msg)
  {}
  Init_Obstacle_label bbox_h(::obstacle_msgs::msg::Obstacle::_bbox_h_type arg)
  {
    msg_.bbox_h = std::move(arg);
    return Init_Obstacle_label(msg_);
  }

private:
  ::obstacle_msgs::msg::Obstacle msg_;
};

class Init_Obstacle_bbox_w
{
public:
  explicit Init_Obstacle_bbox_w(::obstacle_msgs::msg::Obstacle & msg)
  : msg_(msg)
  {}
  Init_Obstacle_bbox_h bbox_w(::obstacle_msgs::msg::Obstacle::_bbox_w_type arg)
  {
    msg_.bbox_w = std::move(arg);
    return Init_Obstacle_bbox_h(msg_);
  }

private:
  ::obstacle_msgs::msg::Obstacle msg_;
};

class Init_Obstacle_bbox_y
{
public:
  explicit Init_Obstacle_bbox_y(::obstacle_msgs::msg::Obstacle & msg)
  : msg_(msg)
  {}
  Init_Obstacle_bbox_w bbox_y(::obstacle_msgs::msg::Obstacle::_bbox_y_type arg)
  {
    msg_.bbox_y = std::move(arg);
    return Init_Obstacle_bbox_w(msg_);
  }

private:
  ::obstacle_msgs::msg::Obstacle msg_;
};

class Init_Obstacle_bbox_x
{
public:
  explicit Init_Obstacle_bbox_x(::obstacle_msgs::msg::Obstacle & msg)
  : msg_(msg)
  {}
  Init_Obstacle_bbox_y bbox_x(::obstacle_msgs::msg::Obstacle::_bbox_x_type arg)
  {
    msg_.bbox_x = std::move(arg);
    return Init_Obstacle_bbox_y(msg_);
  }

private:
  ::obstacle_msgs::msg::Obstacle msg_;
};

class Init_Obstacle_depth
{
public:
  explicit Init_Obstacle_depth(::obstacle_msgs::msg::Obstacle & msg)
  : msg_(msg)
  {}
  Init_Obstacle_bbox_x depth(::obstacle_msgs::msg::Obstacle::_depth_type arg)
  {
    msg_.depth = std::move(arg);
    return Init_Obstacle_bbox_x(msg_);
  }

private:
  ::obstacle_msgs::msg::Obstacle msg_;
};

class Init_Obstacle_height
{
public:
  explicit Init_Obstacle_height(::obstacle_msgs::msg::Obstacle & msg)
  : msg_(msg)
  {}
  Init_Obstacle_depth height(::obstacle_msgs::msg::Obstacle::_height_type arg)
  {
    msg_.height = std::move(arg);
    return Init_Obstacle_depth(msg_);
  }

private:
  ::obstacle_msgs::msg::Obstacle msg_;
};

class Init_Obstacle_width
{
public:
  explicit Init_Obstacle_width(::obstacle_msgs::msg::Obstacle & msg)
  : msg_(msg)
  {}
  Init_Obstacle_height width(::obstacle_msgs::msg::Obstacle::_width_type arg)
  {
    msg_.width = std::move(arg);
    return Init_Obstacle_height(msg_);
  }

private:
  ::obstacle_msgs::msg::Obstacle msg_;
};

class Init_Obstacle_z
{
public:
  explicit Init_Obstacle_z(::obstacle_msgs::msg::Obstacle & msg)
  : msg_(msg)
  {}
  Init_Obstacle_width z(::obstacle_msgs::msg::Obstacle::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_Obstacle_width(msg_);
  }

private:
  ::obstacle_msgs::msg::Obstacle msg_;
};

class Init_Obstacle_y
{
public:
  explicit Init_Obstacle_y(::obstacle_msgs::msg::Obstacle & msg)
  : msg_(msg)
  {}
  Init_Obstacle_z y(::obstacle_msgs::msg::Obstacle::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_Obstacle_z(msg_);
  }

private:
  ::obstacle_msgs::msg::Obstacle msg_;
};

class Init_Obstacle_x
{
public:
  explicit Init_Obstacle_x(::obstacle_msgs::msg::Obstacle & msg)
  : msg_(msg)
  {}
  Init_Obstacle_y x(::obstacle_msgs::msg::Obstacle::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Obstacle_y(msg_);
  }

private:
  ::obstacle_msgs::msg::Obstacle msg_;
};

class Init_Obstacle_header
{
public:
  Init_Obstacle_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Obstacle_x header(::obstacle_msgs::msg::Obstacle::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Obstacle_x(msg_);
  }

private:
  ::obstacle_msgs::msg::Obstacle msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::obstacle_msgs::msg::Obstacle>()
{
  return obstacle_msgs::msg::builder::Init_Obstacle_header();
}

}  // namespace obstacle_msgs

#endif  // OBSTACLE_MSGS__MSG__DETAIL__OBSTACLE__BUILDER_HPP_
