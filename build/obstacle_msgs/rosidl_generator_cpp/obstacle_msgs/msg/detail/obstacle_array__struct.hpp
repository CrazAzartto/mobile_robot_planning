// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from obstacle_msgs:msg/ObstacleArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "obstacle_msgs/msg/obstacle_array.hpp"


#ifndef OBSTACLE_MSGS__MSG__DETAIL__OBSTACLE_ARRAY__STRUCT_HPP_
#define OBSTACLE_MSGS__MSG__DETAIL__OBSTACLE_ARRAY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'obstacles'
#include "obstacle_msgs/msg/detail/obstacle__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__obstacle_msgs__msg__ObstacleArray __attribute__((deprecated))
#else
# define DEPRECATED__obstacle_msgs__msg__ObstacleArray __declspec(deprecated)
#endif

namespace obstacle_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ObstacleArray_
{
  using Type = ObstacleArray_<ContainerAllocator>;

  explicit ObstacleArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit ObstacleArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _obstacles_type =
    std::vector<obstacle_msgs::msg::Obstacle_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<obstacle_msgs::msg::Obstacle_<ContainerAllocator>>>;
  _obstacles_type obstacles;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__obstacles(
    const std::vector<obstacle_msgs::msg::Obstacle_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<obstacle_msgs::msg::Obstacle_<ContainerAllocator>>> & _arg)
  {
    this->obstacles = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    obstacle_msgs::msg::ObstacleArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const obstacle_msgs::msg::ObstacleArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<obstacle_msgs::msg::ObstacleArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<obstacle_msgs::msg::ObstacleArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      obstacle_msgs::msg::ObstacleArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<obstacle_msgs::msg::ObstacleArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      obstacle_msgs::msg::ObstacleArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<obstacle_msgs::msg::ObstacleArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<obstacle_msgs::msg::ObstacleArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<obstacle_msgs::msg::ObstacleArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__obstacle_msgs__msg__ObstacleArray
    std::shared_ptr<obstacle_msgs::msg::ObstacleArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__obstacle_msgs__msg__ObstacleArray
    std::shared_ptr<obstacle_msgs::msg::ObstacleArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ObstacleArray_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->obstacles != other.obstacles) {
      return false;
    }
    return true;
  }
  bool operator!=(const ObstacleArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ObstacleArray_

// alias to use template instance with default allocator
using ObstacleArray =
  obstacle_msgs::msg::ObstacleArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace obstacle_msgs

#endif  // OBSTACLE_MSGS__MSG__DETAIL__OBSTACLE_ARRAY__STRUCT_HPP_
