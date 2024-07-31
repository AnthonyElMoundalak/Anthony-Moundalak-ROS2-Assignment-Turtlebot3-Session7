// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from my_interface:srv/FindClosestWall.idl
// generated code does not contain a copyright notice

#ifndef MY_INTERFACE__SRV__DETAIL__FIND_CLOSEST_WALL__BUILDER_HPP_
#define MY_INTERFACE__SRV__DETAIL__FIND_CLOSEST_WALL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "my_interface/srv/detail/find_closest_wall__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace my_interface
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_interface::srv::FindClosestWall_Request>()
{
  return ::my_interface::srv::FindClosestWall_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace my_interface


namespace my_interface
{

namespace srv
{

namespace builder
{

class Init_FindClosestWall_Response_success
{
public:
  Init_FindClosestWall_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::my_interface::srv::FindClosestWall_Response success(::my_interface::srv::FindClosestWall_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_interface::srv::FindClosestWall_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_interface::srv::FindClosestWall_Response>()
{
  return my_interface::srv::builder::Init_FindClosestWall_Response_success();
}

}  // namespace my_interface

#endif  // MY_INTERFACE__SRV__DETAIL__FIND_CLOSEST_WALL__BUILDER_HPP_
