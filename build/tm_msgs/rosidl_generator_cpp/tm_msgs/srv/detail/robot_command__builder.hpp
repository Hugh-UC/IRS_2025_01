// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tm_msgs:srv/RobotCommand.idl
// generated code does not contain a copyright notice

#ifndef TM_MSGS__SRV__DETAIL__ROBOT_COMMAND__BUILDER_HPP_
#define TM_MSGS__SRV__DETAIL__ROBOT_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "tm_msgs/srv/detail/robot_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace tm_msgs
{

namespace srv
{

namespace builder
{

class Init_RobotCommand_Request_command_parameter_string
{
public:
  explicit Init_RobotCommand_Request_command_parameter_string(::tm_msgs::srv::RobotCommand_Request & msg)
  : msg_(msg)
  {}
  ::tm_msgs::srv::RobotCommand_Request command_parameter_string(::tm_msgs::srv::RobotCommand_Request::_command_parameter_string_type arg)
  {
    msg_.command_parameter_string = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tm_msgs::srv::RobotCommand_Request msg_;
};

class Init_RobotCommand_Request_command
{
public:
  Init_RobotCommand_Request_command()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotCommand_Request_command_parameter_string command(::tm_msgs::srv::RobotCommand_Request::_command_type arg)
  {
    msg_.command = std::move(arg);
    return Init_RobotCommand_Request_command_parameter_string(msg_);
  }

private:
  ::tm_msgs::srv::RobotCommand_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::tm_msgs::srv::RobotCommand_Request>()
{
  return tm_msgs::srv::builder::Init_RobotCommand_Request_command();
}

}  // namespace tm_msgs


namespace tm_msgs
{

namespace srv
{

namespace builder
{

class Init_RobotCommand_Response_is_success
{
public:
  Init_RobotCommand_Response_is_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::tm_msgs::srv::RobotCommand_Response is_success(::tm_msgs::srv::RobotCommand_Response::_is_success_type arg)
  {
    msg_.is_success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tm_msgs::srv::RobotCommand_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::tm_msgs::srv::RobotCommand_Response>()
{
  return tm_msgs::srv::builder::Init_RobotCommand_Response_is_success();
}

}  // namespace tm_msgs

#endif  // TM_MSGS__SRV__DETAIL__ROBOT_COMMAND__BUILDER_HPP_
