// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from tm_msgs:srv/RobotCommand.idl
// generated code does not contain a copyright notice

#ifndef TM_MSGS__SRV__DETAIL__ROBOT_COMMAND__TRAITS_HPP_
#define TM_MSGS__SRV__DETAIL__ROBOT_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "tm_msgs/srv/detail/robot_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace tm_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const RobotCommand_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: command
  {
    out << "command: ";
    rosidl_generator_traits::value_to_yaml(msg.command, out);
    out << ", ";
  }

  // member: command_parameter_string
  {
    out << "command_parameter_string: ";
    rosidl_generator_traits::value_to_yaml(msg.command_parameter_string, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RobotCommand_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: command
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "command: ";
    rosidl_generator_traits::value_to_yaml(msg.command, out);
    out << "\n";
  }

  // member: command_parameter_string
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "command_parameter_string: ";
    rosidl_generator_traits::value_to_yaml(msg.command_parameter_string, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RobotCommand_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace tm_msgs

namespace rosidl_generator_traits
{

[[deprecated("use tm_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const tm_msgs::srv::RobotCommand_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  tm_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use tm_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const tm_msgs::srv::RobotCommand_Request & msg)
{
  return tm_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<tm_msgs::srv::RobotCommand_Request>()
{
  return "tm_msgs::srv::RobotCommand_Request";
}

template<>
inline const char * name<tm_msgs::srv::RobotCommand_Request>()
{
  return "tm_msgs/srv/RobotCommand_Request";
}

template<>
struct has_fixed_size<tm_msgs::srv::RobotCommand_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<tm_msgs::srv::RobotCommand_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<tm_msgs::srv::RobotCommand_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace tm_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const RobotCommand_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: is_success
  {
    out << "is_success: ";
    rosidl_generator_traits::value_to_yaml(msg.is_success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RobotCommand_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: is_success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_success: ";
    rosidl_generator_traits::value_to_yaml(msg.is_success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RobotCommand_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace tm_msgs

namespace rosidl_generator_traits
{

[[deprecated("use tm_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const tm_msgs::srv::RobotCommand_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  tm_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use tm_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const tm_msgs::srv::RobotCommand_Response & msg)
{
  return tm_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<tm_msgs::srv::RobotCommand_Response>()
{
  return "tm_msgs::srv::RobotCommand_Response";
}

template<>
inline const char * name<tm_msgs::srv::RobotCommand_Response>()
{
  return "tm_msgs/srv/RobotCommand_Response";
}

template<>
struct has_fixed_size<tm_msgs::srv::RobotCommand_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<tm_msgs::srv::RobotCommand_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<tm_msgs::srv::RobotCommand_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<tm_msgs::srv::RobotCommand>()
{
  return "tm_msgs::srv::RobotCommand";
}

template<>
inline const char * name<tm_msgs::srv::RobotCommand>()
{
  return "tm_msgs/srv/RobotCommand";
}

template<>
struct has_fixed_size<tm_msgs::srv::RobotCommand>
  : std::integral_constant<
    bool,
    has_fixed_size<tm_msgs::srv::RobotCommand_Request>::value &&
    has_fixed_size<tm_msgs::srv::RobotCommand_Response>::value
  >
{
};

template<>
struct has_bounded_size<tm_msgs::srv::RobotCommand>
  : std::integral_constant<
    bool,
    has_bounded_size<tm_msgs::srv::RobotCommand_Request>::value &&
    has_bounded_size<tm_msgs::srv::RobotCommand_Response>::value
  >
{
};

template<>
struct is_service<tm_msgs::srv::RobotCommand>
  : std::true_type
{
};

template<>
struct is_service_request<tm_msgs::srv::RobotCommand_Request>
  : std::true_type
{
};

template<>
struct is_service_response<tm_msgs::srv::RobotCommand_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // TM_MSGS__SRV__DETAIL__ROBOT_COMMAND__TRAITS_HPP_
