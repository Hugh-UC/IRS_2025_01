// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from tm_msgs:srv/RobotCommand.idl
// generated code does not contain a copyright notice

#ifndef TM_MSGS__SRV__DETAIL__ROBOT_COMMAND__STRUCT_H_
#define TM_MSGS__SRV__DETAIL__ROBOT_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'command'
// Member 'command_parameter_string'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/RobotCommand in the package tm_msgs.
typedef struct tm_msgs__srv__RobotCommand_Request
{
  rosidl_runtime_c__String command;
  rosidl_runtime_c__String command_parameter_string;
} tm_msgs__srv__RobotCommand_Request;

// Struct for a sequence of tm_msgs__srv__RobotCommand_Request.
typedef struct tm_msgs__srv__RobotCommand_Request__Sequence
{
  tm_msgs__srv__RobotCommand_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tm_msgs__srv__RobotCommand_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/RobotCommand in the package tm_msgs.
typedef struct tm_msgs__srv__RobotCommand_Response
{
  bool is_success;
} tm_msgs__srv__RobotCommand_Response;

// Struct for a sequence of tm_msgs__srv__RobotCommand_Response.
typedef struct tm_msgs__srv__RobotCommand_Response__Sequence
{
  tm_msgs__srv__RobotCommand_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tm_msgs__srv__RobotCommand_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TM_MSGS__SRV__DETAIL__ROBOT_COMMAND__STRUCT_H_
