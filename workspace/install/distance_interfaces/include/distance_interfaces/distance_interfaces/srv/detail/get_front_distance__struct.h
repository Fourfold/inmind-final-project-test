// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from distance_interfaces:srv/GetFrontDistance.idl
// generated code does not contain a copyright notice

#ifndef DISTANCE_INTERFACES__SRV__DETAIL__GET_FRONT_DISTANCE__STRUCT_H_
#define DISTANCE_INTERFACES__SRV__DETAIL__GET_FRONT_DISTANCE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GetFrontDistance in the package distance_interfaces.
typedef struct distance_interfaces__srv__GetFrontDistance_Request
{
  uint8_t structure_needs_at_least_one_member;
} distance_interfaces__srv__GetFrontDistance_Request;

// Struct for a sequence of distance_interfaces__srv__GetFrontDistance_Request.
typedef struct distance_interfaces__srv__GetFrontDistance_Request__Sequence
{
  distance_interfaces__srv__GetFrontDistance_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} distance_interfaces__srv__GetFrontDistance_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/GetFrontDistance in the package distance_interfaces.
typedef struct distance_interfaces__srv__GetFrontDistance_Response
{
  float front_distance;
} distance_interfaces__srv__GetFrontDistance_Response;

// Struct for a sequence of distance_interfaces__srv__GetFrontDistance_Response.
typedef struct distance_interfaces__srv__GetFrontDistance_Response__Sequence
{
  distance_interfaces__srv__GetFrontDistance_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} distance_interfaces__srv__GetFrontDistance_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DISTANCE_INTERFACES__SRV__DETAIL__GET_FRONT_DISTANCE__STRUCT_H_
