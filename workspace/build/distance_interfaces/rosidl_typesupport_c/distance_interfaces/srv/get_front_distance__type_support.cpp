// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from distance_interfaces:srv/GetFrontDistance.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "distance_interfaces/srv/detail/get_front_distance__struct.h"
#include "distance_interfaces/srv/detail/get_front_distance__type_support.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace distance_interfaces
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _GetFrontDistance_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetFrontDistance_Request_type_support_ids_t;

static const _GetFrontDistance_Request_type_support_ids_t _GetFrontDistance_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _GetFrontDistance_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetFrontDistance_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetFrontDistance_Request_type_support_symbol_names_t _GetFrontDistance_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, distance_interfaces, srv, GetFrontDistance_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, distance_interfaces, srv, GetFrontDistance_Request)),
  }
};

typedef struct _GetFrontDistance_Request_type_support_data_t
{
  void * data[2];
} _GetFrontDistance_Request_type_support_data_t;

static _GetFrontDistance_Request_type_support_data_t _GetFrontDistance_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetFrontDistance_Request_message_typesupport_map = {
  2,
  "distance_interfaces",
  &_GetFrontDistance_Request_message_typesupport_ids.typesupport_identifier[0],
  &_GetFrontDistance_Request_message_typesupport_symbol_names.symbol_name[0],
  &_GetFrontDistance_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t GetFrontDistance_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetFrontDistance_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace distance_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, distance_interfaces, srv, GetFrontDistance_Request)() {
  return &::distance_interfaces::srv::rosidl_typesupport_c::GetFrontDistance_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "distance_interfaces/srv/detail/get_front_distance__struct.h"
// already included above
// #include "distance_interfaces/srv/detail/get_front_distance__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace distance_interfaces
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _GetFrontDistance_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetFrontDistance_Response_type_support_ids_t;

static const _GetFrontDistance_Response_type_support_ids_t _GetFrontDistance_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _GetFrontDistance_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetFrontDistance_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetFrontDistance_Response_type_support_symbol_names_t _GetFrontDistance_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, distance_interfaces, srv, GetFrontDistance_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, distance_interfaces, srv, GetFrontDistance_Response)),
  }
};

typedef struct _GetFrontDistance_Response_type_support_data_t
{
  void * data[2];
} _GetFrontDistance_Response_type_support_data_t;

static _GetFrontDistance_Response_type_support_data_t _GetFrontDistance_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetFrontDistance_Response_message_typesupport_map = {
  2,
  "distance_interfaces",
  &_GetFrontDistance_Response_message_typesupport_ids.typesupport_identifier[0],
  &_GetFrontDistance_Response_message_typesupport_symbol_names.symbol_name[0],
  &_GetFrontDistance_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t GetFrontDistance_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetFrontDistance_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace distance_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, distance_interfaces, srv, GetFrontDistance_Response)() {
  return &::distance_interfaces::srv::rosidl_typesupport_c::GetFrontDistance_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "distance_interfaces/srv/detail/get_front_distance__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace distance_interfaces
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _GetFrontDistance_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetFrontDistance_type_support_ids_t;

static const _GetFrontDistance_type_support_ids_t _GetFrontDistance_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _GetFrontDistance_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetFrontDistance_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetFrontDistance_type_support_symbol_names_t _GetFrontDistance_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, distance_interfaces, srv, GetFrontDistance)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, distance_interfaces, srv, GetFrontDistance)),
  }
};

typedef struct _GetFrontDistance_type_support_data_t
{
  void * data[2];
} _GetFrontDistance_type_support_data_t;

static _GetFrontDistance_type_support_data_t _GetFrontDistance_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetFrontDistance_service_typesupport_map = {
  2,
  "distance_interfaces",
  &_GetFrontDistance_service_typesupport_ids.typesupport_identifier[0],
  &_GetFrontDistance_service_typesupport_symbol_names.symbol_name[0],
  &_GetFrontDistance_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t GetFrontDistance_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetFrontDistance_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace distance_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, distance_interfaces, srv, GetFrontDistance)() {
  return &::distance_interfaces::srv::rosidl_typesupport_c::GetFrontDistance_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif
