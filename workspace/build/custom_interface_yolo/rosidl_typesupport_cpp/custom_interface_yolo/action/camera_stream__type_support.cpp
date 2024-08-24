// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from custom_interface_yolo:action/CameraStream.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "custom_interface_yolo/action/detail/camera_stream__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace custom_interface_yolo
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _CameraStream_Goal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _CameraStream_Goal_type_support_ids_t;

static const _CameraStream_Goal_type_support_ids_t _CameraStream_Goal_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _CameraStream_Goal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _CameraStream_Goal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _CameraStream_Goal_type_support_symbol_names_t _CameraStream_Goal_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, custom_interface_yolo, action, CameraStream_Goal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, custom_interface_yolo, action, CameraStream_Goal)),
  }
};

typedef struct _CameraStream_Goal_type_support_data_t
{
  void * data[2];
} _CameraStream_Goal_type_support_data_t;

static _CameraStream_Goal_type_support_data_t _CameraStream_Goal_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _CameraStream_Goal_message_typesupport_map = {
  2,
  "custom_interface_yolo",
  &_CameraStream_Goal_message_typesupport_ids.typesupport_identifier[0],
  &_CameraStream_Goal_message_typesupport_symbol_names.symbol_name[0],
  &_CameraStream_Goal_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t CameraStream_Goal_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_CameraStream_Goal_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace custom_interface_yolo

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<custom_interface_yolo::action::CameraStream_Goal>()
{
  return &::custom_interface_yolo::action::rosidl_typesupport_cpp::CameraStream_Goal_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, custom_interface_yolo, action, CameraStream_Goal)() {
  return get_message_type_support_handle<custom_interface_yolo::action::CameraStream_Goal>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "custom_interface_yolo/action/detail/camera_stream__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace custom_interface_yolo
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _CameraStream_Result_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _CameraStream_Result_type_support_ids_t;

static const _CameraStream_Result_type_support_ids_t _CameraStream_Result_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _CameraStream_Result_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _CameraStream_Result_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _CameraStream_Result_type_support_symbol_names_t _CameraStream_Result_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, custom_interface_yolo, action, CameraStream_Result)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, custom_interface_yolo, action, CameraStream_Result)),
  }
};

typedef struct _CameraStream_Result_type_support_data_t
{
  void * data[2];
} _CameraStream_Result_type_support_data_t;

static _CameraStream_Result_type_support_data_t _CameraStream_Result_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _CameraStream_Result_message_typesupport_map = {
  2,
  "custom_interface_yolo",
  &_CameraStream_Result_message_typesupport_ids.typesupport_identifier[0],
  &_CameraStream_Result_message_typesupport_symbol_names.symbol_name[0],
  &_CameraStream_Result_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t CameraStream_Result_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_CameraStream_Result_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace custom_interface_yolo

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<custom_interface_yolo::action::CameraStream_Result>()
{
  return &::custom_interface_yolo::action::rosidl_typesupport_cpp::CameraStream_Result_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, custom_interface_yolo, action, CameraStream_Result)() {
  return get_message_type_support_handle<custom_interface_yolo::action::CameraStream_Result>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "custom_interface_yolo/action/detail/camera_stream__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace custom_interface_yolo
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _CameraStream_Feedback_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _CameraStream_Feedback_type_support_ids_t;

static const _CameraStream_Feedback_type_support_ids_t _CameraStream_Feedback_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _CameraStream_Feedback_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _CameraStream_Feedback_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _CameraStream_Feedback_type_support_symbol_names_t _CameraStream_Feedback_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, custom_interface_yolo, action, CameraStream_Feedback)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, custom_interface_yolo, action, CameraStream_Feedback)),
  }
};

typedef struct _CameraStream_Feedback_type_support_data_t
{
  void * data[2];
} _CameraStream_Feedback_type_support_data_t;

static _CameraStream_Feedback_type_support_data_t _CameraStream_Feedback_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _CameraStream_Feedback_message_typesupport_map = {
  2,
  "custom_interface_yolo",
  &_CameraStream_Feedback_message_typesupport_ids.typesupport_identifier[0],
  &_CameraStream_Feedback_message_typesupport_symbol_names.symbol_name[0],
  &_CameraStream_Feedback_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t CameraStream_Feedback_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_CameraStream_Feedback_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace custom_interface_yolo

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<custom_interface_yolo::action::CameraStream_Feedback>()
{
  return &::custom_interface_yolo::action::rosidl_typesupport_cpp::CameraStream_Feedback_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, custom_interface_yolo, action, CameraStream_Feedback)() {
  return get_message_type_support_handle<custom_interface_yolo::action::CameraStream_Feedback>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "custom_interface_yolo/action/detail/camera_stream__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace custom_interface_yolo
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _CameraStream_SendGoal_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _CameraStream_SendGoal_Request_type_support_ids_t;

static const _CameraStream_SendGoal_Request_type_support_ids_t _CameraStream_SendGoal_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _CameraStream_SendGoal_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _CameraStream_SendGoal_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _CameraStream_SendGoal_Request_type_support_symbol_names_t _CameraStream_SendGoal_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, custom_interface_yolo, action, CameraStream_SendGoal_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, custom_interface_yolo, action, CameraStream_SendGoal_Request)),
  }
};

typedef struct _CameraStream_SendGoal_Request_type_support_data_t
{
  void * data[2];
} _CameraStream_SendGoal_Request_type_support_data_t;

static _CameraStream_SendGoal_Request_type_support_data_t _CameraStream_SendGoal_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _CameraStream_SendGoal_Request_message_typesupport_map = {
  2,
  "custom_interface_yolo",
  &_CameraStream_SendGoal_Request_message_typesupport_ids.typesupport_identifier[0],
  &_CameraStream_SendGoal_Request_message_typesupport_symbol_names.symbol_name[0],
  &_CameraStream_SendGoal_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t CameraStream_SendGoal_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_CameraStream_SendGoal_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace custom_interface_yolo

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<custom_interface_yolo::action::CameraStream_SendGoal_Request>()
{
  return &::custom_interface_yolo::action::rosidl_typesupport_cpp::CameraStream_SendGoal_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, custom_interface_yolo, action, CameraStream_SendGoal_Request)() {
  return get_message_type_support_handle<custom_interface_yolo::action::CameraStream_SendGoal_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "custom_interface_yolo/action/detail/camera_stream__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace custom_interface_yolo
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _CameraStream_SendGoal_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _CameraStream_SendGoal_Response_type_support_ids_t;

static const _CameraStream_SendGoal_Response_type_support_ids_t _CameraStream_SendGoal_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _CameraStream_SendGoal_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _CameraStream_SendGoal_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _CameraStream_SendGoal_Response_type_support_symbol_names_t _CameraStream_SendGoal_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, custom_interface_yolo, action, CameraStream_SendGoal_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, custom_interface_yolo, action, CameraStream_SendGoal_Response)),
  }
};

typedef struct _CameraStream_SendGoal_Response_type_support_data_t
{
  void * data[2];
} _CameraStream_SendGoal_Response_type_support_data_t;

static _CameraStream_SendGoal_Response_type_support_data_t _CameraStream_SendGoal_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _CameraStream_SendGoal_Response_message_typesupport_map = {
  2,
  "custom_interface_yolo",
  &_CameraStream_SendGoal_Response_message_typesupport_ids.typesupport_identifier[0],
  &_CameraStream_SendGoal_Response_message_typesupport_symbol_names.symbol_name[0],
  &_CameraStream_SendGoal_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t CameraStream_SendGoal_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_CameraStream_SendGoal_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace custom_interface_yolo

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<custom_interface_yolo::action::CameraStream_SendGoal_Response>()
{
  return &::custom_interface_yolo::action::rosidl_typesupport_cpp::CameraStream_SendGoal_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, custom_interface_yolo, action, CameraStream_SendGoal_Response)() {
  return get_message_type_support_handle<custom_interface_yolo::action::CameraStream_SendGoal_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "custom_interface_yolo/action/detail/camera_stream__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace custom_interface_yolo
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _CameraStream_SendGoal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _CameraStream_SendGoal_type_support_ids_t;

static const _CameraStream_SendGoal_type_support_ids_t _CameraStream_SendGoal_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _CameraStream_SendGoal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _CameraStream_SendGoal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _CameraStream_SendGoal_type_support_symbol_names_t _CameraStream_SendGoal_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, custom_interface_yolo, action, CameraStream_SendGoal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, custom_interface_yolo, action, CameraStream_SendGoal)),
  }
};

typedef struct _CameraStream_SendGoal_type_support_data_t
{
  void * data[2];
} _CameraStream_SendGoal_type_support_data_t;

static _CameraStream_SendGoal_type_support_data_t _CameraStream_SendGoal_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _CameraStream_SendGoal_service_typesupport_map = {
  2,
  "custom_interface_yolo",
  &_CameraStream_SendGoal_service_typesupport_ids.typesupport_identifier[0],
  &_CameraStream_SendGoal_service_typesupport_symbol_names.symbol_name[0],
  &_CameraStream_SendGoal_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t CameraStream_SendGoal_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_CameraStream_SendGoal_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace custom_interface_yolo

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<custom_interface_yolo::action::CameraStream_SendGoal>()
{
  return &::custom_interface_yolo::action::rosidl_typesupport_cpp::CameraStream_SendGoal_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "custom_interface_yolo/action/detail/camera_stream__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace custom_interface_yolo
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _CameraStream_GetResult_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _CameraStream_GetResult_Request_type_support_ids_t;

static const _CameraStream_GetResult_Request_type_support_ids_t _CameraStream_GetResult_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _CameraStream_GetResult_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _CameraStream_GetResult_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _CameraStream_GetResult_Request_type_support_symbol_names_t _CameraStream_GetResult_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, custom_interface_yolo, action, CameraStream_GetResult_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, custom_interface_yolo, action, CameraStream_GetResult_Request)),
  }
};

typedef struct _CameraStream_GetResult_Request_type_support_data_t
{
  void * data[2];
} _CameraStream_GetResult_Request_type_support_data_t;

static _CameraStream_GetResult_Request_type_support_data_t _CameraStream_GetResult_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _CameraStream_GetResult_Request_message_typesupport_map = {
  2,
  "custom_interface_yolo",
  &_CameraStream_GetResult_Request_message_typesupport_ids.typesupport_identifier[0],
  &_CameraStream_GetResult_Request_message_typesupport_symbol_names.symbol_name[0],
  &_CameraStream_GetResult_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t CameraStream_GetResult_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_CameraStream_GetResult_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace custom_interface_yolo

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<custom_interface_yolo::action::CameraStream_GetResult_Request>()
{
  return &::custom_interface_yolo::action::rosidl_typesupport_cpp::CameraStream_GetResult_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, custom_interface_yolo, action, CameraStream_GetResult_Request)() {
  return get_message_type_support_handle<custom_interface_yolo::action::CameraStream_GetResult_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "custom_interface_yolo/action/detail/camera_stream__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace custom_interface_yolo
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _CameraStream_GetResult_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _CameraStream_GetResult_Response_type_support_ids_t;

static const _CameraStream_GetResult_Response_type_support_ids_t _CameraStream_GetResult_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _CameraStream_GetResult_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _CameraStream_GetResult_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _CameraStream_GetResult_Response_type_support_symbol_names_t _CameraStream_GetResult_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, custom_interface_yolo, action, CameraStream_GetResult_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, custom_interface_yolo, action, CameraStream_GetResult_Response)),
  }
};

typedef struct _CameraStream_GetResult_Response_type_support_data_t
{
  void * data[2];
} _CameraStream_GetResult_Response_type_support_data_t;

static _CameraStream_GetResult_Response_type_support_data_t _CameraStream_GetResult_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _CameraStream_GetResult_Response_message_typesupport_map = {
  2,
  "custom_interface_yolo",
  &_CameraStream_GetResult_Response_message_typesupport_ids.typesupport_identifier[0],
  &_CameraStream_GetResult_Response_message_typesupport_symbol_names.symbol_name[0],
  &_CameraStream_GetResult_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t CameraStream_GetResult_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_CameraStream_GetResult_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace custom_interface_yolo

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<custom_interface_yolo::action::CameraStream_GetResult_Response>()
{
  return &::custom_interface_yolo::action::rosidl_typesupport_cpp::CameraStream_GetResult_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, custom_interface_yolo, action, CameraStream_GetResult_Response)() {
  return get_message_type_support_handle<custom_interface_yolo::action::CameraStream_GetResult_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "custom_interface_yolo/action/detail/camera_stream__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace custom_interface_yolo
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _CameraStream_GetResult_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _CameraStream_GetResult_type_support_ids_t;

static const _CameraStream_GetResult_type_support_ids_t _CameraStream_GetResult_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _CameraStream_GetResult_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _CameraStream_GetResult_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _CameraStream_GetResult_type_support_symbol_names_t _CameraStream_GetResult_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, custom_interface_yolo, action, CameraStream_GetResult)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, custom_interface_yolo, action, CameraStream_GetResult)),
  }
};

typedef struct _CameraStream_GetResult_type_support_data_t
{
  void * data[2];
} _CameraStream_GetResult_type_support_data_t;

static _CameraStream_GetResult_type_support_data_t _CameraStream_GetResult_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _CameraStream_GetResult_service_typesupport_map = {
  2,
  "custom_interface_yolo",
  &_CameraStream_GetResult_service_typesupport_ids.typesupport_identifier[0],
  &_CameraStream_GetResult_service_typesupport_symbol_names.symbol_name[0],
  &_CameraStream_GetResult_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t CameraStream_GetResult_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_CameraStream_GetResult_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace custom_interface_yolo

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<custom_interface_yolo::action::CameraStream_GetResult>()
{
  return &::custom_interface_yolo::action::rosidl_typesupport_cpp::CameraStream_GetResult_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "custom_interface_yolo/action/detail/camera_stream__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace custom_interface_yolo
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _CameraStream_FeedbackMessage_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _CameraStream_FeedbackMessage_type_support_ids_t;

static const _CameraStream_FeedbackMessage_type_support_ids_t _CameraStream_FeedbackMessage_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _CameraStream_FeedbackMessage_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _CameraStream_FeedbackMessage_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _CameraStream_FeedbackMessage_type_support_symbol_names_t _CameraStream_FeedbackMessage_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, custom_interface_yolo, action, CameraStream_FeedbackMessage)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, custom_interface_yolo, action, CameraStream_FeedbackMessage)),
  }
};

typedef struct _CameraStream_FeedbackMessage_type_support_data_t
{
  void * data[2];
} _CameraStream_FeedbackMessage_type_support_data_t;

static _CameraStream_FeedbackMessage_type_support_data_t _CameraStream_FeedbackMessage_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _CameraStream_FeedbackMessage_message_typesupport_map = {
  2,
  "custom_interface_yolo",
  &_CameraStream_FeedbackMessage_message_typesupport_ids.typesupport_identifier[0],
  &_CameraStream_FeedbackMessage_message_typesupport_symbol_names.symbol_name[0],
  &_CameraStream_FeedbackMessage_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t CameraStream_FeedbackMessage_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_CameraStream_FeedbackMessage_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace custom_interface_yolo

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<custom_interface_yolo::action::CameraStream_FeedbackMessage>()
{
  return &::custom_interface_yolo::action::rosidl_typesupport_cpp::CameraStream_FeedbackMessage_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, custom_interface_yolo, action, CameraStream_FeedbackMessage)() {
  return get_message_type_support_handle<custom_interface_yolo::action::CameraStream_FeedbackMessage>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

#include "action_msgs/msg/goal_status_array.hpp"
#include "action_msgs/srv/cancel_goal.hpp"
// already included above
// #include "custom_interface_yolo/action/detail/camera_stream__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_typesupport_cpp/action_type_support.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support.hpp"

namespace custom_interface_yolo
{

namespace action
{

namespace rosidl_typesupport_cpp
{

static rosidl_action_type_support_t CameraStream_action_type_support_handle = {
  NULL, NULL, NULL, NULL, NULL};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace custom_interface_yolo

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_action_type_support_t *
get_action_type_support_handle<custom_interface_yolo::action::CameraStream>()
{
  using ::custom_interface_yolo::action::rosidl_typesupport_cpp::CameraStream_action_type_support_handle;
  // Thread-safe by always writing the same values to the static struct
  CameraStream_action_type_support_handle.goal_service_type_support = get_service_type_support_handle<::custom_interface_yolo::action::CameraStream::Impl::SendGoalService>();
  CameraStream_action_type_support_handle.result_service_type_support = get_service_type_support_handle<::custom_interface_yolo::action::CameraStream::Impl::GetResultService>();
  CameraStream_action_type_support_handle.cancel_service_type_support = get_service_type_support_handle<::custom_interface_yolo::action::CameraStream::Impl::CancelGoalService>();
  CameraStream_action_type_support_handle.feedback_message_type_support = get_message_type_support_handle<::custom_interface_yolo::action::CameraStream::Impl::FeedbackMessage>();
  CameraStream_action_type_support_handle.status_message_type_support = get_message_type_support_handle<::custom_interface_yolo::action::CameraStream::Impl::GoalStatusMessage>();
  return &CameraStream_action_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp
