// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from yellow_ball_finder:action/FindYellow.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "yellow_ball_finder/action/detail/find_yellow__struct.h"
#include "yellow_ball_finder/action/detail/find_yellow__type_support.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace yellow_ball_finder
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _FindYellow_Goal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _FindYellow_Goal_type_support_ids_t;

static const _FindYellow_Goal_type_support_ids_t _FindYellow_Goal_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _FindYellow_Goal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _FindYellow_Goal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _FindYellow_Goal_type_support_symbol_names_t _FindYellow_Goal_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, yellow_ball_finder, action, FindYellow_Goal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yellow_ball_finder, action, FindYellow_Goal)),
  }
};

typedef struct _FindYellow_Goal_type_support_data_t
{
  void * data[2];
} _FindYellow_Goal_type_support_data_t;

static _FindYellow_Goal_type_support_data_t _FindYellow_Goal_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _FindYellow_Goal_message_typesupport_map = {
  2,
  "yellow_ball_finder",
  &_FindYellow_Goal_message_typesupport_ids.typesupport_identifier[0],
  &_FindYellow_Goal_message_typesupport_symbol_names.symbol_name[0],
  &_FindYellow_Goal_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t FindYellow_Goal_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_FindYellow_Goal_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace yellow_ball_finder

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, yellow_ball_finder, action, FindYellow_Goal)() {
  return &::yellow_ball_finder::action::rosidl_typesupport_c::FindYellow_Goal_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "yellow_ball_finder/action/detail/find_yellow__struct.h"
// already included above
// #include "yellow_ball_finder/action/detail/find_yellow__type_support.h"
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

namespace yellow_ball_finder
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _FindYellow_Result_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _FindYellow_Result_type_support_ids_t;

static const _FindYellow_Result_type_support_ids_t _FindYellow_Result_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _FindYellow_Result_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _FindYellow_Result_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _FindYellow_Result_type_support_symbol_names_t _FindYellow_Result_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, yellow_ball_finder, action, FindYellow_Result)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yellow_ball_finder, action, FindYellow_Result)),
  }
};

typedef struct _FindYellow_Result_type_support_data_t
{
  void * data[2];
} _FindYellow_Result_type_support_data_t;

static _FindYellow_Result_type_support_data_t _FindYellow_Result_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _FindYellow_Result_message_typesupport_map = {
  2,
  "yellow_ball_finder",
  &_FindYellow_Result_message_typesupport_ids.typesupport_identifier[0],
  &_FindYellow_Result_message_typesupport_symbol_names.symbol_name[0],
  &_FindYellow_Result_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t FindYellow_Result_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_FindYellow_Result_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace yellow_ball_finder

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, yellow_ball_finder, action, FindYellow_Result)() {
  return &::yellow_ball_finder::action::rosidl_typesupport_c::FindYellow_Result_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "yellow_ball_finder/action/detail/find_yellow__struct.h"
// already included above
// #include "yellow_ball_finder/action/detail/find_yellow__type_support.h"
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

namespace yellow_ball_finder
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _FindYellow_Feedback_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _FindYellow_Feedback_type_support_ids_t;

static const _FindYellow_Feedback_type_support_ids_t _FindYellow_Feedback_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _FindYellow_Feedback_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _FindYellow_Feedback_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _FindYellow_Feedback_type_support_symbol_names_t _FindYellow_Feedback_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, yellow_ball_finder, action, FindYellow_Feedback)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yellow_ball_finder, action, FindYellow_Feedback)),
  }
};

typedef struct _FindYellow_Feedback_type_support_data_t
{
  void * data[2];
} _FindYellow_Feedback_type_support_data_t;

static _FindYellow_Feedback_type_support_data_t _FindYellow_Feedback_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _FindYellow_Feedback_message_typesupport_map = {
  2,
  "yellow_ball_finder",
  &_FindYellow_Feedback_message_typesupport_ids.typesupport_identifier[0],
  &_FindYellow_Feedback_message_typesupport_symbol_names.symbol_name[0],
  &_FindYellow_Feedback_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t FindYellow_Feedback_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_FindYellow_Feedback_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace yellow_ball_finder

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, yellow_ball_finder, action, FindYellow_Feedback)() {
  return &::yellow_ball_finder::action::rosidl_typesupport_c::FindYellow_Feedback_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "yellow_ball_finder/action/detail/find_yellow__struct.h"
// already included above
// #include "yellow_ball_finder/action/detail/find_yellow__type_support.h"
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

namespace yellow_ball_finder
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _FindYellow_SendGoal_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _FindYellow_SendGoal_Request_type_support_ids_t;

static const _FindYellow_SendGoal_Request_type_support_ids_t _FindYellow_SendGoal_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _FindYellow_SendGoal_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _FindYellow_SendGoal_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _FindYellow_SendGoal_Request_type_support_symbol_names_t _FindYellow_SendGoal_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, yellow_ball_finder, action, FindYellow_SendGoal_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yellow_ball_finder, action, FindYellow_SendGoal_Request)),
  }
};

typedef struct _FindYellow_SendGoal_Request_type_support_data_t
{
  void * data[2];
} _FindYellow_SendGoal_Request_type_support_data_t;

static _FindYellow_SendGoal_Request_type_support_data_t _FindYellow_SendGoal_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _FindYellow_SendGoal_Request_message_typesupport_map = {
  2,
  "yellow_ball_finder",
  &_FindYellow_SendGoal_Request_message_typesupport_ids.typesupport_identifier[0],
  &_FindYellow_SendGoal_Request_message_typesupport_symbol_names.symbol_name[0],
  &_FindYellow_SendGoal_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t FindYellow_SendGoal_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_FindYellow_SendGoal_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace yellow_ball_finder

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, yellow_ball_finder, action, FindYellow_SendGoal_Request)() {
  return &::yellow_ball_finder::action::rosidl_typesupport_c::FindYellow_SendGoal_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "yellow_ball_finder/action/detail/find_yellow__struct.h"
// already included above
// #include "yellow_ball_finder/action/detail/find_yellow__type_support.h"
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

namespace yellow_ball_finder
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _FindYellow_SendGoal_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _FindYellow_SendGoal_Response_type_support_ids_t;

static const _FindYellow_SendGoal_Response_type_support_ids_t _FindYellow_SendGoal_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _FindYellow_SendGoal_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _FindYellow_SendGoal_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _FindYellow_SendGoal_Response_type_support_symbol_names_t _FindYellow_SendGoal_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, yellow_ball_finder, action, FindYellow_SendGoal_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yellow_ball_finder, action, FindYellow_SendGoal_Response)),
  }
};

typedef struct _FindYellow_SendGoal_Response_type_support_data_t
{
  void * data[2];
} _FindYellow_SendGoal_Response_type_support_data_t;

static _FindYellow_SendGoal_Response_type_support_data_t _FindYellow_SendGoal_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _FindYellow_SendGoal_Response_message_typesupport_map = {
  2,
  "yellow_ball_finder",
  &_FindYellow_SendGoal_Response_message_typesupport_ids.typesupport_identifier[0],
  &_FindYellow_SendGoal_Response_message_typesupport_symbol_names.symbol_name[0],
  &_FindYellow_SendGoal_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t FindYellow_SendGoal_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_FindYellow_SendGoal_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace yellow_ball_finder

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, yellow_ball_finder, action, FindYellow_SendGoal_Response)() {
  return &::yellow_ball_finder::action::rosidl_typesupport_c::FindYellow_SendGoal_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "yellow_ball_finder/action/detail/find_yellow__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace yellow_ball_finder
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _FindYellow_SendGoal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _FindYellow_SendGoal_type_support_ids_t;

static const _FindYellow_SendGoal_type_support_ids_t _FindYellow_SendGoal_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _FindYellow_SendGoal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _FindYellow_SendGoal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _FindYellow_SendGoal_type_support_symbol_names_t _FindYellow_SendGoal_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, yellow_ball_finder, action, FindYellow_SendGoal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yellow_ball_finder, action, FindYellow_SendGoal)),
  }
};

typedef struct _FindYellow_SendGoal_type_support_data_t
{
  void * data[2];
} _FindYellow_SendGoal_type_support_data_t;

static _FindYellow_SendGoal_type_support_data_t _FindYellow_SendGoal_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _FindYellow_SendGoal_service_typesupport_map = {
  2,
  "yellow_ball_finder",
  &_FindYellow_SendGoal_service_typesupport_ids.typesupport_identifier[0],
  &_FindYellow_SendGoal_service_typesupport_symbol_names.symbol_name[0],
  &_FindYellow_SendGoal_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t FindYellow_SendGoal_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_FindYellow_SendGoal_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace yellow_ball_finder

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, yellow_ball_finder, action, FindYellow_SendGoal)() {
  return &::yellow_ball_finder::action::rosidl_typesupport_c::FindYellow_SendGoal_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "yellow_ball_finder/action/detail/find_yellow__struct.h"
// already included above
// #include "yellow_ball_finder/action/detail/find_yellow__type_support.h"
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

namespace yellow_ball_finder
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _FindYellow_GetResult_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _FindYellow_GetResult_Request_type_support_ids_t;

static const _FindYellow_GetResult_Request_type_support_ids_t _FindYellow_GetResult_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _FindYellow_GetResult_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _FindYellow_GetResult_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _FindYellow_GetResult_Request_type_support_symbol_names_t _FindYellow_GetResult_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, yellow_ball_finder, action, FindYellow_GetResult_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yellow_ball_finder, action, FindYellow_GetResult_Request)),
  }
};

typedef struct _FindYellow_GetResult_Request_type_support_data_t
{
  void * data[2];
} _FindYellow_GetResult_Request_type_support_data_t;

static _FindYellow_GetResult_Request_type_support_data_t _FindYellow_GetResult_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _FindYellow_GetResult_Request_message_typesupport_map = {
  2,
  "yellow_ball_finder",
  &_FindYellow_GetResult_Request_message_typesupport_ids.typesupport_identifier[0],
  &_FindYellow_GetResult_Request_message_typesupport_symbol_names.symbol_name[0],
  &_FindYellow_GetResult_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t FindYellow_GetResult_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_FindYellow_GetResult_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace yellow_ball_finder

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, yellow_ball_finder, action, FindYellow_GetResult_Request)() {
  return &::yellow_ball_finder::action::rosidl_typesupport_c::FindYellow_GetResult_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "yellow_ball_finder/action/detail/find_yellow__struct.h"
// already included above
// #include "yellow_ball_finder/action/detail/find_yellow__type_support.h"
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

namespace yellow_ball_finder
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _FindYellow_GetResult_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _FindYellow_GetResult_Response_type_support_ids_t;

static const _FindYellow_GetResult_Response_type_support_ids_t _FindYellow_GetResult_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _FindYellow_GetResult_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _FindYellow_GetResult_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _FindYellow_GetResult_Response_type_support_symbol_names_t _FindYellow_GetResult_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, yellow_ball_finder, action, FindYellow_GetResult_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yellow_ball_finder, action, FindYellow_GetResult_Response)),
  }
};

typedef struct _FindYellow_GetResult_Response_type_support_data_t
{
  void * data[2];
} _FindYellow_GetResult_Response_type_support_data_t;

static _FindYellow_GetResult_Response_type_support_data_t _FindYellow_GetResult_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _FindYellow_GetResult_Response_message_typesupport_map = {
  2,
  "yellow_ball_finder",
  &_FindYellow_GetResult_Response_message_typesupport_ids.typesupport_identifier[0],
  &_FindYellow_GetResult_Response_message_typesupport_symbol_names.symbol_name[0],
  &_FindYellow_GetResult_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t FindYellow_GetResult_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_FindYellow_GetResult_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace yellow_ball_finder

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, yellow_ball_finder, action, FindYellow_GetResult_Response)() {
  return &::yellow_ball_finder::action::rosidl_typesupport_c::FindYellow_GetResult_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "yellow_ball_finder/action/detail/find_yellow__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace yellow_ball_finder
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _FindYellow_GetResult_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _FindYellow_GetResult_type_support_ids_t;

static const _FindYellow_GetResult_type_support_ids_t _FindYellow_GetResult_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _FindYellow_GetResult_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _FindYellow_GetResult_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _FindYellow_GetResult_type_support_symbol_names_t _FindYellow_GetResult_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, yellow_ball_finder, action, FindYellow_GetResult)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yellow_ball_finder, action, FindYellow_GetResult)),
  }
};

typedef struct _FindYellow_GetResult_type_support_data_t
{
  void * data[2];
} _FindYellow_GetResult_type_support_data_t;

static _FindYellow_GetResult_type_support_data_t _FindYellow_GetResult_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _FindYellow_GetResult_service_typesupport_map = {
  2,
  "yellow_ball_finder",
  &_FindYellow_GetResult_service_typesupport_ids.typesupport_identifier[0],
  &_FindYellow_GetResult_service_typesupport_symbol_names.symbol_name[0],
  &_FindYellow_GetResult_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t FindYellow_GetResult_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_FindYellow_GetResult_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace yellow_ball_finder

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, yellow_ball_finder, action, FindYellow_GetResult)() {
  return &::yellow_ball_finder::action::rosidl_typesupport_c::FindYellow_GetResult_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "yellow_ball_finder/action/detail/find_yellow__struct.h"
// already included above
// #include "yellow_ball_finder/action/detail/find_yellow__type_support.h"
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

namespace yellow_ball_finder
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _FindYellow_FeedbackMessage_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _FindYellow_FeedbackMessage_type_support_ids_t;

static const _FindYellow_FeedbackMessage_type_support_ids_t _FindYellow_FeedbackMessage_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _FindYellow_FeedbackMessage_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _FindYellow_FeedbackMessage_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _FindYellow_FeedbackMessage_type_support_symbol_names_t _FindYellow_FeedbackMessage_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, yellow_ball_finder, action, FindYellow_FeedbackMessage)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yellow_ball_finder, action, FindYellow_FeedbackMessage)),
  }
};

typedef struct _FindYellow_FeedbackMessage_type_support_data_t
{
  void * data[2];
} _FindYellow_FeedbackMessage_type_support_data_t;

static _FindYellow_FeedbackMessage_type_support_data_t _FindYellow_FeedbackMessage_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _FindYellow_FeedbackMessage_message_typesupport_map = {
  2,
  "yellow_ball_finder",
  &_FindYellow_FeedbackMessage_message_typesupport_ids.typesupport_identifier[0],
  &_FindYellow_FeedbackMessage_message_typesupport_symbol_names.symbol_name[0],
  &_FindYellow_FeedbackMessage_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t FindYellow_FeedbackMessage_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_FindYellow_FeedbackMessage_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace yellow_ball_finder

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, yellow_ball_finder, action, FindYellow_FeedbackMessage)() {
  return &::yellow_ball_finder::action::rosidl_typesupport_c::FindYellow_FeedbackMessage_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "action_msgs/msg/goal_status_array.h"
#include "action_msgs/srv/cancel_goal.h"
#include "yellow_ball_finder/action/find_yellow.h"
// already included above
// #include "yellow_ball_finder/action/detail/find_yellow__type_support.h"

static rosidl_action_type_support_t _yellow_ball_finder__action__FindYellow__typesupport_c;

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_action_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__ACTION_SYMBOL_NAME(
  rosidl_typesupport_c, yellow_ball_finder, action, FindYellow)()
{
  // Thread-safe by always writing the same values to the static struct
  _yellow_ball_finder__action__FindYellow__typesupport_c.goal_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, yellow_ball_finder, action, FindYellow_SendGoal)();
  _yellow_ball_finder__action__FindYellow__typesupport_c.result_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, yellow_ball_finder, action, FindYellow_GetResult)();
  _yellow_ball_finder__action__FindYellow__typesupport_c.cancel_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, action_msgs, srv, CancelGoal)();
  _yellow_ball_finder__action__FindYellow__typesupport_c.feedback_message_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c, yellow_ball_finder, action, FindYellow_FeedbackMessage)();
  _yellow_ball_finder__action__FindYellow__typesupport_c.status_message_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c, action_msgs, msg, GoalStatusArray)();

  return &_yellow_ball_finder__action__FindYellow__typesupport_c;
}

#ifdef __cplusplus
}
#endif
