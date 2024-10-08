// generated from rosidl_generator_c/resource/rosidl_generator_c__visibility_control.h.in
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACE_YOLO__MSG__ROSIDL_GENERATOR_C__VISIBILITY_CONTROL_H_
#define CUSTOM_INTERFACE_YOLO__MSG__ROSIDL_GENERATOR_C__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSIDL_GENERATOR_C_EXPORT_custom_interface_yolo __attribute__ ((dllexport))
    #define ROSIDL_GENERATOR_C_IMPORT_custom_interface_yolo __attribute__ ((dllimport))
  #else
    #define ROSIDL_GENERATOR_C_EXPORT_custom_interface_yolo __declspec(dllexport)
    #define ROSIDL_GENERATOR_C_IMPORT_custom_interface_yolo __declspec(dllimport)
  #endif
  #ifdef ROSIDL_GENERATOR_C_BUILDING_DLL_custom_interface_yolo
    #define ROSIDL_GENERATOR_C_PUBLIC_custom_interface_yolo ROSIDL_GENERATOR_C_EXPORT_custom_interface_yolo
  #else
    #define ROSIDL_GENERATOR_C_PUBLIC_custom_interface_yolo ROSIDL_GENERATOR_C_IMPORT_custom_interface_yolo
  #endif
#else
  #define ROSIDL_GENERATOR_C_EXPORT_custom_interface_yolo __attribute__ ((visibility("default")))
  #define ROSIDL_GENERATOR_C_IMPORT_custom_interface_yolo
  #if __GNUC__ >= 4
    #define ROSIDL_GENERATOR_C_PUBLIC_custom_interface_yolo __attribute__ ((visibility("default")))
  #else
    #define ROSIDL_GENERATOR_C_PUBLIC_custom_interface_yolo
  #endif
#endif

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_INTERFACE_YOLO__MSG__ROSIDL_GENERATOR_C__VISIBILITY_CONTROL_H_
