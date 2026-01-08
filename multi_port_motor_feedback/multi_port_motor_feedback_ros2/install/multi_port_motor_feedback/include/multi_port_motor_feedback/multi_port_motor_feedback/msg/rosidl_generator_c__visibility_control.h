// generated from rosidl_generator_c/resource/rosidl_generator_c__visibility_control.h.in
// generated code does not contain a copyright notice

#ifndef MULTI_PORT_MOTOR_FEEDBACK__MSG__ROSIDL_GENERATOR_C__VISIBILITY_CONTROL_H_
#define MULTI_PORT_MOTOR_FEEDBACK__MSG__ROSIDL_GENERATOR_C__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSIDL_GENERATOR_C_EXPORT_multi_port_motor_feedback __attribute__ ((dllexport))
    #define ROSIDL_GENERATOR_C_IMPORT_multi_port_motor_feedback __attribute__ ((dllimport))
  #else
    #define ROSIDL_GENERATOR_C_EXPORT_multi_port_motor_feedback __declspec(dllexport)
    #define ROSIDL_GENERATOR_C_IMPORT_multi_port_motor_feedback __declspec(dllimport)
  #endif
  #ifdef ROSIDL_GENERATOR_C_BUILDING_DLL_multi_port_motor_feedback
    #define ROSIDL_GENERATOR_C_PUBLIC_multi_port_motor_feedback ROSIDL_GENERATOR_C_EXPORT_multi_port_motor_feedback
  #else
    #define ROSIDL_GENERATOR_C_PUBLIC_multi_port_motor_feedback ROSIDL_GENERATOR_C_IMPORT_multi_port_motor_feedback
  #endif
#else
  #define ROSIDL_GENERATOR_C_EXPORT_multi_port_motor_feedback __attribute__ ((visibility("default")))
  #define ROSIDL_GENERATOR_C_IMPORT_multi_port_motor_feedback
  #if __GNUC__ >= 4
    #define ROSIDL_GENERATOR_C_PUBLIC_multi_port_motor_feedback __attribute__ ((visibility("default")))
  #else
    #define ROSIDL_GENERATOR_C_PUBLIC_multi_port_motor_feedback
  #endif
#endif

#ifdef __cplusplus
}
#endif

#endif  // MULTI_PORT_MOTOR_FEEDBACK__MSG__ROSIDL_GENERATOR_C__VISIBILITY_CONTROL_H_
