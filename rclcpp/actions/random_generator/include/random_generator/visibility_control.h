#ifndef RANDOM_GENERATOR__VISIBILITY_CONTROL_H_
#define RANDOM_GENERATOR__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define RANDOM_GENERATOR_EXPORT __attribute__ ((dllexport))
    #define RANDOM_GENERATOR_IMPORT __attribute__ ((dllimport))
  #else
    #define RANDOM_GENERATOR_EXPORT __declspec(dllexport)
    #define RANDOM_GENERATOR_IMPORT __declspec(dllimport)
  #endif
  #ifdef RANDOM_GENERATOR_BUILDING_DLL
    #define RANDOM_GENERATOR_PUBLIC RANDOM_GENERATOR_EXPORT
  #else
    #define RANDOM_GENERATOR_PUBLIC RANDOM_GENERATOR_IMPORT
  #endif
  #define RANDOM_GENERATOR_PUBLIC_TYPE RANDOM_GENERATOR_PUBLIC
  #define RANDOM_GENERATOR_LOCAL
#else
  #define RANDOM_GENERATOR_EXPORT __attribute__ ((visibility("default")))
  #define RANDOM_GENERATOR_IMPORT
  #if __GNUC__ >= 4
    #define RANDOM_GENERATOR_PUBLIC __attribute__ ((visibility("default")))
    #define RANDOM_GENERATOR_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define RANDOM_GENERATOR_PUBLIC
    #define RANDOM_GENERATOR_LOCAL
  #endif
  #define RANDOM_GENERATOR_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // RANDOM_GENERATOR__VISIBILITY_CONTROL_H_