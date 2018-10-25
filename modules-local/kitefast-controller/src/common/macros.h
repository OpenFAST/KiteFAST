#ifndef COMMON_MACROS_H_
#define COMMON_MACROS_H_

#include <stddef.h>  // For size_t.
#include <stdint.h>  // For int32_t.

#define ARRAYSIZE(a) \
  ((int32_t)((sizeof(a) / sizeof(*(a))) / (size_t) !(sizeof(a) % sizeof(*(a)))))

#define COMPILE_ASSERT(c, m) typedef char m[(c) ? 1 : -1]

#define UNSAFE_STRUCT_FIELD(type, field) ((const type *)0)->field
#define OFFSETOF(type, field) ((size_t)&UNSAFE_STRUCT_FIELD(type, field))
#define SIZEOF(type, field) sizeof(UNSAFE_STRUCT_FIELD(type, field))

#ifdef __cplusplus
#define DISALLOW_COPY_AND_ASSIGN(TypeName) \
  TypeName(const TypeName &);              \
  void operator=(const TypeName &)
#endif

#define STR_NAME(s) #s      // Convert argument name to string.
#define STR(s) STR_NAME(s)  // Convert argument value to string.

#endif  // COMMON_MACROS_H_
