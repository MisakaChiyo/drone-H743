/*
 * Minimal MATLAB Coder runtime types for the embedded coax controller.
 * This keeps the generated code self-contained in the firmware tree.
 */
#ifndef RTWTYPES_H
#define RTWTYPES_H

#include <stdbool.h>
#include <stdint.h>

typedef double real_T;
typedef float real32_T;
typedef double real64_T;
typedef int8_t int8_T;
typedef uint8_t uint8_T;
typedef int16_t int16_T;
typedef uint16_t uint16_T;
typedef int32_t int32_T;
typedef uint32_t uint32_T;
typedef int64_t int64_T;
typedef uint64_t uint64_T;
typedef bool boolean_T;

#ifndef true
#define true (1U)
#endif

#ifndef false
#define false (0U)
#endif

#endif
