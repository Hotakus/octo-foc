/**
******************************************************************************
* @file           : util_math.h
* @author         : Hotakus (hotakus@foxmail.com)
* @brief          : None
* @date           : 2025/1/29
******************************************************************************
*/

#ifndef UTIL_MATH_H
#define UTIL_MATH_H

#include <math.h>

#define UM_EPSILON 1e-6

#ifdef __cplusplus
extern "C" {
#endif

  int compare_float(const float A, const float B);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif  //UTIL_MATH_H
