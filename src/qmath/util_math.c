/**
******************************************************************************
* @file           : util_math.c
* @author         : Hotakus (hotakus@foxmail.com)
* @brief          : None
* @date           : 2025/1/29
******************************************************************************
*/

#include "util_math.h"

/**
 * @brief Compares two floating-point numbers for equality within a defined tolerance.
 *
 * @param A The first floating-point number.
 * @param B The second floating-point number.
 * @return int Returns 0 if A and B are equal within tolerance, 1 if A > B, -1 if A < B.
 */
int inline compare_float(const float A, const float B) {
    const float diff = A - B;
    if (fabsf(diff) < UM_EPSILON) {
        return 0;
    }
    if (diff > 0) {
        return 1;
    }
    return -1;
}
