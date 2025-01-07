/*
 * @Author: luoqi 
 * @Date: 2022-11-22 22:06:22 
 * @Last Modified by: luoqi
 * @Last Modified time: 2022-11-22 23:23:51
 */

#ifndef _TRIGF_H
#define _TRIGF_H

#ifdef __cplusplus
 extern "C" {
#endif

/* degree */
void fast_sin_cos_q15(float rad, float *sin_out, float *cos_out);

#ifdef __cplusplus
 }
#endif

#endif
