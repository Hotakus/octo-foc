/**
  ******************************************************************************
  * @file           : foc_fs_port.h
  * @author         : Hotakus (hotakus@foxmail.com)
  * @brief          : None
  * @date           : 2025/1/6
  ******************************************************************************
  */

#ifndef FOC_FS_PORT_H
#define FOC_FS_PORT_H

#include "foc.h"

#if FOC_DATA_PERSISTENCE == 1
void foc_fs_port_init(void);
void foc_fs_port_deinit(void);
void *foc_fs_port_open(const char *path, foc_fs_mode_enum_t mode);
void foc_fs_port_close(void *handle);
foc_err_enum_t foc_fs_port_read(void *handle, void *buffer, size_t size);
foc_err_enum_t foc_fs_port_write(void *handle, const void *buffer, size_t size);
#endif

#ifdef __cplusplus
extern "C" {
#endif



#ifdef __cplusplus
}
#endif // __cplusplus

#endif  //FOC_FS_PORT_H
