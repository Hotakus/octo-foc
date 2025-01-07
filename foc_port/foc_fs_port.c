/**
******************************************************************************
* @file           : foc_fs_port.c
* @author         : Hotakus (hotakus@foxmail.com)
* @brief          : None
* @date           : 2025/1/6
******************************************************************************
*/

#include "foc_fs_port.h"

#if FOC_DATA_PERSISTENCE == 1
/* if you use FATFS, uncomment the following code */
// #include "ff.h"

__attribute__((weak)) void foc_fs_port_init(void) {
    // implement your init code here, such as register file system

    /* if you use FATFS, mount */
    // extern FATFS fatfs;
    // FRESULT res = f_mount(&fatfs, "0:", 1);
    // if (res != FR_OK) {
    //     // handle error
    // }
}

__attribute__((weak)) void foc_fs_port_deinit(void) {
    // implement your deinit code here

    /* if you use FATFS, unmount */
    // extern FATFS fatfs;
    // f_mount(&fatfs, "0:", 0);
}

__attribute__((weak)) void *foc_fs_port_open(const char *path, foc_fs_mode_enum_t mode) {
    if (path == NULL) {
        return NULL;
    }

    uint16_t _mode = 0;

    switch (mode) {
        case FOC_FS_MODE_READ: {
            /* if you use FATFS, uncomment the following code */
            // _mode = FA_READ;
            break;
        }
        case FOC_FS_MODE_WRITE: {
            /* if you use FATFS, uncomment the following code */
            // _mode = FA_WRITE;
            break;
        }
        case FOC_FS_MODE_READ | FOC_FS_MODE_WRITE: {
            /* if you use FATFS, uncomment the following code */
            // _mode = FA_READ | FA_WRITE;
            break;
        }
        default:
            return NULL;
    }

    /* if you use FATFS, uncomment the following code */
    // FIL *file = f_open(path, _mode);
    // if (file == NULL) {
    //     return NULL;
    // }
    // return (void *)file;
}

__attribute__((weak)) void foc_fs_port_close(void *handle) {
    /* if you use FATFS, uncomment the following code */
    // FIL *file = (FIL *)handle;
    // f_close(file);
}

__attribute__((weak)) foc_err_enum_t foc_fs_port_read(void *handle, void *buffer, size_t size) {
    /* if you use FATFS, uncomment the following code */
    // FIL *file = (FIL *)handle;
    // size_t br = 0;
    // FRESULT res = f_read(file, buffer, size, &br);
    // if (res != FR_OK) {
    //     return FOC_ERR_FAILED;
    // }
    return FOC_ERR_OK;
}

__attribute__((weak)) foc_err_enum_t foc_fs_port_write(void *handle, const void *buffer, size_t size) {
    /* if you use FATFS, uncomment the following code */
    // FIL *file = (FIL *)handle;
    // size_t bw = 0;
    // FRESULT res = f_write(file, buffer, size, &bw);
    // if (res != FR_OK) {
    //     return FOC_ERR_FAILED;
    // }
    return FOC_ERR_OK;
}
#endif