/************************************************************************
 * @file bsp_mpu6050_driver.h
 * @brief MPU6050 driver public interface header file
 *
 * Detailed description:
 * - Function 1: Define MPU6050 driver status code and data type
 * - Function 2: Provide abstract ops interface for later chip replacement
 * - Function 3: Provide default device acquire, init and raw data read API
 *
 * @author DFBH
 * @date 2026-07-01
 * @version 1.0.0
 *
 * @note This file only declares interface and type
 * @warning Use MPU6050_GetDefaultDevice to get valid device instance first
 ************************************************************************/
#ifndef __BSP_MPU6050_DRIVER_H
#define __BSP_MPU6050_DRIVER_H

/**************** Include ****************/
#include "main.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>

/*************** Macro *******************/
#define MPU6050_DEBUG

/*************** Typedef *****************/
typedef struct mpu6050_dev mpu6050_dev_t;

typedef enum
{
    MPU6050_OK,
    MPU6050_ERROR,
    MPU6050_PARAMERROR,
} mpu6050_status_t;

typedef struct
{
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temp;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} mpu6050_raw_data_t;

typedef struct
{
    mpu6050_status_t (*pf_mpu6050_init)(mpu6050_dev_t *dev);
    mpu6050_status_t (*pf_mpu6050_read_raw)(mpu6050_dev_t *dev, mpu6050_raw_data_t *raw_data);
} mpu6050_ops_t;

struct mpu6050_dev
{
    const mpu6050_ops_t *ops;
    void *ctx;
};

mpu6050_dev_t *MPU6050_GetDefaultDevice(void);
mpu6050_status_t MPU6050_Init(mpu6050_dev_t *dev);
mpu6050_status_t MPU6050_ReadRawData(mpu6050_dev_t *dev, mpu6050_raw_data_t *raw_data);

#endif // !__BSP_MPU6050_DRIVER_H
