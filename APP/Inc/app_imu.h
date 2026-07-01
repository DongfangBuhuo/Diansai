/************************************************************************
 * @file app_imu.h
 * @brief APP layer IMU module header file
 *
 * Detailed description:
 * - Function 1: Declare IMU module init and task create interface
 * - Function 2: Declare IMU processed data type and external get interface
 * - Function 3: Reserve yaw reset and ready state interface for follow-up business
 *
 * @author DFBH
 * @date 2026-07-01
 * @version 1.0.0
 *
 * @note This module is located in APP layer and accesses MPU6050 through BSP driver
 * @warning Before reading IMU data, ensure AppImu_Init and AppImu_TaskCreate have completed
 ************************************************************************/
#ifndef __APP_IMU_H
#define __APP_IMU_H

#include <stdint.h>

typedef struct
{
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    float gyro_z_dps;
    float yaw_deg;
    float gyro_z_bias;
    uint8_t is_ready;
} app_imu_data_t;

void AppImu_Init(void);
void AppImu_TaskCreate(void);
void AppImu_GetData(app_imu_data_t *imu_data);
void AppImu_ResetYaw(void);
uint8_t AppImu_IsReady(void);

#endif // !__APP_IMU_H
