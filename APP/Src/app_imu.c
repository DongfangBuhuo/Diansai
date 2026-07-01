/************************************************************************
 * @file app_imu.c
 * @brief APP layer IMU module implementation file
 *
 * Detailed description:
 * - Function 1: Complete APP layer IMU module initialization
 * - Function 2: Create IMU acquisition and yaw update task
 * - Function 3: Periodically read raw gyro data, calibrate zero bias and update yaw
 *
 * @author DFBH
 * @date 2026-07-01
 * @version 1.0.0
 *
 * @note Current implementation uses gyro Z axis integration to estimate yaw
 * @warning MPU6050 has no magnetometer, so yaw is suitable for short-time relative heading hold only
 ************************************************************************/
#include "app_imu.h"

#include "FreeRTOS.h"
#include "bsp_mpu6050_driver.h"
#include "elog.h"
#include "task.h"
#include <string.h>

#define APP_IMU_TASK_PERIOD_MS (10U)
#define APP_IMU_GYRO_Z_SENSITIVITY (131.0f)
#define APP_IMU_CALIBRATION_COUNT (200U)
#define APP_IMU_STARTUP_DELAY_MS (500U)

static void AppImuTask(void *argument);
static void AppImuCalibrateGyroZ(mpu6050_dev_t *imu_dev);
static float AppImuConvertGyroZToDps(int16_t gyro_z_raw);

static app_imu_data_t g_app_imu_data;

void AppImu_Init(void)
{
    (void)memset(&g_app_imu_data, 0, sizeof(g_app_imu_data));
}

void AppImu_TaskCreate(void)
{
    mpu6050_dev_t *imu_dev;

    imu_dev = MPU6050_GetDefaultDevice();
    (void)xTaskCreate(AppImuTask, "appImuTask", 256U, imu_dev, tskIDLE_PRIORITY + 1U, NULL);
}

void AppImu_GetData(app_imu_data_t *imu_data)
{
    if (imu_data == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    (void)memcpy(imu_data, &g_app_imu_data, sizeof(g_app_imu_data));
    taskEXIT_CRITICAL();
}

void AppImu_ResetYaw(void)
{
    taskENTER_CRITICAL();
    g_app_imu_data.yaw_deg = 0.0f;
    taskEXIT_CRITICAL();
}

uint8_t AppImu_IsReady(void)
{
    return g_app_imu_data.is_ready;
}

static float AppImuConvertGyroZToDps(int16_t gyro_z_raw)
{
    return ((float)gyro_z_raw - g_app_imu_data.gyro_z_bias) / APP_IMU_GYRO_Z_SENSITIVITY;
}

static void AppImuCalibrateGyroZ(mpu6050_dev_t *imu_dev)
{
    uint16_t sample_count;
    int32_t gyro_z_sum;
    mpu6050_raw_data_t raw_data;

    sample_count = 0U;
    gyro_z_sum = 0;
    (void)memset(&raw_data, 0, sizeof(raw_data));

    while (sample_count < APP_IMU_CALIBRATION_COUNT)
    {
        if (MPU6050_ReadRawData(imu_dev, &raw_data) == MPU6050_OK)
        {
            gyro_z_sum += raw_data.gyro_z;
            sample_count++;
        }

        vTaskDelay(pdMS_TO_TICKS(5U));
    }

    taskENTER_CRITICAL();
    g_app_imu_data.gyro_z_bias = (float)gyro_z_sum / (float)APP_IMU_CALIBRATION_COUNT;
    g_app_imu_data.yaw_deg = 0.0f;
    g_app_imu_data.is_ready = 1U;
    taskEXIT_CRITICAL();

#ifdef MPU6050_DEBUG
    log_i("[imu] gyro_z_bias: %.2f\r\n", g_app_imu_data.gyro_z_bias);
#endif
}

static void AppImuTask(void *argument)
{
    mpu6050_dev_t *imu_dev;
    mpu6050_raw_data_t raw_data;
    uint32_t last_tick;
    uint32_t now_tick;
    uint8_t log_divider;
    float dt_s;
    float gyro_z_dps;

    imu_dev = (mpu6050_dev_t *)argument;
    (void)memset(&raw_data, 0, sizeof(raw_data));
    log_divider = 0U;

    vTaskDelay(pdMS_TO_TICKS(APP_IMU_STARTUP_DELAY_MS));
    if (MPU6050_Init(imu_dev) != MPU6050_OK)
    {
#ifdef MPU6050_DEBUG
        log_e("[imu] init failed\r\n");
#endif
        vTaskDelete(NULL);
    }
    AppImuCalibrateGyroZ(imu_dev);
    last_tick = HAL_GetTick();

    for (;;)
    {
        if (MPU6050_ReadRawData(imu_dev, &raw_data) == MPU6050_OK)
        {
            now_tick = HAL_GetTick();
            dt_s = (float)(now_tick - last_tick) * 0.001f;
            if (dt_s <= 0.0f)
            {
                dt_s = (float)APP_IMU_TASK_PERIOD_MS * 0.001f;
            }
            last_tick = now_tick;

            gyro_z_dps = AppImuConvertGyroZToDps(raw_data.gyro_z);

            taskENTER_CRITICAL();
            g_app_imu_data.accel_x = raw_data.accel_x;
            g_app_imu_data.accel_y = raw_data.accel_y;
            g_app_imu_data.accel_z = raw_data.accel_z;
            g_app_imu_data.gyro_x = raw_data.gyro_x;
            g_app_imu_data.gyro_y = raw_data.gyro_y;
            g_app_imu_data.gyro_z = raw_data.gyro_z;
            g_app_imu_data.gyro_z_dps = gyro_z_dps;
            g_app_imu_data.yaw_deg += gyro_z_dps * dt_s;
            taskEXIT_CRITICAL();

#ifdef MPU6050_DEBUG
            log_divider++;
            if (log_divider >= 10U)
            {
                log_divider = 0U;
                log_i("[imu] gz_raw:%d gz_dps:%.2f yaw:%.2f ready:%d\r\n",
                      g_app_imu_data.gyro_z,
                      g_app_imu_data.gyro_z_dps,
                      g_app_imu_data.yaw_deg,
                      g_app_imu_data.is_ready);
            }
#endif
        }

        vTaskDelay(pdMS_TO_TICKS(APP_IMU_TASK_PERIOD_MS));
    }
}
