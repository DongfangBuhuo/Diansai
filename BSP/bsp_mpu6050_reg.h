/************************************************************************
 * @file bsp_mpu6050_reg.h
 * @brief MPU6050 register definition file
 *
 * Detailed description:
 * - Function 1: Define MPU6050 device address
 * - Function 2: Define commonly used register address
 * - Function 3: Provide basic configuration value macro for driver layer
 *
 * @author DFBH
 * @date 2026-07-01
 * @version 1.0.0
 *
 * @note This file only keeps register and configuration macros
 * @warning If the module AD0 pin is pulled high, device address needs adjustment
 ************************************************************************/
#ifndef __BSP_MPU6050_REG_H
#define __BSP_MPU6050_REG_H

#define MPU6050_I2C_ADDR (0x68U)

#define MPU6050_REG_SMPLRT_DIV (0x19U)
#define MPU6050_REG_CONFIG (0x1AU)
#define MPU6050_REG_GYRO_CONFIG (0x1BU)
#define MPU6050_REG_ACCEL_CONFIG (0x1CU)
#define MPU6050_REG_ACCEL_XOUT_H (0x3BU)
#define MPU6050_REG_TEMP_OUT_H (0x41U)
#define MPU6050_REG_GYRO_XOUT_H (0x43U)
#define MPU6050_REG_PWR_MGMT_1 (0x6BU)
#define MPU6050_REG_WHO_AM_I (0x75U)

#define MPU6050_WHO_AM_I_ID (0x68U)

#define MPU6050_GYRO_FS_250DPS (0x00U)
#define MPU6050_ACCEL_FS_2G (0x00U)

#endif // !__BSP_MPU6050_REG_H
