/************************************************************************
 * @file bsp_motor_driver.h
 * @brief 电机驱动对外接口头文件
 *
 * 详细描述:
 * - 功能1: 定义电机驱动抽象层使用的数据类型与状态码
 * - 功能2: 对外提供默认电机设备获取、初始化、方向设置、双电机速度设置和执行更新接口
 * - 功能3: 为后续更换底层芯片或驱动实现预留统一的 ops 抽象
 *
 * @author DFBH
 * @date 2026-06-18
 * @version 1.0.0
 *
 * @note 本文件只负责声明接口与类型, 不直接操作底层硬件寄存器
 * @warning 使用对外接口前, 需先通过 Motor_GetDefaultDevice 获取有效设备实例
 ************************************************************************/
#ifndef __BSP_MOTOR_DRIVER_H
#define __BSP_MOTOR_DRIVER_H

/**************** Include ****************/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include <stdint.h>

/*************** Macro *******************/
//#define MOTOR_DEBUG

/*************** Typedef *****************/
typedef struct motor_dev motor_dev_t;

typedef enum
{
    MOTOR_OK,
    MOTOR_ERROR,
    MOTOR_PARAMERROR,
} motor_status_t;

typedef enum
{
    MOTOR_FORWARD = 0,
    MOTOR_BACKWARD,
    MOTOR_LEFT,
    MOTOR_RIGHT,
    MOTOR_RESERVE = 0xff,
} motor_dir_t;

/**
 * @brief 电机运行信息结构体
 *
 * 用于保存当前电机设备的软件侧状态:
 * - dir: 当前期望运动方向
 * - raw_speed_a: A 电机当前期望原始速度值
 * - raw_speed_b: B 电机当前期望原始速度值
 * - real_speed_a: A 电机当前实际速度值
 * - real_speed_b: B 电机当前实际速度值
 */
typedef struct
{
    motor_dir_t dir;
    int16_t raw_speed_a;
    int16_t raw_speed_b;
    int16_t real_speed_a;
    int16_t real_speed_b;
} motor_info_t;

typedef struct
{
    motor_status_t (*pf_motor_init)(motor_dev_t *dev);
    motor_status_t (*pf_motor_set_dir)(motor_dev_t *dev, motor_dir_t dir);
    motor_status_t (*pf_motor_set_speed)(motor_dev_t *dev, int16_t speed_a, int16_t speed_b);
    motor_status_t (*pf_motor_update)(motor_dev_t *dev);
} motor_ops_t;

struct motor_dev
{
    const motor_ops_t *ops;
    void *info;
};

motor_dev_t *Motor_GetDefaultDevice(void);
motor_status_t Motor_Init(motor_dev_t *dev);
motor_status_t Motor_SetDir(motor_dev_t *dev, motor_dir_t dir);
motor_status_t Motor_SetSpeed(motor_dev_t *dev, int16_t speed_a, int16_t speed_b);
motor_status_t Motor_Update(motor_dev_t *dev);

#endif // !__BSP_MOTOR_DRIVER_H
