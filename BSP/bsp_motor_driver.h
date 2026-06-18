/************************************************************************
 * @file bsp_motor.h
 * @brief 电机控制
 *
 * 详细描述:
 * - 功能1: 具体描述
 * - 功能2: 具体描述
 *
 * @author DFBH
 * @date 2026-06-18
 * @version 1.0.0
 *
 * @note 备注信息 (可选)
 * @warning 警告信息 (可选)
 ************************************************************************/
#ifndef __BSP_MOTOR_DRIVER_H
#define __BSP_MOTOR_DRIVER_H
/****************Include****************/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include <stdint.h>

/***************Typedef*****************/
typedef enum
{
    FORWARD = 0,
    BACKWARD,
    LEFT,
    RIGHT,
} MotorDir;
typedef struct
{
    MotorDir dir;
    int16_t raw_speed;
    int16_t real_speed;
    /* data */
} Motor;

extern Motor motor;

void Motor_Init();
void Motor_SetDir(MotorDir dir);
void Motor_SetSpeed(int16_t speed);
void Motor_Update(void);
#endif // !__BSP_MOTOR_DRIVER_H