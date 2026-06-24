/************************************************************************
 * @file app_motor.h
 * @brief APP 层电机模块头文件
 *
 * 详细描述:
 * - 功能1: 声明电机模块初始化接口
 * - 功能2: 声明电机任务创建接口
 * - 功能3: 声明算法层向电机层发送控制命令的接口
 *
 * @author DFBH
 * @date 2026-06-18
 * @version 1.0.0
 *
 * @note 本模块位于 APP 层, 负责组织电机任务与命令通道
 * @warning 上层模块不应直接操作电机驱动对象, 应通过本模块提供的接口下发命令
 ************************************************************************/
#ifndef __APP_MOTOR_H
#define __APP_MOTOR_H

#include "FreeRTOS.h"
#include "bsp_motor_driver.h"

typedef struct
{
    motor_dir_t dir;
    int16_t speed_a;
    int16_t speed_b;
} app_motor_cmd_t;

void AppMotor_Init(void);
void AppMotor_TaskCreate(void);
BaseType_t AppMotor_SendCommand(const app_motor_cmd_t *cmd);

#endif // !__APP_MOTOR_H
