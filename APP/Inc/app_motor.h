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

/**
 * @brief 电机控制命令结构体
 *
 * 用于描述算法层或其他业务模块发给电机任务的控制命令:
 * - dir: 目标方向
 * - speed: 目标速度
 */
typedef struct
{
    motor_dir_t dir;
    uint16_t speed;
} app_motor_cmd_t;

/**
 * @brief APP 层电机模块初始化
 *
 * 功能说明:
 * - 获取默认电机设备对象
 * - 初始化 BSP 电机驱动
 * - 创建电机命令队列
 *
 * @retval None
 */
void AppMotor_Init(void);

/**
 * @brief APP 层电机任务创建接口
 *
 * 功能说明:
 * - 创建电机控制任务
 * - 由该任务统一接收命令并驱动底层电机
 *
 * @retval None
 */
void AppMotor_TaskCreate(void);

/**
 * @brief 向电机模块发送控制命令
 *
 * 功能说明:
 * - 将目标方向和速度发送到电机命令队列
 * - 供算法层或其他业务模块调用
 *
 * @param cmd 电机命令指针
 * @return BaseType_t pdPASS 表示发送成功, 其他值表示失败
 */
BaseType_t AppMotor_SendCommand(const app_motor_cmd_t *cmd);

#endif // !__APP_MOTOR_H
