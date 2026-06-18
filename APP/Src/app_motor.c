/************************************************************************
 * @file app_motor.c
 * @brief APP 层电机模块实现文件
 *
 * 详细描述:
 * - 功能1: 初始化 APP 层电机模块与底层电机驱动
 * - 功能2: 创建电机控制任务
 * - 功能3: 通过私有命令队列接收上层控制指令并驱动电机执行
 *
 * @author DFBH
 * @date 2026-06-18
 * @version 1.0.0
 *
 * @note 本模块采用原生 FreeRTOS API, 并使用私有队列作为任务间通信通道
 * @warning 当前队列长度为 1, 表示系统只保留最新一条电机控制命令
 ************************************************************************/
#include "app_motor.h"

#include <stdio.h>

#include "queue.h"
#include "task.h"

static void AppMotorTask(void *argument);

/**
 * @brief 电机命令队列
 *
 * 该队列仅在本文件内使用, 用于缓存发往电机任务的控制命令。
 * 使用 static 的目的是限制其可见范围, 防止其他模块直接操作队列句柄。
 */
static QueueHandle_t g_motor_cmd_queue;

/**
 * @brief APP 层电机模块初始化
 *
 * 功能说明:
 * - 获取默认电机设备对象
 * - 调用 BSP 电机驱动初始化接口
 * - 创建电机命令队列
 *
 * @retval None
 */
void AppMotor_Init(void)
{
    motor_dev_t *motor_dev;

    motor_dev = Motor_GetDefaultDevice();
    (void)Motor_Init(motor_dev);

    if (g_motor_cmd_queue == NULL)
    {
        g_motor_cmd_queue = xQueueCreate(5, sizeof(app_motor_cmd_t));
    }
}

/**
 * @brief APP 层电机任务创建接口
 *
 * 功能说明:
 * - 获取默认电机设备对象
 * - 使用原生 FreeRTOS 接口创建电机控制任务
 *
 * @retval None
 */
void AppMotor_TaskCreate(void)
{
    motor_dev_t *motor_dev;

    motor_dev = Motor_GetDefaultDevice();
    (void)xTaskCreate(AppMotorTask, "appMotorTask", 256U, motor_dev, tskIDLE_PRIORITY + 1U, NULL);
}

/**
 * @brief 发送电机控制命令
 *
 * 功能说明:
 * - 检查命令指针和队列句柄合法性
 * - 将最新控制命令覆盖写入队列
 * - 保证电机任务始终处理最新的一条命令
 *
 * @param cmd 电机命令指针
 * @return BaseType_t 发送结果
 */
BaseType_t AppMotor_SendCommand(const app_motor_cmd_t *cmd)
{
    if ((cmd == NULL) || (g_motor_cmd_queue == NULL))
    {
        return pdFAIL;
    }

    return xQueueOverwrite(g_motor_cmd_queue, cmd);
}

/**
 * @brief 电机控制任务函数
 *
 * 功能说明:
 * - 阻塞等待来自算法层或其他业务模块的电机命令
 * - 收到命令后依次设置方向、速度并执行更新
 * - 统一由本任务串行访问电机驱动, 降低并发访问风险
 *
 * @param argument 任务参数, 当前传入默认电机设备指针
 * @retval None
 */
static void AppMotorTask(void *argument)
{
    motor_dev_t *motor_dev;
    app_motor_cmd_t cmd;

    motor_dev = (motor_dev_t *)argument;

    for (;;)
    {
        if (xQueueReceive(g_motor_cmd_queue, &cmd, portMAX_DELAY) == pdPASS)
        {
            (void)Motor_SetDir(motor_dev, cmd.dir);
            (void)Motor_SetSpeed(motor_dev, cmd.speed);
            (void)Motor_Update(motor_dev);
        }
    }
}
