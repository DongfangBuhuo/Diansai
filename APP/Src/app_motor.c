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
#include <string.h>

#include "queue.h"
#include "task.h"

#define APP_MOTOR_TASK_PERIOD_MS (10U)

static void AppMotorTask(void *argument);

static QueueHandle_t g_motor_cmd_queue;
static app_motor_cmd_t g_motor_cmd;

void AppMotor_Init(void)
{
    motor_dev_t *motor_dev;

    (void)memset(&g_motor_cmd, 0, sizeof(g_motor_cmd));
    motor_dev = Motor_GetDefaultDevice();
    (void)Motor_Init(motor_dev);

    if (g_motor_cmd_queue == NULL)
    {
        g_motor_cmd_queue = xQueueCreate(1, sizeof(app_motor_cmd_t));
    }
}

void AppMotor_TaskCreate(void)
{
    motor_dev_t *motor_dev;

    motor_dev = Motor_GetDefaultDevice();
    (void)xTaskCreate(AppMotorTask, "appMotorTask", 256U, motor_dev, tskIDLE_PRIORITY + 1U, NULL);
}

BaseType_t AppMotor_SendCommand(const app_motor_cmd_t *cmd)
{
    if ((cmd == NULL) || (g_motor_cmd_queue == NULL))
    {
        return pdFAIL;
    }

    return xQueueOverwrite(g_motor_cmd_queue, cmd);
}

static void AppMotorTask(void *argument)
{
    motor_dev_t *motor_dev;
    TickType_t last_wake;
    app_motor_cmd_t cmd;

    motor_dev = (motor_dev_t *)argument;
    last_wake = xTaskGetTickCount();

    for (;;)
    {
        while (xQueueReceive(g_motor_cmd_queue, &cmd, 0U) == pdPASS)
        {
            g_motor_cmd = cmd;
        }

        (void)Motor_SetDir(motor_dev, g_motor_cmd.dir);
        (void)Motor_SetSpeed(motor_dev, g_motor_cmd.speed_a, g_motor_cmd.speed_b);
        (void)Motor_Update(motor_dev);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(APP_MOTOR_TASK_PERIOD_MS));
    }
}
