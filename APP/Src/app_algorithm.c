/************************************************************************
 * @file app_algorithm.c
 * @brief APP 层算法模块实现文件
 *
 * 详细描述:
 * - 功能1: 预留算法模块初始化入口
 * - 功能2: 预留算法任务创建入口
 * - 功能3: 为后续方向识别与控制决策逻辑提供实现位置
 *
 * @author DFBH
 * @date 2026-06-18
 * @version 1.0.0
 *
 * @note 采用加权平均 + PID 偏差计算, 通过 AppMotor_SendCommand 下发差速控制命令
 * @warning 若尚未实现具体算法逻辑, 不应在任务中直接驱动底层电机
 ************************************************************************/
#include "app_algorithm.h"
#include "FreeRTOS.h"
#include "app_ir.h"
#include "app_motor.h"
#include "elog.h"
#include "queue.h"
#include "task.h"
#include <stdio.h>
static QueueHandle_t g_ir_data_queue;
static void AppAlgorithmTask(void *argument);

static app_algorithm_t g_app_algorithm = {
    .pid =
        {
            .kd = 9,
            .ki = 0,
            .kp = 14,
            .integral = 0,
            .error = 0,
            .last_error = 0,
            .d_error = 0,
            .output = 0,
        },
    .track_state = APP_LINE_STATE_NORMAL,
    .base_speed = 200,
    .speed_max = 500,
    .speed_min = 0,
    .diff = 0,
    .active_count = 0,
    .weights = {-15, -9, -3, -1, 1, 3, 9, 15},
    .weighted_sum = 0,
};

void AppAlgorithmGetIr(app_ir_data_t *ir_data)
{
    if (NULL == ir_data || NULL == g_ir_data_queue)
    {
#ifdef DEBUG
        printf("Param is NULL \r\n at Function %s", __FUNCTION__);
#endif // DEBUG
        return;
    }

    xQueueOverwrite(g_ir_data_queue, ir_data);
}
void AppAlgorithm_Init(void)
{
    if (g_ir_data_queue == NULL)
    {
        g_ir_data_queue = xQueueCreate(1, sizeof(app_ir_data_t));
    }
}

void AppAlgorithm_TaskCreate(void)
{
    xTaskCreate(AppAlgorithmTask, "appAlgorithmTask", 128, NULL, tskIDLE_PRIORITY + 1U, NULL);
}
static void AppAlgorithmTask(void *argument)
{
    uint8_t temp_data;
    app_motor_cmd_t motor_cmd;
    app_ir_data_t ir_data;
    for (;;)
    {
        // receive data from ir
        if (xQueueReceive(g_ir_data_queue, &ir_data, portMAX_DELAY))
        {
            g_app_algorithm.weighted_sum = 0;
            g_app_algorithm.active_count = 0;
#ifdef DEBUG
            log_i("x1:%d,x2:%d,x3:%d,x4:%d,x5:%d,x6:%d,x7:%d,x8:%d\r\n",
                  ir_data.data.x1,
                  ir_data.data.x2,
                  ir_data.data.x3,
                  ir_data.data.x4,
                  ir_data.data.x5,
                  ir_data.data.x6,
                  ir_data.data.x7,
                  ir_data.data.x8);
#endif // DEBUG
            temp_data = ir_data.Alldata;
            for (int8_t i = 0; i < 8; i++)
            {
                if ((temp_data >> i & 0x01) == 0)
                {
                    g_app_algorithm.active_count++;
                    g_app_algorithm.weighted_sum += g_app_algorithm.weights[i];
                }
            }
            if (g_app_algorithm.active_count != 0U)
            {
                g_app_algorithm.pid.error = g_app_algorithm.weighted_sum / g_app_algorithm.active_count;
#ifdef DEBUG
                log_i("[error]: %d ", g_app_algorithm.pid.error);
#endif
            }
            else
            {
                g_app_algorithm.pid.error = g_app_algorithm.pid.last_error;
            }
            g_app_algorithm.pid.d_error = g_app_algorithm.pid.error - g_app_algorithm.pid.last_error;
            // 进行算法分析
            g_app_algorithm.pid.output =
                g_app_algorithm.pid.kp * g_app_algorithm.pid.error +
                g_app_algorithm.pid.kd * g_app_algorithm.pid.d_error;
            g_app_algorithm.diff = g_app_algorithm.pid.output;
            motor_cmd.dir = MOTOR_FORWARD;

            motor_cmd.speed_a = g_app_algorithm.base_speed + g_app_algorithm.diff;
            motor_cmd.speed_b = g_app_algorithm.base_speed - g_app_algorithm.diff;
#ifdef DEBUG
            log_i("[motor_cmd]: speed_a:%d speed_b:%d diff:%d \r\n",
                  motor_cmd.speed_a,
                  motor_cmd.speed_b,
                  g_app_algorithm.diff);
#endif // DEBUG
            if (motor_cmd.speed_a < g_app_algorithm.speed_min)
                motor_cmd.speed_a = g_app_algorithm.speed_min;
            if (motor_cmd.speed_a > g_app_algorithm.speed_max)
                motor_cmd.speed_a = g_app_algorithm.speed_max;
            if (motor_cmd.speed_b < g_app_algorithm.speed_min)
                motor_cmd.speed_b = g_app_algorithm.speed_min;
            if (motor_cmd.speed_b > g_app_algorithm.speed_max)
                motor_cmd.speed_b = g_app_algorithm.speed_max;
            AppMotor_SendCommand(&motor_cmd);
            g_app_algorithm.pid.last_error = g_app_algorithm.pid.error;
        }
    }
}
