/************************************************************************
 * @file app_algorithm.c
 * @brief APP 灞傜畻娉曟ā鍧楀疄鐜版枃浠? *
 * 璇︾粏鎻忚堪:
 * - 鍔熻兘1: 棰勭暀绠楁硶妯″潡鍒濆鍖栧叆鍙? * - 鍔熻兘2: 棰勭暀绠楁硶浠诲姟鍒涘缓鍏ュ彛
 * - 鍔熻兘3: 涓哄悗缁柟鍚戣瘑鍒笌鎺у埗鍐崇瓥閫昏緫鎻愪緵瀹炵幇浣嶇疆
 *
 * @author DFBH
 * @date 2026-06-18
 * @version 1.0.0
 *
 * @note 閲囩敤鍔犳潈骞冲潎 + PID 鍋忓樊璁＄畻, 閫氳繃 AppMotor_SendCommand 涓嬪彂宸€熸帶鍒跺懡浠? * @warning 鑻ュ皻鏈疄鐜板叿浣撶畻娉曢€昏緫, 涓嶅簲鍦ㄤ换鍔′腑鐩存帴椹卞姩搴曞眰鐢垫満
 ************************************************************************/
#include "app_algorithm.h"

#include "FreeRTOS.h"
#include "app_imu.h"
#include "app_ir.h"
#include "app_motor.h"
#include "elog.h"
#include "queue.h"
#include "task.h"
#include <stdio.h>

//#define APP_ALGORITHM_DEBUG

static QueueHandle_t g_ir_data_queue;
static uint8_t g_yaw_log_div;

static void AppAlgorithmTask(void *argument);
static uint8_t AppAlgorithmReadLine(app_ir_data_t *ir_data, uint8_t sensor[8]);
static void AppAlgorithmEnterYaw(void);
static void AppAlgorithmEnterLine(void);
static int16_t AppAlgorithmCalcLineDiff(const uint8_t sensor[8]);
static int16_t AppAlgorithmCalcYawDiff(void);
static int16_t AppAlgorithmClampSpeed(int16_t speed);
static int16_t AppAlgorithmClampDiff(int32_t diff);

static app_algorithm_t g_app_algorithm = {
    .line_pid =
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
    .yaw_pid =
        {
            .kp = 12.0f,
            .ki = 0.0f,
            .kd = 6.0f,
            .error = 0.0f,
            .last_error = 0.0f,
            .d_error = 0.0f,
            .integral = 0.0f,
            .output = 0.0f,
        },
    .state = APP_CTRL_LINE,
    .base_speed = 200,
    .speed_max = 500,
    .speed_min = 0,
    .diff = 0,
    .active_count = 0,
    .weights = {-15, -9, -3, -1, 1, 3, 9, 15},
    .weighted_sum = 0,
    .yaw_ref = 0.0f,
};

void AppAlgorithmGetIr(app_ir_data_t *ir_data)
{
    if ((ir_data == NULL) || (g_ir_data_queue == NULL))
    {
#ifdef APP_ALGORITHM_DEBUG
        printf("Param is NULL \r\n at Function %s", __FUNCTION__);
#endif
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

    g_yaw_log_div = 0U;
}

void AppAlgorithm_TaskCreate(void)
{
    xTaskCreate(AppAlgorithmTask, "appAlgorithmTask", 256U, NULL, tskIDLE_PRIORITY + 1U, NULL);
}

static void AppAlgorithmTask(void *argument)
{
    app_motor_cmd_t motor_cmd;
    app_ir_data_t ir_data;
    uint8_t sensor[8];
    uint8_t has_line;

    (void)argument;

    for (;;)
    {
        if (xQueueReceive(g_ir_data_queue, &ir_data, portMAX_DELAY) == pdPASS)
        {
            has_line = AppAlgorithmReadLine(&ir_data, sensor);

#ifdef APP_ALGORITHM_DEBUG
            log_i("x1:%d,x2:%d,x3:%d,x4:%d,x5:%d,x6:%d,x7:%d,x8:%d\r\n",
                  sensor[0],
                  sensor[1],
                  sensor[2],
                  sensor[3],
                  sensor[4],
                  sensor[5],
                  sensor[6],
                  sensor[7]);
#endif

            if (has_line != 0U)
            {
                if (g_app_algorithm.state == APP_CTRL_YAW)
                {
                    AppAlgorithmEnterLine();
                }

                g_app_algorithm.diff = AppAlgorithmCalcLineDiff(sensor);
            }
            else
            {
                if ((g_app_algorithm.state == APP_CTRL_LINE) && (AppImu_IsReady() != 0U))
                {
                    AppAlgorithmEnterYaw();
                }

                if (g_app_algorithm.state == APP_CTRL_YAW)
                {
                    g_app_algorithm.diff = AppAlgorithmCalcYawDiff();
                }
                else
                {
                    g_app_algorithm.diff = AppAlgorithmClampDiff(g_app_algorithm.line_pid.output);
                }
            }

            motor_cmd.dir = MOTOR_FORWARD;
            motor_cmd.speed_a = AppAlgorithmClampSpeed(g_app_algorithm.base_speed + g_app_algorithm.diff);
            motor_cmd.speed_b = AppAlgorithmClampSpeed(g_app_algorithm.base_speed - g_app_algorithm.diff);

#ifdef APP_ALGORITHM_DEBUG
            log_i("[motor_cmd]: mode:%d speed_a:%d speed_b:%d diff:%d \r\n",
                  g_app_algorithm.state,
                  motor_cmd.speed_a,
                  motor_cmd.speed_b,
                  g_app_algorithm.diff);
#endif

            AppMotor_SendCommand(&motor_cmd);
        }
    }
}

static uint8_t AppAlgorithmReadLine(app_ir_data_t *ir_data, uint8_t sensor[8])
{
    uint8_t has_line;
    uint8_t i;

    sensor[0] = ir_data->data.x1;
    sensor[1] = ir_data->data.x2;
    sensor[2] = ir_data->data.x3;
    sensor[3] = ir_data->data.x4;
    sensor[4] = ir_data->data.x5;
    sensor[5] = ir_data->data.x6;
    sensor[6] = ir_data->data.x7;
    sensor[7] = ir_data->data.x8;

    g_app_algorithm.active_count = 0U;
    has_line = 0U;

    for (i = 0U; i < 8U; i++)
    {
        if (sensor[i] == 0U)
        {
            g_app_algorithm.active_count++;
            has_line = 1U;
        }
    }

    return has_line;
}

static void AppAlgorithmEnterYaw(void)
{
    app_imu_data_t imu_data;

    AppImu_GetData(&imu_data);
    g_app_algorithm.state = APP_CTRL_YAW;
    g_app_algorithm.yaw_ref = imu_data.yaw_deg;
    g_app_algorithm.yaw_pid.error = 0.0f;
    g_app_algorithm.yaw_pid.last_error = 0.0f;
    g_app_algorithm.yaw_pid.d_error = 0.0f;
    g_app_algorithm.yaw_pid.integral = 0.0f;
    g_app_algorithm.yaw_pid.output = 0.0f;
    g_yaw_log_div = 0U;

#ifdef APP_ALGORITHM_DEBUG
    log_w("[yaw_enter] ref:%.2f yaw:%.2f gz:%.2f\r\n",
          g_app_algorithm.yaw_ref,
          imu_data.yaw_deg,
          imu_data.gyro_z_dps);
#endif
}

static void AppAlgorithmEnterLine(void)
{
    g_app_algorithm.state = APP_CTRL_LINE;
    g_app_algorithm.line_pid.error = 0;
    g_app_algorithm.line_pid.last_error = 0;
    g_app_algorithm.line_pid.integral = 0;
    g_app_algorithm.line_pid.d_error = 0;
    g_app_algorithm.line_pid.output = 0;
}

static int16_t AppAlgorithmCalcLineDiff(const uint8_t sensor[8])
{
    uint8_t i;

    g_app_algorithm.weighted_sum = 0;

    for (i = 0U; i < 8U; i++)
    {
        if (sensor[i] == 0U)
        {
            g_app_algorithm.weighted_sum += g_app_algorithm.weights[i];
        }
    }

    if (g_app_algorithm.active_count != 0U)
    {
        g_app_algorithm.line_pid.error =
            g_app_algorithm.weighted_sum / g_app_algorithm.active_count;
    }
    else
    {
        g_app_algorithm.line_pid.error = g_app_algorithm.line_pid.last_error;
    }

    g_app_algorithm.line_pid.d_error =
        g_app_algorithm.line_pid.error - g_app_algorithm.line_pid.last_error;
    g_app_algorithm.line_pid.output =
        g_app_algorithm.line_pid.kp * g_app_algorithm.line_pid.error +
        g_app_algorithm.line_pid.kd * g_app_algorithm.line_pid.d_error;
    g_app_algorithm.line_pid.last_error = g_app_algorithm.line_pid.error;

#ifdef APP_ALGORITHM_DEBUG
    log_i("[line] err:%d act:%d sum:%d out:%d\r\n",
          g_app_algorithm.line_pid.error,
          g_app_algorithm.active_count,
          g_app_algorithm.weighted_sum,
          g_app_algorithm.line_pid.output);
#endif

    return AppAlgorithmClampDiff(g_app_algorithm.line_pid.output);
}

static int16_t AppAlgorithmCalcYawDiff(void)
{
    app_imu_data_t imu_data;
    float output;
    int32_t diff;

    AppImu_GetData(&imu_data);
    g_app_algorithm.yaw_pid.error = imu_data.yaw_deg - g_app_algorithm.yaw_ref;
    g_app_algorithm.yaw_pid.d_error =
        g_app_algorithm.yaw_pid.error - g_app_algorithm.yaw_pid.last_error;
    g_app_algorithm.yaw_pid.integral += g_app_algorithm.yaw_pid.error;
    output = g_app_algorithm.yaw_pid.kp * g_app_algorithm.yaw_pid.error +
             g_app_algorithm.yaw_pid.ki * g_app_algorithm.yaw_pid.integral +
             g_app_algorithm.yaw_pid.kd * g_app_algorithm.yaw_pid.d_error;
    g_app_algorithm.yaw_pid.output = output;
    g_app_algorithm.yaw_pid.last_error = g_app_algorithm.yaw_pid.error;
    diff = (int32_t)g_app_algorithm.yaw_pid.output;

#ifdef APP_ALGORITHM_DEBUG
    g_yaw_log_div++;
    if (g_yaw_log_div >= 5U)
    {
        g_yaw_log_div = 0U;
        log_i("[yaw] ref:%.2f yaw:%.2f err:%.2f de:%.2f out:%.2f diff:%ld gz:%.2f\r\n",
              g_app_algorithm.yaw_ref,
              imu_data.yaw_deg,
              g_app_algorithm.yaw_pid.error,
              g_app_algorithm.yaw_pid.d_error,
              g_app_algorithm.yaw_pid.output,
              diff,
              imu_data.gyro_z_dps);
    }
#endif

    return AppAlgorithmClampDiff(diff);
}

static int16_t AppAlgorithmClampSpeed(int16_t speed)
{
    if (speed < g_app_algorithm.speed_min)
    {
        speed = g_app_algorithm.speed_min;
    }
    if (speed > g_app_algorithm.speed_max)
    {
        speed = g_app_algorithm.speed_max;
    }

    return speed;
}

static int16_t AppAlgorithmClampDiff(int32_t diff)
{
    if (diff > g_app_algorithm.base_speed)
    {
        diff = g_app_algorithm.base_speed;
    }
    if (diff < -g_app_algorithm.base_speed)
    {
        diff = -g_app_algorithm.base_speed;
    }

    return (int16_t)diff;
}
