/************************************************************************
 * @file app_algorithm.h
 * @brief APP 层算法模块头文件
 *
 * 详细描述:
 * - 功能1: 声明算法模块初始化接口
 * - 功能2: 声明算法任务创建接口
 * - 功能3: 为后续方向识别、路径控制等算法功能提供扩展入口
 *
 * @author DFBH
 * @date 2026-06-18
 * @version 1.0.0
 *
 * @note 算法模块不应直接操作电机底层驱动, 应通过 AppMotor_SendCommand 下发控制命令
 * @warning 若算法任务依赖其他传感器模块, 应确保相关模块先完成初始化
 ************************************************************************/
#ifndef __APP_ALGORITHM_H
#define __APP_ALGORITHM_H

#include <stdint.h>

#include "app_ir.h"

typedef struct
{
    int16_t kp;
    int16_t ki;
    int16_t kd;
    int16_t error;
    int16_t last_error;
    int16_t d_error;
    int32_t integral;
    int16_t output;
} app_pid_t;

typedef enum
{
    APP_LINE_STATE_NORMAL,
    APP_LINE_STATE_TURN,
    APP_LINE_STATE_SHARP_TURN,
    APP_LINE_STATE_LOST,
} app_track_state_t;

typedef struct
{
    app_pid_t pid;
    app_track_state_t track_state;
    int16_t base_speed;
    int16_t speed_min;
    int16_t speed_max;
    int16_t diff;
    int16_t weighted_sum;
    uint8_t active_count;
    int16_t weights[8];
} app_algorithm_t;

void AppAlgorithm_Init(void);
void AppAlgorithm_TaskCreate(void);
void AppAlgorithmGetIr(app_ir_data_t *ir_data);

#endif // !__APP_ALGORITHM_H
