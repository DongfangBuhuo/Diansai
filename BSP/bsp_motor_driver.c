/************************************************************************
 * @file bsp_motor_driver.c
 * @brief 电机驱动实现文件
 *
 * 详细描述:
 * - 功能1: 提供基于 STM32 定时器 PWM 与 GPIO 的电机驱动实现
 * - 功能2: 维护默认电机设备对象及其运行状态
 * - 功能3: 对上层暴露统一的初始化、方向设置、双电机速度设置和更新接口
 *
 * @author DFBH
 * @date 2026-06-18
 * @version 1.0.0
 *
 * @note 当前实现绑定 TIM2 作为 PWM 输出, TIM3/TIM4 作为编码器接口
 * @warning 当前方向与速度设置采用“先缓存, 后统一更新”的工作方式
 ************************************************************************/
#include "bsp_motor_driver.h"

#include "tim.h"
#include <stdio.h>

static motor_info_t g_motor_info;

static motor_status_t motor_stm32_init(motor_dev_t *dev);
static motor_status_t motor_set_speed(motor_dev_t *dev, int16_t speed_a, int16_t speed_b);
static motor_status_t motor_set_dir(motor_dev_t *dev, motor_dir_t dir);
static motor_status_t motor_update(motor_dev_t *dev);

static uint16_t _limit_speed(int16_t raw);
static void _set_speed(motor_dev_t *dev);
static void _set_dir(motor_dev_t *dev);

static const motor_ops_t g_motor_ops = {
    .pf_motor_init = motor_stm32_init,
    .pf_motor_set_dir = motor_set_dir,
    .pf_motor_set_speed = motor_set_speed,
    .pf_motor_update = motor_update,
};

static motor_dev_t g_motor_dev = {
    .ops = &g_motor_ops,
    .info = &g_motor_info,
};

static motor_status_t motor_stm32_init(motor_dev_t *dev)
{
    motor_info_t *info = (motor_info_t *)dev->info;

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    info->dir = MOTOR_FORWARD;
    info->raw_speed_a = 0;
    info->raw_speed_b = 0;
    info->real_speed_a = 0;
    info->real_speed_b = 0;
    motor_update(dev);

    return MOTOR_OK;
}

static motor_status_t motor_set_speed(motor_dev_t *dev, int16_t speed_a, int16_t speed_b)
{
    motor_info_t *info;

    if (NULL == dev)
    {
#ifdef MOTOR_DEBUG
        printf("MOTOR_ERROR: dev is NULL at %s\r\n", __FUNCTION__);
#endif
        return MOTOR_ERROR;
    }

    info = (motor_info_t *)dev->info;
    info->raw_speed_a = speed_a;
    info->raw_speed_b = speed_b;
    return MOTOR_OK;
}

static motor_status_t motor_set_dir(motor_dev_t *dev, motor_dir_t dir)
{
    motor_info_t *info;

    if (NULL == dev)
    {
#ifdef MOTOR_DEBUG
        printf("MOTOR_ERROR: dev is NULL at %s\r\n", __FUNCTION__);
#endif
        return MOTOR_ERROR;
    }

    info = (motor_info_t *)dev->info;
    info->dir = dir;
    return MOTOR_OK;
}

static motor_status_t motor_update(motor_dev_t *dev)
{
    if (NULL == dev)
    {
#ifdef MOTOR_DEBUG
        printf("MOTOR_ERROR: dev is NULL at %s\r\n", __FUNCTION__);
#endif
        return MOTOR_ERROR;
    }

    _set_speed(dev);
    _set_dir(dev);

    return MOTOR_OK;
}

static uint16_t _limit_speed(int16_t raw)
{
    uint16_t pulse;

    if (raw < 0)
        pulse = (uint16_t)(-raw);
    else
        pulse = (uint16_t)raw;

    if (pulse > 999U)
        pulse = 999U;

    return pulse;
}

static void _set_speed(motor_dev_t *dev)
{
    motor_info_t *info = (motor_info_t *)dev->info;
    uint16_t pulse_a;
    uint16_t pulse_b;

    pulse_a = _limit_speed(info->raw_speed_a);
    pulse_b = _limit_speed(info->raw_speed_b);

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse_a);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse_b);
}

static void _set_dir(motor_dev_t *dev)
{
    motor_info_t *info = (motor_info_t *)dev->info;

    switch (info->dir)
    {
        case MOTOR_FORWARD:
            HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
            break;

        case MOTOR_BACKWARD:
            HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
            break;

        case MOTOR_LEFT:
            HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
            break;

        case MOTOR_RIGHT:
            HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
            break;

        default:
            break;
    }
}

motor_dev_t *Motor_GetDefaultDevice(void)
{
    return &g_motor_dev;
}

motor_status_t Motor_Init(motor_dev_t *dev)
{
    if (NULL == dev || NULL == dev->info || NULL == dev->ops)
    {
#ifdef MOTOR_DEBUG
        printf("MOTOR_PARAMERROR: dev/info/ops is NULL at %s\r\n", __FUNCTION__);
#endif
        return MOTOR_PARAMERROR;
    }

    return dev->ops->pf_motor_init(dev);
}

motor_status_t Motor_SetDir(motor_dev_t *dev, motor_dir_t dir)
{
    if (NULL == dev || NULL == dev->ops || NULL == dev->ops->pf_motor_set_dir ||
        dir > MOTOR_RESERVE)
    {
#ifdef MOTOR_DEBUG
        printf("MOTOR_PARAMERROR: invalid param at %s\r\n", __FUNCTION__);
#endif
        return MOTOR_PARAMERROR;
    }
    return dev->ops->pf_motor_set_dir(dev, dir);
}

motor_status_t Motor_SetSpeed(motor_dev_t *dev, int16_t speed_a, int16_t speed_b)
{
    if (NULL == dev || NULL == dev->ops || NULL == dev->ops->pf_motor_set_speed)
    {
#ifdef MOTOR_DEBUG
        printf("MOTOR_PARAMERROR: dev/ops is NULL at %s\r\n", __FUNCTION__);
#endif
        return MOTOR_PARAMERROR;
    }
    return dev->ops->pf_motor_set_speed(dev, speed_a, speed_b);
}

motor_status_t Motor_Update(motor_dev_t *dev)
{
    if (NULL == dev || NULL == dev->ops || NULL == dev->ops->pf_motor_update)
    {
#ifdef MOTOR_DEBUG
        printf("MOTOR_PARAMERROR: dev/ops is NULL at %s\r\n", __FUNCTION__);
#endif
        return MOTOR_PARAMERROR;
    }
    return dev->ops->pf_motor_update(dev);
}
