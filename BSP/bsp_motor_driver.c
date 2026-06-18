/************************************************************************
 * @file bsp_motor_driver.c
 * @brief 电机驱动实现文件
 *
 * 详细描述:
 * - 功能1: 提供基于 STM32 定时器 PWM 与 GPIO 的电机驱动实现
 * - 功能2: 维护默认电机设备对象及其运行状态
 * - 功能3: 对上层暴露统一的初始化、方向设置、速度设置和更新接口
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

/**
 * @brief 默认电机设备的软件状态信息
 *
 * 该全局静态对象用于保存默认电机设备的当前方向、
 * 目标速度和预留的实际速度信息。
 */
static motor_info_t g_motor_info;

/* STM32 平台实现接口 */
static motor_status_t motor_stm32_init(motor_dev_t *dev);

/* 通用设备操作接口 */
static motor_status_t motor_set_speed(motor_dev_t *dev, int16_t speed);
static motor_status_t motor_set_dir(motor_dev_t *dev, motor_dir_t dir);
static motor_status_t motor_update(motor_dev_t *dev);

/* 底层硬件执行函数 */
static void _set_speed(motor_dev_t *dev);
static void _set_dir(motor_dev_t *dev);

/**
 * @brief 默认电机设备操作表
 *
 * 将对外暴露的统一电机接口映射到当前 STM32 平台实现。
 */
static const motor_ops_t g_motor_ops = {
    .pf_motor_init = motor_stm32_init,
    .pf_motor_set_dir = motor_set_dir,
    .pf_motor_set_speed = motor_set_speed,
    .pf_motor_update = motor_update,
};

/**
 * @brief 默认电机设备对象
 *
 * 该对象是当前工程默认使用的电机设备实例,
 * 内部绑定了操作表和状态信息块。
 */
static motor_dev_t g_motor_dev = {
    .ops = &g_motor_ops,
    .info = &g_motor_info,
};

/**
 * @brief STM32 平台电机初始化函数
 *
 * 功能说明:
 * - 启动 TIM2 的 PWM 输出通道
 * - 启动 TIM3/TIM4 的编码器接口
 * - 设置默认方向为前进
 * - 执行一次状态同步, 使初始方向生效
 *
 * @param dev 电机设备指针
 * @return motor_status_t 初始化结果状态码
 */
static motor_status_t motor_stm32_init(motor_dev_t *dev)
{
    motor_info_t *info = (motor_info_t *)dev->info;

    /* PWM */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); /* 电机 A PWM 输出 */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); /* 电机 B PWM 输出 */

    /* Encoder */
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); /* 电机 A 编码器 */
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); /* 电机 B 编码器 */

    /* Default state */
    info->dir = MOTOR_FORWARD;
    motor_update(dev);

    return MOTOR_OK;
}

/**
 * @brief 设置目标速度
 *
 * 功能说明:
 * - 检查设备指针合法性
 * - 将上层传入的目标速度缓存到软件状态中
 * - 不直接操作硬件, 需由 motor_update 或 Motor_Update 生效
 *
 * @param dev 电机设备指针
 * @param speed 目标速度值
 * @return motor_status_t 设置结果状态码
 */
static motor_status_t motor_set_speed(motor_dev_t *dev, int16_t speed)
{
    motor_info_t *info;

    if (NULL == dev)
    {
#ifdef DEBUG
        printf("MOTOR_ERROR: dev is NULL at %s\r\n", __FUNCTION__);
#endif
        return MOTOR_ERROR;
    }

    info = (motor_info_t *)dev->info;
    info->raw_speed = speed;
    return MOTOR_OK;
}

/**
 * @brief 设置目标方向
 *
 * 功能说明:
 * - 检查设备指针合法性
 * - 将上层传入的目标方向缓存到软件状态中
 * - 不直接操作硬件, 需由 motor_update 或 Motor_Update 生效
 *
 * @param dev 电机设备指针
 * @param dir 目标方向
 * @return motor_status_t 设置结果状态码
 */
static motor_status_t motor_set_dir(motor_dev_t *dev, motor_dir_t dir)
{
    motor_info_t *info;

    if (NULL == dev)
    {
#ifdef DEBUG
        printf("MOTOR_ERROR: dev is NULL at %s\r\n", __FUNCTION__);
#endif
        return MOTOR_ERROR;
    }

    info = (motor_info_t *)dev->info;
    info->dir = dir;
    return MOTOR_OK;
}

/**
 * @brief 执行电机状态更新
 *
 * 功能说明:
 * - 检查设备指针合法性
 * - 将缓存的速度写入 PWM 比较值
 * - 将缓存的方向写入 GPIO 引脚状态
 *
 * @param dev 电机设备指针
 * @return motor_status_t 更新结果状态码
 */
static motor_status_t motor_update(motor_dev_t *dev)
{
    if (NULL == dev)
    {
#ifdef DEBUG
        printf("MOTOR_ERROR: dev is NULL at %s\r\n", __FUNCTION__);
#endif
        return MOTOR_ERROR;
    }

    _set_speed(dev);
    _set_dir(dev);

    return MOTOR_OK;
}

/**
 * @brief 写入电机速度到底层 PWM
 *
 * 功能说明:
 * - 读取当前缓存的 raw_speed
 * - 将负值转换为绝对值脉冲宽度
 * - 对输出占空比做上限裁剪
 * - 同步写入 TIM2 的两个 PWM 通道
 *
 * @param dev 电机设备指针
 * @retval None
 */
static void _set_speed(motor_dev_t *dev)
{
    motor_info_t *info = (motor_info_t *)dev->info;
    uint16_t pulse;
    int16_t raw = info->raw_speed;

    if (raw < 0)
        pulse = -raw;
    else
        pulse = raw;

    if (pulse > 999)
        pulse = 999; /* PWM period = 1000 - 1 */

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse);
}

/**
 * @brief 写入电机方向到底层 GPIO
 *
 * 功能说明:
 * - 根据当前缓存方向选择引脚组合
 * - 控制 AIN1/AIN2 与 BIN1/BIN2 实现前进、后退和原地转向
 *
 * @param dev 电机设备指针
 * @retval None
 */
static void _set_dir(motor_dev_t *dev)
{
    motor_info_t *info = (motor_info_t *)dev->info;

    switch (info->dir)
    {
        case MOTOR_FORWARD:
            /* 电机 A: AIN1=1, AIN2=0 */
            HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
            /* 电机 B: BIN1=1, BIN2=0 */
            HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
            break;

        case MOTOR_BACKWARD:
            /* 电机 A: AIN1=0, AIN2=1 */
            HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
            /* 电机 B: BIN1=0, BIN2=1 */
            HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
            break;

        case MOTOR_LEFT:
            /* 原地左转: 左轮反转, 右轮正转 */
            HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
            break;

        case MOTOR_RIGHT:
            /* 原地右转: 左轮正转, 右轮反转 */
            HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
            break;

        default:
            break;
    }
}

/**
 * @brief 获取默认电机设备实例
 *
 * 供上层 APP 或其他模块获取当前工程的默认电机设备对象。
 *
 * @return motor_dev_t* 默认电机设备指针
 */
motor_dev_t *Motor_GetDefaultDevice(void)
{
    return &g_motor_dev;
}

/**
 * @brief 初始化电机设备
 *
 * 功能说明:
 * - 检查 dev、info、ops 是否有效
 * - 调用底层实现完成电机驱动初始化
 *
 * @param dev 电机设备指针
 * @return motor_status_t 初始化结果状态码
 */
motor_status_t Motor_Init(motor_dev_t *dev)
{
    if (NULL == dev || NULL == dev->info || NULL == dev->ops)
    {
#ifdef DEBUG
        printf("MOTOR_PARAMERROR: dev/info/ops is NULL at %s\r\n", __FUNCTION__);
#endif
        return MOTOR_PARAMERROR;
    }

    return dev->ops->pf_motor_init(dev);
}

/**
 * @brief 设置电机方向
 *
 * 功能说明:
 * - 检查设备、操作表和方向参数是否合法
 * - 调用底层实现缓存方向参数
 *
 * @param dev 电机设备指针
 * @param dir 目标方向
 * @return motor_status_t 设置结果状态码
 */
motor_status_t Motor_SetDir(motor_dev_t *dev, motor_dir_t dir)
{
    if (NULL == dev || NULL == dev->ops || NULL == dev->ops->pf_motor_set_dir ||
        dir > MOTOR_RESERVE)
    {
#ifdef DEBUG
        printf("MOTOR_PARAMERROR: invalid param at %s\r\n", __FUNCTION__);
#endif
        return MOTOR_PARAMERROR;
    }
    return dev->ops->pf_motor_set_dir(dev, dir);
}

/**
 * @brief 设置电机速度
 *
 * 功能说明:
 * - 检查设备和操作表是否合法
 * - 调用底层实现缓存速度参数
 *
 * @param dev 电机设备指针
 * @param speed 目标速度值
 * @return motor_status_t 设置结果状态码
 */
motor_status_t Motor_SetSpeed(motor_dev_t *dev, uint16_t speed)
{
    if (NULL == dev || NULL == dev->ops || NULL == dev->ops->pf_motor_set_speed)
    {
#ifdef DEBUG
        printf("MOTOR_PARAMERROR: dev/ops is NULL at %s\r\n", __FUNCTION__);
#endif
        return MOTOR_PARAMERROR;
    }
    return dev->ops->pf_motor_set_speed(dev, (int16_t)speed);
}

/**
 * @brief 更新电机输出状态
 *
 * 功能说明:
 * - 检查设备和操作表是否合法
 * - 调用底层实现将缓存状态同步到底层硬件
 *
 * @param dev 电机设备指针
 * @return motor_status_t 更新结果状态码
 */
motor_status_t Motor_Update(motor_dev_t *dev)
{
    if (NULL == dev || NULL == dev->ops || NULL == dev->ops->pf_motor_update)
    {
#ifdef DEBUG
        printf("MOTOR_PARAMERROR: dev/ops is NULL at %s\r\n", __FUNCTION__);
#endif
        return MOTOR_PARAMERROR;
    }
    return dev->ops->pf_motor_update(dev);
}
