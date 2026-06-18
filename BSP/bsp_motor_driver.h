/************************************************************************
 * @file bsp_motor_driver.h
 * @brief 电机驱动对外接口头文件
 *
 * 详细描述:
 * - 功能1: 定义电机驱动抽象层使用的数据类型与状态码
 * - 功能2: 对外提供默认电机设备获取、初始化、方向设置、速度设置和执行更新接口
 * - 功能3: 为后续更换底层芯片或驱动实现预留统一的 ops 抽象
 *
 * @author DFBH
 * @date 2026-06-18
 * @version 1.0.0
 *
 * @note 本文件只负责声明接口与类型, 不直接操作底层硬件寄存器
 * @warning 使用对外接口前, 需先通过 Motor_GetDefaultDevice 获取有效设备实例
 ************************************************************************/
#ifndef __BSP_MOTOR_DRIVER_H
#define __BSP_MOTOR_DRIVER_H

/**************** Include ****************/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include <stdint.h>

/*************** Typedef *****************/
typedef struct motor_dev motor_dev_t;

/**
 * @brief 电机驱动状态码
 *
 * 用于表示电机驱动接口执行结果:
 * - MOTOR_OK: 执行成功
 * - MOTOR_ERROR: 运行期错误, 例如底层执行失败
 * - MOTOR_PARAMERROR: 参数错误, 例如空指针或非法枚举值
 */
typedef enum
{
    MOTOR_OK,
    MOTOR_ERROR,
    MOTOR_PARAMERROR,
} motor_status_t;

/**
 * @brief 电机运动方向枚举
 *
 * 该方向定义面向当前双轮小车控制语义:
 * - MOTOR_FORWARD: 前进
 * - MOTOR_BACKWARD: 后退
 * - MOTOR_LEFT: 原地左转
 * - MOTOR_RIGHT: 原地右转
 * - MOTOR_RESERVE: 保留值, 用于参数边界判断
 */
typedef enum
{
    MOTOR_FORWARD = 0,
    MOTOR_BACKWARD,
    MOTOR_LEFT,
    MOTOR_RIGHT,
    MOTOR_RESERVE = 0xff,
} motor_dir_t;

/**
 * @brief 电机运行信息结构体
 *
 * 用于保存当前电机设备的软件侧状态:
 * - dir: 当前期望运动方向
 * - raw_speed: 当前期望原始速度值, 主要用于 PWM 输出
 * - real_speed: 当前实际速度值, 预留给编码器测速或闭环控制使用
 */
typedef struct
{
    motor_dir_t dir;
    uint16_t raw_speed;
    uint16_t real_speed;
} motor_info_t;

/**
 * @brief 电机驱动操作函数表
 *
 * 该结构体用于抽象不同底层实现的统一操作接口:
 * - pf_motor_init: 初始化底层外设与驱动状态
 * - pf_motor_set_dir: 设置目标方向
 * - pf_motor_set_speed: 设置目标速度
 * - pf_motor_update: 将缓存状态同步到底层硬件
 */
typedef struct
{
    motor_status_t (*pf_motor_init)(motor_dev_t *dev);
    motor_status_t (*pf_motor_set_dir)(motor_dev_t *dev, motor_dir_t dir);
    motor_status_t (*pf_motor_set_speed)(motor_dev_t *dev, int16_t speed);
    motor_status_t (*pf_motor_update)(motor_dev_t *dev);
} motor_ops_t;

/**
 * @brief 电机设备对象
 *
 * 该结构体用于描述一个具体电机设备实例:
 * - ops: 指向当前设备实现对应的操作函数表
 * - info: 指向当前设备的私有状态信息
 */
struct motor_dev
{
    const motor_ops_t *ops;
    void *info;
};

/**
 * @brief 获取默认电机设备实例
 *
 * 该接口返回当前工程默认使用的电机设备对象,
 * 上层 APP 层通过该实例访问电机驱动能力。
 *
 * @return 默认电机设备指针
 */
motor_dev_t *Motor_GetDefaultDevice(void);

/**
 * @brief 初始化电机设备
 *
 * 该接口用于完成电机驱动初始化, 包括 PWM 输出、编码器启动
 * 以及电机软件状态初始化。
 *
 * @param dev 电机设备指针
 * @return motor_status_t 初始化结果状态码
 */
motor_status_t Motor_Init(motor_dev_t *dev);

/**
 * @brief 设置电机方向
 *
 * 该接口只更新软件侧目标方向, 真正写入硬件需结合 Motor_Update 使用。
 *
 * @param dev 电机设备指针
 * @param dir 目标方向
 * @return motor_status_t 设置结果状态码
 */
motor_status_t Motor_SetDir(motor_dev_t *dev, motor_dir_t dir);

/**
 * @brief 设置电机速度
 *
 * 该接口只更新软件侧目标速度, 真正输出到 PWM 需结合 Motor_Update 使用。
 *
 * @param dev 电机设备指针
 * @param speed 目标速度值
 * @return motor_status_t 设置结果状态码
 */
motor_status_t Motor_SetSpeed(motor_dev_t *dev, uint16_t speed);

/**
 * @brief 同步电机状态到硬件
 *
 * 该接口将当前软件侧缓存的方向与速度参数
 * 统一写入到底层 GPIO 与 PWM 外设。
 *
 * @param dev 电机设备指针
 * @return motor_status_t 更新结果状态码
 */
motor_status_t Motor_Update(motor_dev_t *dev);

#endif // !__BSP_MOTOR_DRIVER_H
