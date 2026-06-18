/************************************************************************
 * @file bsp_motor_driver.c
 * @brief 电机驱动c文件
 *
 * 详细描述:
 * - 功能1: 具体描述
 * - 功能2: 具体描述
 *
 * @author DFBH
 * @date 2026-06-18
 * @version 1.0.0
 *
 * @note 备注信息 (可选)
 * @warning 警告信息 (可选)
 ************************************************************************/
#include "bsp_motor_driver.h"
#include "tim.h"

Motor motor;

static void _set_speed(Motor *p_motor)
{
    uint16_t pulse;
    int16_t raw = p_motor->raw_speed;

    if (raw < 0)
        pulse = -raw;
    else
        pulse = raw;

    if (pulse > 999)
        pulse = 999; /* Period = 1000-1 */

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse);
}
static void _set_dir(Motor *p_motor)
{
    switch (p_motor->dir)
    {
        case FORWARD:
            /* 电机A: AIN1=1, AIN2=0 */
            HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
            /* 电机B: BIN1=1, BIN2=0 */
            HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
            break;

        case BACKWARD:
            /* 电机A: AIN1=0, AIN2=1 */
            HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
            /* 电机B: BIN1=0, BIN2=1 */
            HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
            break;

        case LEFT:
            /* 原地左转: 左轮倒转, 右轮正转 */
            HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
            break;

        case RIGHT:
            /* 原地右转: 左轮正转, 右轮倒转 */
            HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
            break;

        default:
            break;
    }
}
void Motor_Init()
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // 电机A PWM
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // 电机B PWM

    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // 电机A编码器
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); // 电机B编码器

    motor.dir = FORWARD;
    _set_dir(&motor);
}
void Motor_SetDir(MotorDir dir)
{
    motor.dir = dir;
}
void Motor_SetSpeed(int16_t speed)
{
    motor.raw_speed = speed;
}
void Motor_Update(void)
{

    _set_dir(&motor);
    _set_speed(&motor);
}