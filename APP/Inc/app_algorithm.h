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

void AppAlgorithm_Init(void);
void AppAlgorithm_TaskCreate(void);

#endif // !__APP_ALGORITHM_H
