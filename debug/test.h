/**
  ******************************************************************************
  * @file    test.h
  * @brief   麦轮底盘四方向调试测试接口（前 -> 后 -> 左 -> 右）
  *
  *          供 freertos.c 里创建调试任务使用。调试期间请注释掉原
  *          Chassis_Task，避免两个任务同时抢占 UART5 总线。
  ******************************************************************************
  */
#ifndef __TEST_H
#define __TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* ============================================================================
 *                            接口
 * ========================================================================== */

/** 麦轮四方向调试测试任务：上电后依次 前进 -> 后退 -> 左移 -> 右移 */
void StartTest_Task(void *argument);

/** 测试任务句柄与属性（供 MX_FREERTOS_Init() 创建任务时使用） */
extern osThreadId_t     Test_TaskHandle;
extern const osThreadAttr_t Test_Task_attributes;

#ifdef __cplusplus
}
#endif

#endif /* __TEST_H */
