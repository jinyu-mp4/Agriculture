/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "imu601.h"
#include "laser.h"
#include "zdt_motor.h"
#include "lcd_spi_154.h"
#include "lcd_fonts.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* 传感器共享数据：由 DataTask 定时刷新，其他任务只读 */
volatile float    g_yaw_deg;
volatile uint16_t g_front_mm;
volatile uint16_t g_right_mm;
volatile bool     g_front_valid;
volatile bool     g_right_valid;

/* USER CODE END Variables */
/* Definitions for Chassis_Task */
osThreadId_t Chassis_TaskHandle;
const osThreadAttr_t Chassis_Task_attributes = {
  .name = "Chassis_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal1,
};
/* Definitions for Data_Task */
osThreadId_t Data_TaskHandle;
const osThreadAttr_t Data_Task_attributes = {
  .name = "Data_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void chassis_move_until(ZDT_MoveDir dir, LaserID laser, bool until_le, uint16_t threshold_mm);
/* USER CODE END FunctionPrototypes */

void StartChassis_Task(void *argument);
void StartData_Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Chassis_Task */
  Chassis_TaskHandle = osThreadNew(StartChassis_Task, NULL, &Chassis_Task_attributes);

  /* creation of Data_Task */
  Data_TaskHandle = osThreadNew(StartData_Task, NULL, &Data_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartChassis_Task */
/**
  * @brief  Function implementing the CHASSIS_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartChassis_Task */
void StartChassis_Task(void *argument)
{
  /* USER CODE BEGIN StartChassis_Task */
  int state = 0;

  /* 上电初始化：电机、激光、陀螺仪（陀螺仪复位+校准内部含约 500ms 延时） */
  ZDT_Motor_Init();
  Laser_Init();
  IMU601_Init();

  /* 等待 5000ms，等待陀螺仪初始化/稳定 */
  osDelay(5000);

  for (;;)
  {
    switch (state)
    {
      case 0:   /* 向右行驶，直到 right <= 359.5cm (3595mm) */
        chassis_move_until(MOVE_RIGHT, LASER_RIGHT, true, 3595);
        break;
      case 1:   /* 向后行驶，直到 front >= 266cm (2660mm) */
        chassis_move_until(MOVE_BACKWARD, LASER_FRONT, false, 2660);
        break;
      case 2:   /* 向右行驶，直到 right <= 289.5cm (2895mm) */
        chassis_move_until(MOVE_RIGHT, LASER_RIGHT, true, 2895);
        break;
      case 3:   /* 向前行驶，直到 front <= 6cm (60mm) */
        chassis_move_until(MOVE_FORWARD, LASER_FRONT, true, 60);
        break;
      case 4:   /* 向右行驶，直到 right <= 219.5cm (2195mm) */
        chassis_move_until(MOVE_RIGHT, LASER_RIGHT, true, 2195);
        break;
      case 5:   /* 向后行驶，直到 front >= 266cm (2660mm) */
        chassis_move_until(MOVE_BACKWARD, LASER_FRONT, false, 2660);
        break;
      case 6:   /* 向右行驶，直到 right <= 149.5cm (1495mm) */
        chassis_move_until(MOVE_RIGHT, LASER_RIGHT, true, 1495);
        break;
      case 7:   /* 向前行驶，直到 front <= 6cm (60mm) */
        chassis_move_until(MOVE_FORWARD, LASER_FRONT, true, 60);
        break;
      case 8:   /* 向右行驶，直到 right <= 79.5cm (795mm) */
        chassis_move_until(MOVE_RIGHT, LASER_RIGHT, true, 795);
        break;
      case 9:   /* 向后行驶，直到 front >= 266cm (2660mm) */
        chassis_move_until(MOVE_BACKWARD, LASER_FRONT, false, 2660);
        break;
      case 10:  /* 向右行驶，直到 right <= 9.5cm (95mm) */
        chassis_move_until(MOVE_RIGHT, LASER_RIGHT, true, 95);
        break;
      case 11:  /* 向前行驶，直到 front <= 6cm (60mm) */
        chassis_move_until(MOVE_FORWARD, LASER_FRONT, true, 60);
        break;
      case 12:  /* 向左行驶，直到 right >= 400cm (4000mm) */
        chassis_move_until(MOVE_LEFT, LASER_RIGHT, false, 4000);
        break;

      default:  /* 全部动作完成：停车待机 */
        ZDT_Chassis_Stop();
        osDelay(1000);
        continue;
    }

    /* 停下后等待 500ms */
    osDelay(500);
    state++;
  }
  /* USER CODE END StartChassis_Task */
}

/* USER CODE BEGIN Header_StartData_Task */
/**
  * @brief  Function implementing the DataTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartData_Task */
void StartData_Task(void *argument)
{
  /* USER CODE BEGIN StartData_Task */
  /* 传感器数据采集任务：50Hz 定时读取 yaw + 两路激光距离，写入共享变量 */

  /* LCD 显示初始化：设置字体与数字补位方式 */
  LCD_SetAsciiFont(&ASCII_Font16);
  LCD_ShowNumMode(Fill_Space);

  for (;;)
  {
    g_yaw_deg     = IMU601_GetYaw();
    g_front_mm    = Laser_GetDistance(LASER_FRONT);
    g_right_mm    = Laser_GetDistance(LASER_RIGHT);
    g_front_valid = Laser_IsValid(LASER_FRONT);
    g_right_valid = Laser_IsValid(LASER_RIGHT);

    /* 将三个读取值显示在 LCD 上 */
    LCD_DisplayString(10, 30, "Yaw  :");
    LCD_DisplayDecimals(70, 30, (double)g_yaw_deg, 7, 1);

    LCD_DisplayString(10, 60, "Front:");
    LCD_DisplayNumber(70, 60, (int32_t)g_front_mm, 5);

    LCD_DisplayString(10, 90, "Right:");
    LCD_DisplayNumber(70, 90, (int32_t)g_right_mm, 5);

    osDelay(20);   /* 20ms = 50Hz */
  }
  /* USER CODE END StartData_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* 开环行驶的“足够远”距离(mm)：先下发该距离的位置命令，行驶途中由激光触发停车 */
#define CHASSIS_FAR_MM   (100000.0f)

/* 向 dir 方向连续行驶，直到指定激光满足阈值后停车（开环位置模式，不使用 PID）。
 *   until_le = true  : 距离 <= threshold_mm 时停车
 *   until_le = false : 距离 >= threshold_mm 时停车
 *   激光无效(尚未收到有效帧/超量程)时继续行驶，避免把 0 误判为“很近”。 */
static void chassis_move_until(ZDT_MoveDir dir, LaserID laser, bool until_le, uint16_t threshold_mm)
{
    ZDT_Chassis_Move(dir, CHASSIS_FAR_MM);   /* 开环启动，不使用 PID */

    for (;;)
    {
        uint16_t d  = Laser_GetDistance(laser);
        bool     ok = Laser_IsValid(laser) &&
                      (until_le ? (d <= threshold_mm) : (d >= threshold_mm));

        if (ok)
        {
            break;
        }
        osDelay(10);   /* 10ms 轮询一次 */
    }

    ZDT_Chassis_Stop();
}
/* USER CODE END Application */

