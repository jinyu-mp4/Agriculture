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
volatile uint16_t g_front_mm;
volatile uint16_t g_right_mm;
volatile uint16_t g_back_mm;
volatile bool     g_front_valid;
volatile bool     g_right_valid;
volatile bool     g_back_valid;

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

  /* 上电初始化：电机、激光 */
  ZDT_Motor_Init();
  Laser_Init();

  /* 上电后等待 5s：待激光测距稳定后再开始走 */
  osDelay(5000);

  for (;;)
  {
    switch (state)
    {
      case 0:   /* 向右行驶 + front 纵向居中(70mm)，直到 right <= 359.5cm (3595mm) */
        ZDT_Chassis_Drive_LatCenter(MOVE_RIGHT, ZDT_DEFAULT_SPEED_RPM, LASER_FRONT, 3595);
        break;
      case 1:   /* 向后行驶（横向居中），直到 back <= 5cm (70mm) */
        ZDT_Chassis_Drive_LaserCenter(MOVE_BACKWARD, ZDT_DEFAULT_SPEED_RPM, LASER_BACK, true, 80);
        break;
      case 2:   /* 向右行驶 + back 纵向居中(70mm)，直到 right <= 289.5cm (2895mm) */
        ZDT_Chassis_Drive_LatCenter(MOVE_RIGHT, ZDT_DEFAULT_SPEED_RPM, LASER_BACK, 2895);
        break;
      case 3:   /* 向前行驶（横向居中），直到 front <= 5cm (70mm) */
        ZDT_Chassis_Drive_LaserCenter(MOVE_FORWARD, ZDT_DEFAULT_SPEED_RPM, LASER_FRONT, true, 70);
        break;
      case 4:   /* 向右行驶 + front 纵向居中(70mm)，直到 right <= 219.5cm (2195mm) */
        ZDT_Chassis_Drive_LatCenter(MOVE_RIGHT, ZDT_DEFAULT_SPEED_RPM, LASER_FRONT, 2195);
        break;
      case 5:   /* 向后行驶（横向居中），直到 back <= 5cm (70mm) */
        ZDT_Chassis_Drive_LaserCenter(MOVE_BACKWARD, ZDT_DEFAULT_SPEED_RPM, LASER_BACK, true, 80);
        break;
      case 6:   /* 向右行驶 + back 纵向居中(70mm)，直到 right <= 149.5cm (1495mm) */
        ZDT_Chassis_Drive_LatCenter(MOVE_RIGHT, ZDT_DEFAULT_SPEED_RPM, LASER_BACK, 1495);
        break;
      case 7:   /* 向前行驶（横向居中），直到 front <= 5cm (70mm) */
        ZDT_Chassis_Drive_LaserCenter(MOVE_FORWARD, ZDT_DEFAULT_SPEED_RPM, LASER_FRONT, true, 70);
        break;
      case 8:   /* 向右行驶 + front 纵向居中(70mm)，直到 right <= 79.5cm (795mm) */
        ZDT_Chassis_Drive_LatCenter(MOVE_RIGHT, ZDT_DEFAULT_SPEED_RPM, LASER_FRONT, 795);
        break;
      case 9:   /* 向后行驶（横向居中），直到 back <= 5cm (70mm) */
        ZDT_Chassis_Drive_LaserCenter(MOVE_BACKWARD, ZDT_DEFAULT_SPEED_RPM, LASER_BACK, true, 80);
        break;
      case 10:  /* 向右行驶 + back 纵向居中(70mm)，直到 right <= 9.5cm (95mm) */
        ZDT_Chassis_Drive_LatCenter(MOVE_RIGHT, ZDT_DEFAULT_SPEED_RPM, LASER_BACK, 95);
        break;
      case 11:  /* 向前行驶（横向居中），直到 front <= 5cm (70mm) */
        ZDT_Chassis_Drive_LaserCenter(MOVE_FORWARD, ZDT_DEFAULT_SPEED_RPM, LASER_FRONT, true, 70);
        break;
      case 12:  /* 向左行驶 4m + 前侧激光纵向居中（纵向 PID，目标 70mm） */
        ZDT_Chassis_Drive_LongCenter(MOVE_LEFT, ZDT_DEFAULT_SPEED_RPM, 4200.0f);
        break;

      default:  /* 全部动作完成：停车待机 */
        ZDT_Chassis_Stop();
        osDelay(1000);
        continue;
    }

    /* 停下后等待 700ms */
    osDelay(700);
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
  /* 传感器数据采集任务：70Hz 定时读取三路激光距离，写入共享变量 */

  /* LCD 显示初始化：设置字体与数字补位方式 */
  LCD_SetAsciiFont(&ASCII_Font16);
  LCD_ShowNumMode(Fill_Space);

  for (;;)
  {
    g_front_mm    = Laser_GetDistance(LASER_FRONT);
    g_right_mm    = Laser_GetDistance(LASER_RIGHT);
    g_back_mm     = Laser_GetDistance(LASER_BACK);
    g_front_valid = Laser_IsValid(LASER_FRONT);
    g_right_valid = Laser_IsValid(LASER_RIGHT);
    g_back_valid  = Laser_IsValid(LASER_BACK);

    /* 将三个读取值显示在 LCD 上 */
    LCD_DisplayString(10, 60, "Front:");
    LCD_DisplayNumber(70, 60, (int32_t)g_front_mm, 5);

    LCD_DisplayString(10, 90, "Right:");
    LCD_DisplayNumber(70, 90, (int32_t)g_right_mm, 5);

    LCD_DisplayString(10, 120, "Back:");
    LCD_DisplayNumber(70, 120, (int32_t)g_back_mm, 5);

    osDelay(20);   /* 20ms = 70Hz */
  }
  /* USER CODE END StartData_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* USER CODE END Application */

