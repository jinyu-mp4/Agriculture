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
/* Definitions for CHASSIS_Task */
osThreadId_t CHASSIS_TaskHandle;
const osThreadAttr_t CHASSIS_Task_attributes = {
  .name = "CHASSIS_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/* USER CODE END FunctionPrototypes */

void StartChassis_Task(void *argument);

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
  /* creation of CHASSIS_Task */
  CHASSIS_TaskHandle = osThreadNew(StartChassis_Task, NULL, &CHASSIS_Task_attributes);

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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
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
  for (;;)
  {
    g_yaw_deg     = IMU601_GetYaw();
    g_front_mm    = Laser_GetDistance(LASER_FRONT);
    g_right_mm    = Laser_GetDistance(LASER_RIGHT);
    g_front_valid = Laser_IsValid(LASER_FRONT);
    g_right_valid = Laser_IsValid(LASER_RIGHT);

    osDelay(20);   /* 20ms = 50Hz */
  }
  /* USER CODE END StartData_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* USER CODE END Application */
