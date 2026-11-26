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
#include "icm42688.h"
#include "laser.h"
#include "zdt_motor.h"
#include "tower_motor.h"
#include "jetson_coord.h"
#include "claw.h"
#include "lcd_spi_154.h"
#include "lcd_fonts.h"
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IMU_LOOP_MS              10U
#define CHASSIS_FAST_RPM         50    /* 测试：先统一 50 */
#define CHASSIS_SLOW_RPM         50    /* 测试：先统一 50 */

/* 塔台捡拾控制参数 */
#define TOWER_TRIGGER_Y_ERR    100     /* |err_y| <= 100 触发塔台作业 */
#define TOWER_CORR_DEADBAND    10     /* 纠偏死区：|误差| <= 10 像素视为已对准 */
#define TOWER_CORR_IN_POS_CNT  3      /* 连续 N 帧在死区内才算对准 */
#define TOWER_CORR_K_Y         1.0f   /* 转台(电机5)纠 Y 的比例增益 */
#define TOWER_CORR_K_X         1.0f   /* 作业臂(电机6)纠 X 的比例增益 */
#define TOWER_CORR_MAX_PULSES  800    /* 单次纠偏最大脉冲（限幅，防冲过头） */
#define TOWER_CORR_MAX_MOVES   30     /* 每轴最多发送的纠偏动作数 */
#define TOWER_CORR_TIMEOUT_MS 6000     /* 单轴纠偏时间上限 */
#define TOWER_COORD_TIMEOUT_MS 500    /* 视觉断流时结束本轮纠偏 */
#define TOWER_AXIS_SETTLE_MS     100    /* 运动估时外的稳定余量 */
#define TOWER_DROP_POSITION   (-3600) /* 放蛋位置，相对转台零点的脉冲数 */
#define TOWER_CLAW_OPEN_MS      500    /* 对应夹爪打开指令中的 T0500 */
#define TOWER_LIFT_PULSES_PER_MM  164.95f   /* 实测：3200 脉冲下降 19.4mm */
#define TOWER_LIFT_DOWN_MM        106.0f     /* 下降高度(mm)，自由改这里 */
#define TOWER_LIFT_DOWN_PULSES    ((int32_t)(TOWER_LIFT_DOWN_MM * TOWER_LIFT_PULSES_PER_MM))
#define TOWER_LIFT_TOP_SETTLE_MS  1000U     /* 提升后到转台动作的最短等待 */

/* 旧塔台流程参数（保留旧代码使用） */
#define TOWER_CORR_MAX_ITER     30     /* 单轴纠偏最大循环次数 */
#define TOWER_CORR_DELAY_MS     150    /* 每次纠偏后等待电机+视觉更新的时间 */
#define TOWER_TURNTABLE_MS(p)   ((uint32_t)((uint32_t)(p) * 568UL / 1000UL))  /* 转台(33RPM)移动毫秒 */
#define TOWER_LIFT_TRAVEL_MS    3000   /* 升降到位等待 */
#define TOWER_POST_DELAY_MS     2000   /* 作业完成后恢复底盘前的等待 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static const osThreadAttr_t s_imu_task_attributes = {
  .name = "IMU_Task",
  .stack_size = 512 * 4,
  .priority = osPriorityHigh2,
};

/* 传感器共享数据：由 DataTask 定时刷新，其他任务只读 */
volatile uint16_t g_front_mm;
volatile uint16_t g_right_mm;
volatile uint16_t g_back_mm;
volatile bool     g_front_valid;
volatile bool     g_right_valid;
volatile bool     g_back_valid;

/* 上位机 Jetson 传来的物块坐标（Data_Task 刷新，Tower_Task 作业结束时清零） */
volatile int32_t  g_coord_x;
volatile int32_t  g_coord_y;
volatile bool     g_coord_valid;

/* 两个任务通过请求和停车确认交接 UART5。 */
volatile bool     g_tower_working = false;
volatile bool     g_chassis_parked = false;
static volatile bool s_route_active;

/* 底盘状态机当前状态号（Chassis_Task 写，Data_Task 只读显示） */
volatile int32_t  g_state = 0;

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
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Tower_Task */
osThreadId_t Tower_TaskHandle;
const osThreadAttr_t Tower_Task_attributes = {
  .name = "Tower_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void StartImuTask(void *argument);
static bool Tower_WaitFreshPoint(uint32_t *frame, CoordPoint *point);
static uint32_t Tower_StartReturn(TowerMotorID id, int32_t position);
/* USER CODE END FunctionPrototypes */

void StartChassis_Task(void *argument);
void StartData_Task(void *argument);
void StartTask03(void *argument);

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

  /* creation of Tower_Task */
  Tower_TaskHandle = osThreadNew(StartTask03, NULL, &Tower_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  osThreadNew(StartImuTask, NULL, &s_imu_task_attributes);
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
  g_state = 0;

  /* 上电初始化：电机、激光、夹爪反馈接收 */
  ZDT_Motor_Init();
  Laser_Init();
  Claw_Init();

  /* 单调机械臂：暂时注释掉底盘运动（按键等待 + 状态机），只保留初始化 */
#if 0
  /* 上电后等待按下 PC1 按键才开始行驶（PC1 另一端接 GND，低电平有效） */
  while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) != GPIO_PIN_RESET)
  {
    osDelay(20);
  }
  osDelay(3000);
  s_route_active = true;

  /* 各状态的停止条件：所有激光距离都在指定区间内才切换 */
  static const ZDT_AuxCond c_right_le100_back_ge200[2]     = { {LASER_RIGHT, 0, 100},   {LASER_BACK, 200, 0} };
  static const ZDT_AuxCond c_back_le80[1]                = { {LASER_BACK, 0, 80} };
  static const ZDT_AuxCond c_right_ge775_front_ge200[2]  = { {LASER_RIGHT, 775, 0},  {LASER_FRONT, 200, 0} };
  static const ZDT_AuxCond c_front_le80_right_ge200[2]   = { {LASER_FRONT, 0, 80},   {LASER_RIGHT, 200, 0} };
  static const ZDT_AuxCond c_right_ge1455_back_ge200[2]  = { {LASER_RIGHT, 1455, 0}, {LASER_BACK, 200, 0} };
  static const ZDT_AuxCond c_back_le80_right_ge200[2]    = { {LASER_BACK, 0, 80},    {LASER_RIGHT, 200, 0} };
  static const ZDT_AuxCond c_right_ge2130_front_ge200[2] = { {LASER_RIGHT, 2130, 0}, {LASER_FRONT, 200, 0} };
  static const ZDT_AuxCond c_right_ge2900_back_ge200[2]  = { {LASER_RIGHT, 2900, 0}, {LASER_BACK, 200, 0} };
  static const ZDT_AuxCond c_right_ge3540_front_ge200[2] = { {LASER_RIGHT, 3540, 0}, {LASER_FRONT, 200, 0} };

  for (;;)
  {
    bool moved;
    switch (g_state)
    {
      case 0:   /* 向右，直到 right<100 且 back>200 */
        moved = ZDT_Chassis_Drive(MOVE_RIGHT, CHASSIS_FAST_RPM, c_right_le100_back_ge200, 2, LASER_FRONT, 80);
        break;
      case 1:   /* 向后，直到 back<80 */
        moved = ZDT_Chassis_Drive(MOVE_BACKWARD, CHASSIS_FAST_RPM, c_back_le80, 1, LASER_RIGHT, 80);
        break;
      case 2:   /* 向左，直到 right>775 且 front>200 */
        moved = ZDT_Chassis_Drive(MOVE_LEFT, CHASSIS_SLOW_RPM, c_right_ge775_front_ge200, 2, LASER_BACK, 80);
        break;
      case 3:   /* 向前，直到 front<80 且 right>200 */
        moved = ZDT_Chassis_Drive(MOVE_FORWARD, CHASSIS_SLOW_RPM, c_front_le80_right_ge200, 2, LASER_RIGHT, 50);
        break;
      case 4:   /* 向左，直到 right>1455 且 back>200 */
        moved = ZDT_Chassis_Drive(MOVE_LEFT, CHASSIS_SLOW_RPM, c_right_ge1455_back_ge200, 2, LASER_FRONT, 80);
        break;
      case 5:   /* 向后，直到 back<80 且 right>200 */
        moved = ZDT_Chassis_Drive(MOVE_BACKWARD, CHASSIS_SLOW_RPM, c_back_le80_right_ge200, 2, LASER_RIGHT, 50);
        break;
      case 6:   /* 向左，直到 right>2130 且 front>200 */
        moved = ZDT_Chassis_Drive(MOVE_LEFT, CHASSIS_SLOW_RPM, c_right_ge2130_front_ge200, 2, LASER_BACK, 80);
        break;
      case 7:   /* 向前，直到 front<80 且 right>200 */
        moved = ZDT_Chassis_Drive(MOVE_FORWARD, CHASSIS_SLOW_RPM, c_front_le80_right_ge200, 2, LASER_RIGHT, 50);
        break;
      case 8:   /* 向左，直到 right>2900 且 back>200 */
        moved = ZDT_Chassis_Drive(MOVE_LEFT, CHASSIS_SLOW_RPM, c_right_ge2900_back_ge200, 2, LASER_FRONT, 80);
        break;
      case 9:   /* 向后，直到 back<80 且 right>200 */
        moved = ZDT_Chassis_Drive(MOVE_BACKWARD, CHASSIS_SLOW_RPM, c_back_le80_right_ge200, 2, LASER_RIGHT, 50);
        break;
      case 10:  /* 向左，直到 right>3540 且 front>200 */
        moved = ZDT_Chassis_Drive(MOVE_LEFT, CHASSIS_SLOW_RPM, c_right_ge3540_front_ge200, 2, LASER_BACK, 80);
        break;
      case 11:  /* 向前，直到 front<80 且 right>200 */
        moved = ZDT_Chassis_Drive(MOVE_FORWARD, CHASSIS_SLOW_RPM, c_front_le80_right_ge200, 2, LASER_RIGHT, 50);
        break;

      default:  /* 状态12：全部完成，停车待机 */
        s_route_active = false;
        ZDT_Chassis_Stop();
        osDelay(1000);
        continue;
    }

    if (!moved)
    {
      g_state = -1;
      continue;
    }

    /* 停下后等待 700ms */
    osDelay(700);
    g_state++;
  }
#endif

  for (;;)
  {
    osDelay(1000);
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
  /* 显示和视觉坐标刷新由本任务处理，航向采样使用独立任务。 */
  static const char *const yaw_status_text[] = {
    [ICM42688_STARTING] = "INIT",
    [ICM42688_SCL_LOW] = "SCL0",
    [ICM42688_SDA_LOW] = "SDA0",
    [ICM42688_BUS_BUSY] = "BUSY",
    [ICM42688_NO_ACK] = "NACK",
    [ICM42688_ID_ERROR] = "ID",
    [ICM42688_CONFIG_ERROR] = "CFG",
    [ICM42688_CALIBRATING] = "CAL",
    [ICM42688_NOT_STILL] = "STILL",
    [ICM42688_DATA_ERROR] = "DATA",
    [ICM42688_READY] = "DATA", /* 已校准，但最近的样本过期 */
  };

  /* LCD 显示初始化：设置字体与数字补位方式 */
  LCD_SetAsciiFont(&ASCII_Font16);
  LCD_ShowNumMode(Fill_Space);

  for (;;)
  {
    CoordPoint pt;

    g_front_mm    = Laser_GetDistance(LASER_FRONT);
    g_right_mm    = Laser_GetDistance(LASER_RIGHT);
    g_back_mm     = Laser_GetDistance(LASER_BACK);
    g_front_valid = Laser_IsValid(LASER_FRONT);
    g_right_valid = Laser_IsValid(LASER_RIGHT);
    g_back_valid  = Laser_IsValid(LASER_BACK);

    /* 实时读取上位机传来的物块坐标 */
    g_coord_valid = Coord_GetPoint(&pt);
    if (g_coord_valid)
    {
        g_coord_x = pt.x;
        g_coord_y = pt.y;
    }

    /* 将读取值显示在 LCD 上 */
    LCD_DisplayString(10, 60, "Front:");
    LCD_DisplayNumber(70, 60, (int32_t)g_front_mm, 5);

    LCD_DisplayString(10, 90, "Right:");
    LCD_DisplayNumber(70, 90, (int32_t)g_right_mm, 5);

    LCD_DisplayString(10, 120, "Back:");
    LCD_DisplayNumber(70, 120, (int32_t)g_back_mm, 5);

    LCD_DisplayString(10, 150, "X:");
    LCD_DisplayNumber(70, 150, g_coord_x, 5);

    LCD_DisplayString(10, 180, "Y:");
    LCD_DisplayNumber(70, 180, g_coord_y, 5);

    LCD_DisplayString(10, 210, "State:");
    LCD_DisplayNumber(70, 210, g_state, 3);

    float yaw;
    char yaw_text[6];
    if (ICM42688_GetYaw(&yaw))
    {
      unsigned int tenths = (unsigned int)(yaw * 10.0f + 0.5f) % 3600U;
      snprintf(yaw_text, sizeof(yaw_text), "%3u.%u", tenths / 10U, tenths % 10U);
    }
    else
    {
      ICM42688_Status status = ICM42688_GetStatus();
      if (status == ICM42688_BUS_ERROR)
        snprintf(yaw_text, sizeof(yaw_text), "B%04X", (unsigned int)(ICM42688_GetBusError() & 0xFFFFU));
      else
        snprintf(yaw_text, sizeof(yaw_text), "%5s", yaw_status_text[status]);
    }
    LCD_DisplayString(116, 210, "YAW:");
    LCD_DisplayString(156, 210, yaw_text);

    osDelay(20);   /* 20ms = 50Hz */
  }
  /* USER CODE END StartData_Task */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the Tower_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  int32_t err_x, err_y;
  int32_t off_5 = 0, off_6 = 0, off_7 = 0;   /* 各电机相对初始位置的累计脉冲偏移 */
  bool initialized = false;

  for (;;)
  {
    /* 1. 等触发：收到有效坐标，且 |err_y| <= 100 才让电机转动 */
    for (;;)
    {
      if (g_coord_valid)
      {
        err_y = g_coord_y - TOWER_Y_TARGET;
        if (err_y > -TOWER_TRIGGER_Y_ERR && err_y < TOWER_TRIGGER_Y_ERR)
        {
          break;
        }
      }
      osDelay(20);
    }

    osDelay(100);

    if (!initialized)
    {
      TowerMotor_Init();
      TowerMotor_Enable(TOWER_TURNTABLE, true);
      TowerMotor_Enable(TOWER_ARM, true);
      TowerMotor_Enable(TOWER_LIFT, true);
      osDelay(500);   /* 等电机使能后稳定，再设零点，避免零点漂移导致回零过冲 */
      TowerMotor_ZeroPosition(TOWER_TURNTABLE);   /* 上电位置设为绝对零点（供绝对回零用） */
      TowerMotor_ZeroPosition(TOWER_ARM);
      TowerMotor_ZeroPosition(TOWER_LIFT);
      Claw_Open();
      osDelay(500);
      initialized = true;
    }

    /* 2. 电机 5（转台）：纠偏 Y，记录偏移（每轮等新一帧坐标再纠） */
    int in_pos_y = 0;
    uint32_t frame = Coord_GetFrameCount();
    CoordPoint point;
    for (int i = 0; i < TOWER_CORR_MAX_ITER; i++)
    {
      if (!Tower_WaitFreshPoint(&frame, &point)) break;   /* 视觉断流，结束纠偏 */

      err_y = point.y - TOWER_Y_TARGET;
      if (err_y > -TOWER_CORR_DEADBAND && err_y < TOWER_CORR_DEADBAND)
      {
        if (++in_pos_y >= TOWER_CORR_IN_POS_CNT) break;
        continue;
      }
      in_pos_y = 0;

      int32_t pulses = (int32_t)((float)err_y * TOWER_CORR_K_Y);
      if (pulses >  TOWER_CORR_MAX_PULSES) pulses =  TOWER_CORR_MAX_PULSES;
      if (pulses < -TOWER_CORR_MAX_PULSES) pulses = -TOWER_CORR_MAX_PULSES;
      pulses = -pulses;   /* 电机5 纠偏方向取反 */

      TowerMotor_Move(TOWER_TURNTABLE, pulses);
      off_5 += pulses;
    }

    /* 3. 电机 6（作业臂）：纠偏 X，记录偏移（每轮等新一帧坐标再纠） */
    int in_pos_x = 0;
    frame = Coord_GetFrameCount();
    for (int i = 0; i < TOWER_CORR_MAX_ITER; i++)
    {
      if (!Tower_WaitFreshPoint(&frame, &point)) break;   /* 视觉断流，结束纠偏 */

      err_x = point.x - TOWER_X_TARGET;
      if (err_x > -TOWER_CORR_DEADBAND && err_x < TOWER_CORR_DEADBAND)
      {
        if (++in_pos_x >= TOWER_CORR_IN_POS_CNT) break;
        continue;
      }
      in_pos_x = 0;

      int32_t pulses = (int32_t)((float)err_x * TOWER_CORR_K_X);
      if (pulses >  TOWER_CORR_MAX_PULSES) pulses =  TOWER_CORR_MAX_PULSES;
      if (pulses < -TOWER_CORR_MAX_PULSES) pulses = -TOWER_CORR_MAX_PULSES;

      TowerMotor_Move(TOWER_ARM, pulses);
      off_6 += pulses;
    }

    /* 4. 电机 7 下降 -> 夹爪闭合 -> 等 1s */
    TowerMotor_Move(TOWER_LIFT, -TOWER_LIFT_DOWN_PULSES);
    off_7 -= TOWER_LIFT_DOWN_PULSES;
    osDelay(TowerMotor_TravelTimeMs(TOWER_LIFT, TOWER_LIFT_DOWN_PULSES) + 500);   /* 按实际速度+加速度算到位时间 */
    Claw_Close();           /* 下降到位后立即闭合 */
    osDelay(1000);          /* 等 1s */

    /* 5. 电机 7 复位 -> 复位后等 1s */
    TowerMotor_Move(TOWER_LIFT, -off_7);
    off_7 = 0;
    osDelay(TowerMotor_TravelTimeMs(TOWER_LIFT, TOWER_LIFT_DOWN_PULSES) + 500);   /* 等升降真正回到顶再走下一步 */
    osDelay(1000);          /* 复位后等 1s */

    /* 5.5 作业臂先回零（放蛋前收回，避免转台移动时作业臂还伸着碰结构） */
    {
      uint32_t arm_ms = Tower_StartReturn(TOWER_ARM, off_6);
      off_6 = 0;
      if (arm_ms) osDelay(arm_ms);
    }

    /* 6. 转台移到放蛋位置（绝对 -3600，避免相对累计漂移） */
    {
      int32_t  dist = -3600 - off_5;
      TowerMotor_MoveAbsolute(TOWER_TURNTABLE, TOWER_DROP_POSITION);
      off_5 = TOWER_DROP_POSITION;
      osDelay(TowerMotor_TravelTimeMs(TOWER_TURNTABLE, dist) + 500);   /* 按实际速度算时间 +500ms 余量 */
    }

    /* 7. 夹爪打开（放蛋） */
    Claw_Open();
    osDelay(500);

    /* 8. 转台回零（作业臂已在 5.5 回零） */
    {
      uint32_t turntable_ms = Tower_StartReturn(TOWER_TURNTABLE, off_5);
      off_5 = 0;
      off_7 = 0;
      if (turntable_ms) osDelay(turntable_ms);
    }

    /* 9. 回零等待结束后，清掉本轮的视觉坐标。 */
    taskENTER_CRITICAL();
    Coord_ClearPoint();
    g_coord_x = 0;
    g_coord_y = 0;
    g_coord_valid = false;
    taskEXIT_CRITICAL();

    /* 恢复底盘，等下一次触发。 */
    g_tower_working = false;
    osDelay(TOWER_POST_DELAY_MS);
  }
  /* USER CODE END StartTask03 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
static bool Tower_WaitFreshPoint(uint32_t *frame, CoordPoint *point)
{
  uint32_t start = osKernelGetTickCount();
  do
  {
    if (Coord_GetPoint(point) && point->frame != *frame)
    {
      *frame = point->frame;
      return true;
    }
    osDelay(5);
  } while (osKernelGetTickCount() - start < TOWER_COORD_TIMEOUT_MS);
  return false;
}

static uint32_t Tower_StartReturn(TowerMotorID id, int32_t position)
{
  if (position == 0) return 0;

  /* 绝对零点可重复发送，UART 恢复后继续本次回零。 */
  while (!TowerMotor_MoveAbsolute(id, 0)) osDelay(50);
  return TowerMotor_TravelTimeMs(id, position) + TOWER_AXIS_SETTLE_MS;
}

static void StartImuTask(void *argument)
{
  (void)argument;
  for (;;)
  {
    if (ICM42688_Init())
    {
      while (!ICM42688_CalibrateYaw() && ICM42688_GetStatus() == ICM42688_NOT_STILL)
        osDelay(250);

      uint32_t next_tick = osKernelGetTickCount();
      while (ICM42688_GetStatus() == ICM42688_READY)
      {
        next_tick += IMU_LOOP_MS;
        if (osDelayUntil(next_tick) != osOK) next_tick = osKernelGetTickCount();
        ICM42688_UpdateYaw();
      }
    }

    /* 行驶中失联时保留上电零点，路线结束后再重新校准。 */
    while (s_route_active) osDelay(IMU_LOOP_MS);
    osDelay(250);
  }
}
/* USER CODE END Application */

