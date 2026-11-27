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
/* 麦轮按运动方向选用电机基础转速（RPM），满足各段停车条件后直接停车。 */
#define CHASSIS_STRAIGHT_RPM      50   /* 纵移：向前或向后。原 100 改为 50 */
#define CHASSIS_SIDEWAYS_RPM      50   /* 侧移：向左或向右，包含定距左移。 */
#define CHASSIS_EXIT_DISTANCE_MM 700.0f /* 12 段左移距离（mm） */

/* 塔台捡拾控制参数 */
#define TOWER_ARM_WORK_PULSES      (-1600)  /* 电机 6 作业点，距上电零点伸出 1600 脉冲 */
#define TOWER_ARM_EXTENDED_PULSES  (-3200)  /* 电机 6 最大伸出 3200 脉冲，驱动坐标为 -3200，原点为 0 */
#define TOWER_CORR_DEADBAND    10     /* 纠偏死区：|误差| <= 10 像素视为已对准 */
#define TOWER_CORR_IN_POS_CNT  3      /* 连续 N 帧在死区内才算对准 */
#define TOWER_CORR_K_Y         1.0f   /* 转台(电机5)纠 Y 的比例增益 */
#define TOWER_CORR_K_X         1.0f   /* 作业臂(电机6)纠 X 的比例增益 */
#define TOWER_CORR_MAX_PULSES  800    /* 单次纠偏最大脉冲（限幅，防冲过头） */
#define TOWER_CORR_MAX_MOVES   30     /* 每轴最多发送的纠偏动作数 */
#define TOWER_CORR_TIMEOUT_MS 6000     /* 单轴纠偏时间上限 */
#define TOWER_COORD_TIMEOUT_MS 500    /* 视觉断流时结束本轮纠偏 */
#define TOWER_RETURN_STOP_MARGIN_MS 100U  /* 转台回零估时外的余量（ms），之后主动停车 */
#define TOWER_DROP_POSITION   (-3600) /* 放蛋位置，相对转台零点的脉冲数 */
#define TOWER_CLAW_MOVE_MS      500U   /* 开爪和闭爪指令中的 T0500，单位 ms */
#define TOWER_LIFT_PULSES_PER_MM  164.95f   /* 实测：3200 脉冲下降 19.4mm */
#define TOWER_LIFT_DOWN_MM        112.0f     /* 下降高度(mm)，自由改这里 */
#define TOWER_LIFT_DOWN_PULSES    ((int32_t)(TOWER_LIFT_DOWN_MM * TOWER_LIFT_PULSES_PER_MM))
#define TOWER_LIFT_TOP_SETTLE_MS  1000U     /* 提升后到转台动作的最短等待 */

/* 旧塔台流程参数（保留旧代码使用） */
#define TOWER_CORR_MAX_ITER     30     /* 单轴纠偏最大循环次数 */
#define TOWER_CORR_DELAY_MS     150    /* 每次纠偏后等待电机+视觉更新的时间 */
#define TOWER_TURNTABLE_MS(p)   ((uint32_t)((uint32_t)(p) * 568UL / 1000UL))  /* 转台(33RPM)移动毫秒 */
#define TOWER_LIFT_TRAVEL_MS    3000   /* 升降到位等待 */
#define TOWER_ABORT_COOLDOWN_MS 2000U  /* 取消夹取后给底盘 2000 ms 离开同一枚蛋 */
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

/* 首次就位或视觉触发时，底盘停车后置 g_tower_working，塔台作业完成后交回 UART5。 */
volatile bool     g_tower_working = false;
volatile bool     g_tower_ready = false;   /* 首次就位完成且可响应新坐标。 */
static volatile bool s_tower_initialized = false;  /* 电机 5、6、7 的上电零点均已设置。 */
/* 塔台命令发送失败后锁定作业，TOWER_MOTOR_NUM 表示无故障。 */
static volatile TowerMotorID s_tower_fault = TOWER_MOTOR_NUM;
/* 显示本轮正在执行的阶段，故障时保留最后状态以便区分视觉、升降和夹爪问题。 */
static char *volatile s_tower_status = "Tower idle";
static volatile bool s_route_active;

/* 底盘状态机当前状态号（Chassis_Task 写，Data_Task 只读显示） */
volatile int32_t  g_state = 0;

/* 航向角闭环目标(°)：按钮按下时锁存当前陀螺仪读数。
 * 上电后要等 Jetson 开机才发车，等待期间陀螺仪已累计漂移，0° 已不可用。 */
volatile float g_yaw_target_deg = 0.0f;

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
static bool Tower_SendAbsolute(TowerMotorID id, int32_t target);
static bool Tower_MoveArm(int32_t *position, int32_t target);
static bool Tower_ReturnTurntable(int32_t position);
static bool Tower_OpenClawAtDrop(void);
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
  /* 单独调试底盘时不创建塔台任务，避免塔台动作和接管 UART5。 */
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

  /* 底盘调试启用 PC1 启动和路线状态机，按键后等待 3 秒再发车。 */
#if 1
  /* 上电后等待按下 PC1 按键才开始行驶（PC1 另一端接 GND，低电平有效） */
  while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) != GPIO_PIN_RESET)
  {
    osDelay(20);
  }
  /* 按钮按下即锁存当前航向为闭环目标：上电等待 Jetson 开机期间陀螺仪已漂移，0° 已不可用 */
  {
    float yaw0;
    g_yaw_target_deg = ICM42688_GetYaw(&yaw0) ? yaw0 : 0.0f;
  }
  osDelay(3000);
  s_route_active = true;

  /* 各状态的停止条件：所有激光距离都在指定区间内才切换 */
  static const ZDT_AuxCond c_right_le100[1]               = { {LASER_RIGHT, 0, 110} };
  static const ZDT_AuxCond c_back_le80[1]                = { {LASER_BACK, 0, 80} };
  static const ZDT_AuxCond c_right_ge775_front_ge200[1]  = { {LASER_RIGHT, 775, 0}};
  static const ZDT_AuxCond c_front_le80_right_ge200[1]   = { {LASER_FRONT, 0, 100}};
  static const ZDT_AuxCond c_right_ge1455_back_ge200[1]  = { {LASER_RIGHT, 1455, 0}};
  static const ZDT_AuxCond c_back_le80_right_ge200[1]    = { {LASER_BACK, 0, 100}};
  static const ZDT_AuxCond c_right_ge2130_front_ge200[1] = { {LASER_RIGHT, 2130, 0}};
  static const ZDT_AuxCond c_right_ge2900_back_ge200[1]  = { {LASER_RIGHT, 2900, 0}};
  static const ZDT_AuxCond c_right_ge3540_front_ge200[1] = { {LASER_RIGHT, 3540, 0}};

  for (;;)
  {
    bool moved;
    /* ZDT_Chassis_Drive 的最后两项是外环纠偏激光和目标墙距（mm）。 */
    switch (g_state)
    {
      case 0:   /* 向右平移，右侧激光纠偏(目标 100mm)，右侧距离不大于 110mm 时停车。 */
        moved = ZDT_Chassis_Drive_Monitored(MOVE_RIGHT, CHASSIS_SIDEWAYS_RPM, c_right_le100, 1, LASER_FRONT, 40);
        break;
      case 1:   /* 向后直行，后侧距离不大于 80 mm 时直接停车。 */
        moved = ZDT_Chassis_Drive_Monitored(MOVE_BACKWARD, CHASSIS_STRAIGHT_RPM, c_back_le80, 1, LASER_RIGHT, 80);
        break;
      case 2:   /* 向左，直到 right>775 且 front>200 */
        moved = ZDT_Chassis_Drive(MOVE_LEFT, CHASSIS_SIDEWAYS_RPM, c_right_ge775_front_ge200, 1, LASER_BACK, 55);
        break;
      case 3:   /* 首次把电机 6 移到作业点，再开始本段行驶和视觉触发。 */
        if (s_tower_initialized) ZDT_Chassis_ParkForTower();
        moved = ZDT_Chassis_Drive(MOVE_FORWARD, CHASSIS_STRAIGHT_RPM, c_front_le80_right_ge200, 1, LASER_RIGHT, 50);
        break;
      case 4:   /* 向左，直到 right>1455 且 back>200 */
        moved = ZDT_Chassis_Drive(MOVE_LEFT, CHASSIS_SIDEWAYS_RPM, c_right_ge1455_back_ge200, 1, LASER_FRONT, 40);
        break;
      case 5:   /* 向后，直到 back<80 且 right>200 */
        moved = ZDT_Chassis_Drive(MOVE_BACKWARD, CHASSIS_STRAIGHT_RPM, c_back_le80_right_ge200, 1 , LASER_RIGHT, 50);
        break;
      case 6:   /* 向左，直到 right>2130 且 front>200 */
        moved = ZDT_Chassis_Drive(MOVE_LEFT, CHASSIS_SIDEWAYS_RPM, c_right_ge2130_front_ge200, 1, LASER_BACK, 55);
        break;
      case 7:   /* 向前，直到 front<80 且 right>200 */
        moved = ZDT_Chassis_Drive(MOVE_FORWARD, CHASSIS_STRAIGHT_RPM, c_front_le80_right_ge200, 1, LASER_RIGHT, 50);
        break;
      case 8:   /* 向左，直到 right>2900 且 back>200 */
        moved = ZDT_Chassis_Drive(MOVE_LEFT, CHASSIS_SIDEWAYS_RPM, c_right_ge2900_back_ge200, 1, LASER_FRONT, 40);
        break;
      case 9:   /* 向后，直到 back<80 且 right>200 */
        moved = ZDT_Chassis_Drive(MOVE_BACKWARD, CHASSIS_STRAIGHT_RPM, c_back_le80_right_ge200, 1 , LASER_RIGHT, 50);
        break;
      case 10:  /* 向左，直到 right>3540 且 front>200 */
        moved = ZDT_Chassis_Drive(MOVE_LEFT, CHASSIS_SIDEWAYS_RPM, c_right_ge3540_front_ge200, 1, LASER_BACK, 55);
        break;
      case 11:  /* 向前，直到 front<80 且 right>200 */
        moved = ZDT_Chassis_Drive(MOVE_FORWARD, CHASSIS_STRAIGHT_RPM, c_front_le80_right_ge200, 1, LASER_RIGHT, 50);
        break;
      case 12:  /* 按定距左移（mm），识别到鸡蛋停车，作业后续走估算的剩余距离。 */
        moved = ZDT_Chassis_Move_Direct(MOVE_LEFT, CHASSIS_EXIT_DISTANCE_MM, CHASSIS_SIDEWAYS_RPM);
        break;

      default:  /* 到达路线终点后停车待机。 */
        s_route_active = false;
        ZDT_Chassis_Stop();
        osDelay(1000);
        continue;
    }

#if ZDT_CHASSIS_FAULT_STOP_ENABLED
    /* 开启故障停车时，运动失败进入停车待机状态。 */
    if (!moved)
    {
      g_state = -1;
      continue;
    }
#else
    /* 调试时屏蔽故障停车，运动函数返回后按路线等待并进入下一段。 */
    (void)moved;
#endif

    /* 停下后等待 500ms */
    osDelay(500);
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
    /* 串口故障显示电机地址，其他情况显示塔台作业阶段或开爪回读结果。 */
    static char *const tower_fault_text[] = {"Motor 5 TX ERR", "Motor 6 TX ERR", "Motor 7 TX ERR"};
    TowerMotorID tower_fault = s_tower_fault;
    char tower_text[21];
    char *tower_status = tower_fault < TOWER_MOTOR_NUM ? tower_fault_text[tower_fault] : s_tower_status;
    snprintf(tower_text, sizeof(tower_text), "%-20s", tower_status);
    LCD_DisplayString(10, 30, tower_text);
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
  int32_t off_6 = 0;   /* 电机 6 相对上电零点的累计脉冲偏移（电机 7 已改绝对定位，无需软件跟踪） */
  TowerMotorID fault_motor;

  /* 上电使能三轴并记录当前位置为零点，夹爪由自身上电逻辑保持打开。 */
  TowerMotor_Init();
  /* 三轴都需要有效的上电坐标，任一使能或清零命令发送失败就停止塔台任务。 */
  for (TowerMotorID id = TOWER_TURNTABLE; id < TOWER_MOTOR_NUM; id++)
  {
    if (!TowerMotor_Enable(id, true))
    {
      s_tower_fault = id;
      osThreadExit();
    }
  }
  osDelay(500);   /* 等电机使能后稳定，再设零点，避免零点漂移导致回零过冲 */
  for (TowerMotorID id = TOWER_TURNTABLE; id < TOWER_MOTOR_NUM; id++)
  {
    if (!TowerMotor_ZeroPosition(id))
    {
      s_tower_fault = id;
      osThreadExit();
    }
  }
  /* 首次 UART5 交接发生在 state 3。只伸出电机 6，完成后交还底盘并开放视觉触发。 */
  s_tower_initialized = true;
  while (!g_tower_working) osDelay(5);
  fault_motor = TOWER_ARM;
  if (!Tower_MoveArm(&off_6, TOWER_ARM_WORK_PULSES)) goto tower_fault;
  g_tower_working = false;

  for (;;)
  {
    /* 底盘识别到 Y 在目标 ±200 像素内的鸡蛋后先停车，再交接 UART5 开始纠偏。 */
    g_tower_ready = true;
    while (!g_tower_working)
    {
      osDelay(5);
    }
    g_tower_ready = false;

    /* 电机 5 逐次走完 Y 纠偏脉冲，累计位移用于估算转台后续行程。 */
    s_tower_status = "Align Y";
    fault_motor = TOWER_TURNTABLE;
    int32_t off_5 = 0;
    int in_pos_y = 0;
    bool y_aligned = false;
    uint32_t frame = Coord_GetFrameCount();
    CoordPoint point;
    for (int i = 0; i < TOWER_CORR_MAX_ITER; i++)
    {
      if (!Tower_WaitFreshPoint(&frame, &point)) break;   /* 视觉断流，结束纠偏 */

      err_y = point.y - TOWER_Y_TARGET;
      if (err_y >= -TOWER_CORR_DEADBAND && err_y <= TOWER_CORR_DEADBAND)
      {
        if (++in_pos_y >= TOWER_CORR_IN_POS_CNT)
        {
          y_aligned = true;
          break;
        }
        continue;
      }
      in_pos_y = 0;

      int32_t pulses = (int32_t)((float)err_y * TOWER_CORR_K_Y);
      if (pulses >  TOWER_CORR_MAX_PULSES) pulses =  TOWER_CORR_MAX_PULSES;
      if (pulses < -TOWER_CORR_MAX_PULSES) pulses = -TOWER_CORR_MAX_PULSES;
      pulses = -pulses;   /* 电机5 纠偏方向取反 */

      if (!TowerMotor_Move(TOWER_TURNTABLE, pulses)) goto tower_fault;
      off_5 += pulses;
      osDelay(TowerMotor_TravelTimeMs(TOWER_TURNTABLE, pulses) + TOWER_ACTION_GAP_MS);
      frame = Coord_GetFrameCount();
    }

    /* 停住电机 5 并等 500 ms，之后由已在作业点的电机 6 纠 X。 */
    if (!TowerMotor_Stop(TOWER_TURNTABLE)) goto tower_fault;
    osDelay(TOWER_ACTION_GAP_MS);
    fault_motor = TOWER_ARM;
    /* 电机 6 目标限幅到 [-3200, 0] 脉冲，正常夹取要求 X、Y 连续 3 帧都在 ±10 像素内。 */
    s_tower_status = "Align X";
    int in_pos_x = 0;
    int moves_x = 0;
    bool y_in_x_window = true;
    bool grab_requested = false;
    frame = Coord_GetFrameCount();
    while (y_aligned && Tower_WaitFreshPoint(&frame, &point))
    {
      err_x = point.x - TOWER_X_TARGET;
      err_y = point.y - TOWER_Y_TARGET;
      bool y_in_range = err_y >= -TOWER_CORR_DEADBAND && err_y <= TOWER_CORR_DEADBAND;
      if (err_x >= -TOWER_CORR_DEADBAND && err_x <= TOWER_CORR_DEADBAND)
      {
        y_in_x_window = y_in_x_window && y_in_range;
        if (++in_pos_x >= TOWER_CORR_IN_POS_CNT)
        {
          grab_requested = y_in_x_window;
          break;
        }
        continue;
      }
      in_pos_x = 0;
      y_in_x_window = true;

      /* 到 -3200 脉冲后，用到位后新帧确认 Y 仍锁定。X 有偏差时保持限位并尝试夹取一次。 */
      if (off_6 == TOWER_ARM_EXTENDED_PULSES)
      {
        grab_requested = y_in_range;
        break;
      }
      if (moves_x >= TOWER_CORR_MAX_ITER) break;

      int32_t pulses = (int32_t)((float)err_x * TOWER_CORR_K_X);
      if (pulses >  TOWER_CORR_MAX_PULSES) pulses =  TOWER_CORR_MAX_PULSES;
      if (pulses < -TOWER_CORR_MAX_PULSES) pulses = -TOWER_CORR_MAX_PULSES;

      int32_t target = off_6 + pulses;
      if (target < TOWER_ARM_EXTENDED_PULSES) target = TOWER_ARM_EXTENDED_PULSES;
      if (target > 0) target = 0;
      if (target == off_6) break;   /* 到原点且纠偏还要求继续回缩。 */

      if (!Tower_MoveArm(&off_6, target)) goto tower_fault;
      moves_x++;
      frame = Coord_GetFrameCount();
    }

    if (!grab_requested)
    {
      /* 未满足正常或限位夹取条件时取消。电机 7 和 5 回零，电机 6 回 -1600 脉冲。 */
      s_tower_status = "Grab skipped X/Y";
      fault_motor = TOWER_LIFT;
      if (!Tower_SendAbsolute(TOWER_LIFT, 0)) goto tower_fault;
      osDelay(TOWER_ACTION_GAP_MS);   /* 本轮未下降，升降轴已在零点，等 500 ms 再移动作业臂。 */

      fault_motor = TOWER_ARM;
      if (!Tower_MoveArm(&off_6, TOWER_ARM_WORK_PULSES)) goto tower_fault;

      fault_motor = TOWER_TURNTABLE;
      if (!Tower_ReturnTurntable(off_5)) goto tower_fault;
      goto finish_tower;
    }

    /* 电机 7 下降到设定高度，运动估时结束后再等 500 ms 闭爪。 */
    s_tower_status = off_6 == TOWER_ARM_EXTENDED_PULSES ? "Lower at X limit" : "Lift down";
    fault_motor = TOWER_LIFT;
    if (!Tower_SendAbsolute(TOWER_LIFT, -TOWER_LIFT_DOWN_PULSES)) goto tower_fault;
    osDelay(TowerMotor_TravelTimeMs(TOWER_LIFT, TOWER_LIFT_DOWN_PULSES) + TOWER_ACTION_GAP_MS);
    if (!Claw_Close())
    {
      s_tower_status = "Close TX ERR";
      goto claw_fault;
    }
    s_tower_status = "Close sent";
    osDelay(TOWER_CLAW_MOVE_MS + TOWER_ACTION_GAP_MS);

    /* 先抬升电机 7 回上电零点（0 脉冲），再收回电机 6。 */
    s_tower_status = "Lift up";
    if (!Tower_SendAbsolute(TOWER_LIFT, 0)) goto tower_fault;
    osDelay(TowerMotor_TravelTimeMs(TOWER_LIFT, TOWER_LIFT_DOWN_PULSES) + TOWER_ACTION_GAP_MS);

    /* 5.5 电机 7 抬升后，将作业臂收回到上电零点（0 脉冲），再转动塔台。 */
    s_tower_status = "Arm home";
    fault_motor = TOWER_ARM;
    if (!Tower_MoveArm(&off_6, 0)) goto tower_fault;

    /* 转台移到固定放蛋点（绝对脉冲），暂时发不出时重试同一目标。 */
    s_tower_status = "Moving to drop";
    fault_motor = TOWER_TURNTABLE;
    {
      int32_t dist = TOWER_DROP_POSITION - off_5;
      if (!Tower_SendAbsolute(TOWER_TURNTABLE, TOWER_DROP_POSITION)) goto tower_fault;
      osDelay(TowerMotor_TravelTimeMs(TOWER_TURNTABLE, dist) + TOWER_ACTION_GAP_MS);
    }

    /* 到固定放蛋点才发开爪指令，并在原地检查和重试。 */
    if (!Tower_OpenClawAtDrop()) goto claw_fault;

    /* 转台先回到上电零点，随后电机 6 恢复到 -1600 脉冲的作业点。 */
    if (!Tower_ReturnTurntable(TOWER_DROP_POSITION)) goto tower_fault;
    fault_motor = TOWER_ARM;
    if (!Tower_MoveArm(&off_6, TOWER_ARM_WORK_PULSES)) goto tower_fault;

finish_tower:
    /* 9. 成功放蛋或取消夹取后，等复位结束再清坐标并恢复底盘。 */
    taskENTER_CRITICAL();
    Coord_ClearPoint();
    g_coord_x = 0;
    g_coord_y = 0;
    g_coord_valid = false;
    taskEXIT_CRITICAL();

    /* 复位或就位后恢复底盘。取消夹取时留出行驶时间，避免马上再次触发同一枚蛋。 */
    g_tower_working = false;
    if (!grab_requested) osDelay(TOWER_ABORT_COOLDOWN_MS);
  }

tower_fault:
  /* 保持底盘停车请求，只重试停车命令。串口恢复后也结束塔台任务，防止从未知位置继续。 */
  s_tower_fault = fault_motor;
  while (!TowerMotor_Stop(fault_motor)) osDelay(50);
  osThreadExit();

claw_fault:
  /* 开闭发送失败或开爪回读确认不到位时保留底盘停车和屏幕提示。 */
  osThreadExit();
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

static bool Tower_SendAbsolute(TowerMotorID id, int32_t target)
{
  /* 绝对目标以脉冲为单位。短暂串口故障后仍发同一目标，最多 3 次，间隔 50 ms。 */
  for (int attempt = 0; attempt < 3; attempt++)
  {
    if (TowerMotor_MoveAbsolute(id, target)) return true;
    if (attempt < 2) osDelay(50);
  }
  return false;
}

static bool Tower_MoveArm(int32_t *position, int32_t target)
{
  if (target == *position) return true;

  /* 电机 6 发出绝对位置后，按脉冲差等待预计运动时间及 500 ms 间隔，再更新位置。 */
  if (!Tower_SendAbsolute(TOWER_ARM, target)) return false;
  osDelay(TowerMotor_TravelTimeMs(TOWER_ARM, target - *position) + TOWER_ACTION_GAP_MS);
  *position = target;
  return true;
}

static bool Tower_ReturnTurntable(int32_t position)
{
  if (position == 0) return true;

  /* 绝对模式回零：目标是驱动坐标 0。绝对指令幂等，串口超时后即使重发也只回 0，
   * 不会像相对位移那样再走一段（之前转台过转的根因）。发送失败由上层停车，估时后主动停车兜底。 */
  if (!Tower_SendAbsolute(TOWER_TURNTABLE, 0)) return false;
  osDelay(TowerMotor_TravelTimeMs(TOWER_TURNTABLE, position) + TOWER_RETURN_STOP_MARGIN_MS);
  if (!TowerMotor_Stop(TOWER_TURNTABLE)) return false;
  osDelay(TOWER_ACTION_GAP_MS);
  return true;
}

static bool Tower_OpenClawAtDrop(void)
{
  bool position_mismatch = false;
  for (int attempt = 0; attempt < 2; attempt++)
  {
    if (!Claw_Open())
    {
      s_tower_status = "Open TX ERR";
      return false;
    }
    s_tower_status = attempt == 0 ? "Open sent" : "Open retry";
    /* 给舵机 500 ms 完成打开，再等 500 ms 后查询。未回读或偏差超限时只重发一次。 */
    osDelay(TOWER_CLAW_MOVE_MS + TOWER_ACTION_GAP_MS);
    ClawOpenStatus status = Claw_CheckOpen();
    if (status == CLAW_OPEN_REACHED)
    {
      s_tower_status = "Open verified";
      return true;
    }
    if (status == CLAW_OPEN_MISMATCH) position_mismatch = true;
  }
  /* 兼容只有发送线的夹爪。收到实际位置且仍有偏差时停在放蛋点。 */
  s_tower_status = position_mismatch ? "Open not reached" : "Open no reply";
  return !position_mismatch;
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

