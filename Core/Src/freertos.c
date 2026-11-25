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
#include "pid.h"
#include "tower_motor.h"
#include "jetson_coord.h"
#include "claw.h"
#include "lcd_spi_154.h"
#include "lcd_fonts.h"
#include <math.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  ZDT_MoveDir direction;
  uint16_t cruise_rpm;
  ZDT_AuxCond stop;
  ZDT_AuxCond clearance;
  uint16_t position_target_mm;
} ChassisLeg;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define START_BUTTON_ACTIVE      GPIO_PIN_RESET /* PC1 按钮另一端接 GND */
#define START_BUTTON_DEBOUNCE_MS 30U
#define START_DELAY_MS           3000U
#define CHASSIS_FAST_RPM         200U
#define CHASSIS_WORK_RPM         75U
#define CHASSIS_BRAKE_LEAD_MM    250U
#define CHASSIS_BRAKE_RPM_PER_S  400.0f
#define CHASSIS_YAW_LOOP_MS      20U
#define CHASSIS_POS_LOOP_MS      40U
#define CHASSIS_YAW_KP           0.8f
#define CHASSIS_YAW_KI           0.0f
#define CHASSIS_YAW_KD           0.06f
#define CHASSIS_YAW_MAX_RPM      10.0f
#define CHASSIS_YAW_DEADBAND_DEG 0.8f
#define CHASSIS_POS_MAX_RPM      20.0f
#define CHASSIS_POS_DEADBAND_MM  4.0f
#define CHASSIS_POS_MAX_ERROR_MM 200.0f
#define IMU_LOOP_MS              10U

/* 塔台捡拾控制参数 */
#define TOWER_TRIGGER_Y_ERR    50     /* |err_y| <= 50 触发塔台作业 */
#define TOWER_CORR_DEADBAND    10     /* 纠偏死区：|误差| <= 10 像素视为已对准 */
#define TOWER_CORR_IN_POS_CNT  8      /* 连续 N 次在死区内才算对准（防抖动） */
#define TOWER_CORR_K_Y         1.0f   /* 转台(电机5)纠 Y 的比例增益 */
#define TOWER_CORR_K_X         0.5f   /* 作业臂(电机6)纠 X 的比例增益 */
#define TOWER_CORR_MAX_PULSES  800    /* 单次纠偏最大脉冲（限幅，防冲过头） */
#define TOWER_CORR_MAX_ITER    30     /* 单轴纠偏最大次数（防停不下来） */
#define TOWER_CORR_DELAY_MS    150    /* 每次纠偏后等待电机+视觉更新的时间 */
#define TOWER_LIFT_PULSES_PER_MM  164.95f   /* 实测：3200 脉冲下降 19.4mm */
#define TOWER_LIFT_DOWN_MM        79.0f     /* 下降高度(mm)，自由改这里 */
#define TOWER_LIFT_DOWN_PULSES    ((int32_t)(TOWER_LIFT_DOWN_MM * TOWER_LIFT_PULSES_PER_MM))
#define TOWER_LIFT_WAIT_MS     3000   /* 下降后停留 3s */
#define TOWER_POST_DELAY_MS    2000   /* 作业完成后恢复底盘前的等待 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* 前后的停车主条件用各自的激光，侧移统一看右激光。 */
static const ChassisLeg s_route[] = {
  /* case 0–1：快速到达第一条垄，提前减到作业速度。 */
  {MOVE_RIGHT,    CHASSIS_FAST_RPM, {LASER_RIGHT,    0, 80}, {LASER_BACK,  200, 0}, ZDT_LON_TARGET_MM},
  {MOVE_BACKWARD, CHASSIS_FAST_RPM, {LASER_BACK,     0, 80}, {0},                  ZDT_LAT_TARGET_MM},
  /* case 2–11：逐垄往返。沿垄行驶时保持对应垄的右侧激光距离。 */
  {MOVE_LEFT,     CHASSIS_WORK_RPM, {LASER_RIGHT,  775,  0}, {LASER_FRONT, 200, 0}, 0},
  {MOVE_FORWARD,  CHASSIS_WORK_RPM, {LASER_FRONT,    0, 80}, {LASER_RIGHT, 200, 0}, 775},
  {MOVE_LEFT,     CHASSIS_WORK_RPM, {LASER_RIGHT, 1455,  0}, {LASER_BACK,  200, 0}, 0},
  {MOVE_BACKWARD, CHASSIS_WORK_RPM, {LASER_BACK,     0, 80}, {LASER_RIGHT, 200, 0}, 1455},
  {MOVE_LEFT,     CHASSIS_WORK_RPM, {LASER_RIGHT, 2130,  0}, {LASER_FRONT, 200, 0}, 0},
  {MOVE_FORWARD,  CHASSIS_WORK_RPM, {LASER_FRONT,    0, 80}, {LASER_RIGHT, 200, 0}, 2130},
  {MOVE_LEFT,     CHASSIS_WORK_RPM, {LASER_RIGHT, 2900,  0}, {LASER_BACK,  200, 0}, 0},
  {MOVE_BACKWARD, CHASSIS_WORK_RPM, {LASER_BACK,     0, 80}, {LASER_RIGHT, 200, 0}, 2900},
  {MOVE_LEFT,     CHASSIS_WORK_RPM, {LASER_RIGHT, 3540,  0}, {LASER_FRONT, 200, 0}, 0},
  {MOVE_FORWARD,  CHASSIS_WORK_RPM, {LASER_FRONT,    0, 80}, {LASER_RIGHT, 200, 0}, 3540},
};

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

/* 上位机 Jetson 传来的物块坐标（Data_Task 刷新，其他任务只读） */
volatile int32_t  g_coord_x;
volatile int32_t  g_coord_y;
volatile bool     g_coord_valid;

/* 两个任务通过请求和停车确认交接 UART5。 */
volatile bool     g_tower_working = false;
static volatile bool s_chassis_parked;
static volatile bool s_route_active;

/* 底盘状态机当前状态号（Chassis_Task 写，Data_Task 只读显示） */
volatile int32_t  g_state = 0;

/* USER CODE END Variables */
/* Definitions for Chassis_Task */
osThreadId_t Chassis_TaskHandle;
const osThreadAttr_t Chassis_Task_attributes = {
  .name = "Chassis_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
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
static void chassis_wait_start_button(void);
static bool chassis_drive_leg(const ChassisLeg *leg);
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
  float yaw;
  ZDT_Motor_Init();
  Laser_Init();
  chassis_wait_start_button();
  osDelay(START_DELAY_MS);
  while (!ICM42688_GetYaw(&yaw)) osDelay(IMU_LOOP_MS);

  s_route_active = true;
  for (g_state = 0; g_state < (int32_t)(sizeof(s_route) / sizeof(s_route[0])); g_state++)
  {
    if (!chassis_drive_leg(&s_route[g_state]))
    {
      g_state = -1;
      break;
    }
    osDelay(700);
  }

  s_route_active = false;
  for (;;)
  {
    ZDT_Chassis_Stop();
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
    /* 1. 等触发：收到有效坐标，且 |err_y| <= 50 */
    for (;;)
    {
      if (s_route_active && g_state >= 2 && g_coord_valid)
      {
        err_y = g_coord_y - TOWER_Y_TARGET;
        if (err_y > -TOWER_TRIGGER_Y_ERR && err_y < TOWER_TRIGGER_Y_ERR)
        {
          break;
        }
      }
      osDelay(20);
    }

    /* 先让底盘停车并交出 UART5，再发塔台动作指令。 */
    g_tower_working = true;
    while (s_route_active && !s_chassis_parked) osDelay(5);
    if (!s_route_active)
    {
      g_tower_working = false;
      continue;
    }
    osDelay(100);

    if (!initialized)
    {
      TowerMotor_Init();
      TowerMotor_Enable(TOWER_TURNTABLE, true);
      TowerMotor_Enable(TOWER_ARM, true);
      TowerMotor_Enable(TOWER_LIFT, true);
      Claw_Open();
      osDelay(500);
      initialized = true;
    }

    /* 2. 电机 5（转台）：纠偏 Y，记录偏移 */
    int in_pos_y = 0;
    for (int i = 0; i < TOWER_CORR_MAX_ITER; i++)
    {
      err_y = g_coord_y - TOWER_Y_TARGET;
      if (err_y > -TOWER_CORR_DEADBAND && err_y < TOWER_CORR_DEADBAND)
      {
        if (++in_pos_y >= TOWER_CORR_IN_POS_CNT) break;
        osDelay(TOWER_CORR_DELAY_MS);
        continue;
      }
      in_pos_y = 0;

      int32_t pulses = (int32_t)((float)err_y * TOWER_CORR_K_Y);
      if (pulses >  TOWER_CORR_MAX_PULSES) pulses =  TOWER_CORR_MAX_PULSES;
      if (pulses < -TOWER_CORR_MAX_PULSES) pulses = -TOWER_CORR_MAX_PULSES;
      pulses = -pulses;   /* 电机5 纠偏方向取反 */

      TowerMotor_Move(TOWER_TURNTABLE, pulses);
      off_5 += pulses;
      osDelay(TOWER_CORR_DELAY_MS);
    }

    /* 3. 电机 6（作业臂）：纠偏 X，记录偏移 */
    int in_pos_x = 0;
    for (int i = 0; i < TOWER_CORR_MAX_ITER; i++)
    {
      err_x = g_coord_x - TOWER_X_TARGET;
      if (err_x > -TOWER_CORR_DEADBAND && err_x < TOWER_CORR_DEADBAND)
      {
        if (++in_pos_x >= TOWER_CORR_IN_POS_CNT) break;
        osDelay(TOWER_CORR_DELAY_MS);
        continue;
      }
      in_pos_x = 0;

      int32_t pulses = (int32_t)((float)err_x * TOWER_CORR_K_X);
      if (pulses >  TOWER_CORR_MAX_PULSES) pulses =  TOWER_CORR_MAX_PULSES;
      if (pulses < -TOWER_CORR_MAX_PULSES) pulses = -TOWER_CORR_MAX_PULSES;

      TowerMotor_Move(TOWER_ARM, pulses);
      off_6 += pulses;
      osDelay(TOWER_CORR_DELAY_MS);
    }

    /* 4. 电机 7 下降 79mm -> 停1s -> 夹爪闭合 -> 等1s */
    TowerMotor_Move(TOWER_LIFT, -TOWER_LIFT_DOWN_PULSES);
    off_7 -= TOWER_LIFT_DOWN_PULSES;
    osDelay(1500);          /* 等下降到位 */
    osDelay(1000);          /* 停 1s */
    Claw_Close();
    osDelay(1000);          /* 等 1s */

    /* 5. 电机 7 复位 -> 复位后等 1s */
    TowerMotor_Move(TOWER_LIFT, -off_7);
    off_7 = 0;
    osDelay(1500);          /* 等电机7复位到位 */
    osDelay(1000);          /* 复位后等 1s */

    /* 6. 电机 5 移到「初始位置向左 3700 脉冲」处 */
    TowerMotor_Move(TOWER_TURNTABLE, -3700 - off_5);
    off_5 = -3700;
    osDelay(2200);          /* 等电机5 到位（3700 脉冲 @33RPM ≈ 2.1s） */
    osDelay(1000);          /* 停 1s */

    /* 7. 夹爪打开（放蛋） */
    Claw_Open();
    osDelay(500);

    /* 8. 所有电机回到初始位置 */
    TowerMotor_Move(TOWER_TURNTABLE, -off_5);
    TowerMotor_Move(TOWER_ARM, -off_6);
    TowerMotor_Move(TOWER_LIFT, -off_7);
    off_5 = off_6 = off_7 = 0;
    osDelay(3000);

    /* 9. 恢复底盘，等下一次触发 */
    g_tower_working = false;
    osDelay(TOWER_POST_DELAY_MS);
  }
  /* USER CODE END StartTask03 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
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

    /* 行驶中失联时先等底盘退出路线，停车后再重建航向零点。 */
    while (s_route_active) osDelay(IMU_LOOP_MS);
    osDelay(250);
  }
}

static void chassis_wait_start_button(void)
{
  /* 先确认按钮已释放，随后等待稳定的按下沿。 */
  uint32_t released_since = osKernelGetTickCount();
  do
  {
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == START_BUTTON_ACTIVE)
      released_since = osKernelGetTickCount();
    osDelay(5);
  } while (osKernelGetTickCount() - released_since < START_BUTTON_DEBOUNCE_MS);

  for (;;)
  {
    while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) != START_BUTTON_ACTIVE) osDelay(5);
    uint32_t pressed_since = osKernelGetTickCount();
    do
    {
      osDelay(5);
      if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) != START_BUTTON_ACTIVE) break;
      if (osKernelGetTickCount() - pressed_since >= START_BUTTON_DEBOUNCE_MS) return;
    } while (true);
  }
}

static bool chassis_clearance_met(const ZDT_AuxCond *clearance)
{
  uint16_t distance;
  if (clearance->min_mm == 0 && clearance->max_mm == 0) return true;

  return Laser_GetFreshDistance(clearance->laser, &distance) &&
         distance >= clearance->min_mm &&
         (clearance->max_mm == 0 || distance <= clearance->max_mm);
}

static float chassis_position_rpm(const ChassisLeg *leg, bool longitudinal, PID_t *pid, float dt)
{
  uint16_t distance;
  LaserID laser = longitudinal ? LASER_RIGHT : LASER_FRONT;
  if (leg->position_target_mm != 0 && Laser_GetFreshDistance(laser, &distance))
  {
    /* 正误差表示离目标太远。向前或向右移动会缩短对应激光距离。 */
    float error = (float)distance - leg->position_target_mm;
    if (fabsf(error) > CHASSIS_POS_DEADBAND_MM && fabsf(error) <= CHASSIS_POS_MAX_ERROR_MM)
    {
      float rpm = PID_Compute_Error(pid, error, dt);
      bool invert = longitudinal ? ZDT_LAT_DIR_INVERT : ZDT_LON_DIR_INVERT;
      return invert ? -rpm : rpm;
    }
  }

  PID_Reset(pid);
  return 0.0f;
}

static bool chassis_drive_leg(const ChassisLeg *leg)
{
  bool longitudinal = leg->direction == MOVE_FORWARD || leg->direction == MOVE_BACKWARD;
  float direction = (leg->direction == MOVE_FORWARD || leg->direction == MOVE_RIGHT) ? 1.0f : -1.0f;
  float base_rpm = leg->cruise_rpm;
  float position_rpm = 0.0f;
  bool started = false;
  bool braking = false;
  uint32_t next_tick = osKernelGetTickCount();
  uint32_t last_yaw_tick = next_tick - CHASSIS_YAW_LOOP_MS;
  uint32_t last_position_tick = next_tick - CHASSIS_POS_LOOP_MS;
  PID_t yaw_pid;
  PID_t position_pid;

  PID_Init(&yaw_pid, CHASSIS_YAW_KP, CHASSIS_YAW_KI, CHASSIS_YAW_KD,
           -CHASSIS_YAW_MAX_RPM, CHASSIS_YAW_MAX_RPM);
  PID_Init(&position_pid,
           longitudinal ? ZDT_LAT_KP : ZDT_LON_KP,
           longitudinal ? ZDT_LAT_KI : ZDT_LON_KI,
           longitudinal ? ZDT_LAT_KD : ZDT_LON_KD,
           -CHASSIS_POS_MAX_RPM, CHASSIS_POS_MAX_RPM);
  if (position_pid.ki > 0.0f)
  {
    float limit = CHASSIS_POS_MAX_RPM / position_pid.ki;
    PID_SetIntLimit(&position_pid, -limit, limit);
  }

  for (;;)
  {
    if (g_tower_working)
    {
      if (!ZDT_Chassis_Stop()) return false;
      s_chassis_parked = true;
      while (g_tower_working) osDelay(5);
      s_chassis_parked = false;
      PID_Reset(&yaw_pid);
      PID_Reset(&position_pid);
      started = false;
      next_tick = osKernelGetTickCount();
      last_yaw_tick = next_tick - CHASSIS_YAW_LOOP_MS;
      last_position_tick = next_tick - CHASSIS_POS_LOOP_MS;
    }

    float yaw;
    uint16_t distance;
    if (!ICM42688_GetYaw(&yaw) || !Laser_GetFreshDistance(leg->stop.laser, &distance)) return false;

    int remaining_mm = leg->stop.max_mm != 0 ? (int)distance - leg->stop.max_mm
                                               : (int)leg->stop.min_mm - distance;
    if (remaining_mm <= 0 && chassis_clearance_met(&leg->clearance)) return ZDT_Chassis_Stop();

    uint32_t now = osKernelGetTickCount();
    float yaw_dt = (float)(now - last_yaw_tick) * 0.001f;
    last_yaw_tick = now;

    if (leg->cruise_rpm > CHASSIS_WORK_RPM && remaining_mm <= (int)CHASSIS_BRAKE_LEAD_MM)
      braking = true;
    if (braking)
    {
      base_rpm = started ? fmaxf(CHASSIS_WORK_RPM, base_rpm - CHASSIS_BRAKE_RPM_PER_S * yaw_dt)
                         : CHASSIS_WORK_RPM;
    }

    if (now - last_position_tick >= CHASSIS_POS_LOOP_MS)
    {
      float dt = (float)(now - last_position_tick) * 0.001f;
      position_rpm = chassis_position_rpm(leg, longitudinal, &position_pid, dt);
      last_position_tick = now;
    }

    /* 所有路段只平移。取回到上电零向的最短角度，兼容跨 359°/0°。 */
    float yaw_error = yaw > 180.0f ? 360.0f - yaw : -yaw;
    float outside_deadband = fmaxf(0.0f, fabsf(yaw_error) - CHASSIS_YAW_DEADBAND_DEG);
    yaw_error = copysignf(outside_deadband, yaw_error);
    if (!started) yaw_pid.prev_error = yaw_error;
    float yaw_rpm = PID_Compute_Error(&yaw_pid, yaw_error, yaw_dt);

    float forward = longitudinal ? direction * base_rpm : position_rpm;
    float right = longitudinal ? position_rpm : direction * base_rpm;
    if (!ZDT_Chassis_SetVelocity(forward, right, yaw_rpm)) return false;
    started = true;

    next_tick += CHASSIS_YAW_LOOP_MS;
    if (osDelayUntil(next_tick) != osOK) next_tick = osKernelGetTickCount();
  }
}
/* USER CODE END Application */

