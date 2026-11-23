/**
  ******************************************************************************
  * @file    tower_motor.c
  * @brief   塔台三轴 ZDT X42S 闭环步进电机驱动（UART5 总线，只发不收）
  *
  *          协议与底盘 zdt_motor.c 一致，这里单独成文件，方便塔台独立维护。
  *          写命令为【只发不收】：DMA 发送、忽略应答。
  ******************************************************************************
  */

#include "tower_motor.h"
#include "pid.h"
#include "cmsis_os.h"

/* 校验字节 */
#define TOWER_CHECK_BYTE   0x6B

/* 每个电机的硬件映射 */
typedef struct
{
    UART_HandleTypeDef *huart;
    uint8_t             addr;    /* 从机地址（拨码设置） */
    uint8_t             invert;  /* 方向反相：1=反向；实测方向反了把 0 改 1 */
} TowerMotorDef;

static const TowerMotorDef s_tower[TOWER_MOTOR_NUM] =
{
    { &huart5, 5, 0 },   /* TOWER_TURNTABLE 转台 */
    { &huart5, 6, 0 },   /* TOWER_ARM 作业臂前后 */
    { &huart5, 7, 0 },   /* TOWER_LIFT 升降 */
};

/* ============================================================================
 *                            底层收发
 * ========================================================================== */

/** 丢弃串口接收缓冲区里残留的字节，并清溢出标志。 */
static void tower_flush_rx(UART_HandleTypeDef *huart)
{
    while (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE) != RESET)
    {
        (void)(uint8_t)(huart->Instance->DR & 0xFFU);
    }
    __HAL_UART_CLEAR_OREFLAG(huart);
}

/** 只发不收：DMA 发送一帧，忽略电机应答。发送前清残留与溢出标志。
 *  命令帧 buffer 位于上层函数栈上，必须等 DMA 发送完成才能返回。 */
static bool tower_send_frame(UART_HandleTypeDef *huart, const uint8_t *cmd, uint16_t len)
{
    uint32_t start;

    __HAL_UART_CLEAR_OREFLAG(huart);
    while (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE) != RESET)
    {
        (void)(uint8_t)(huart->Instance->DR & 0xFFU);
    }

    if (HAL_UART_Transmit_DMA(huart, (uint8_t *)cmd, len) != HAL_OK)
    {
        return false;
    }

    start = HAL_GetTick();
    while (HAL_UART_GetState(huart) != HAL_UART_STATE_READY)
    {
        if ((HAL_GetTick() - start) > TOWER_UART_TIMEOUT_MS)
        {
            return false;   /* 超时 */
        }
    }

    return true;
}

/** 多机同步触发命令：广播地址 0 + 0xFF 0x66 + 校验 */
static bool tower_sync_trigger(void)
{
    uint8_t cmd[4] = { 0x00, 0xFF, 0x66, TOWER_CHECK_BYTE };
    return tower_send_frame(&huart5, cmd, sizeof(cmd));
}

/** 速度模式运动（0xF6）。snF=1 时缓存待广播统一触发 */
static bool tower_vel_control(TowerMotorID id, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
    uint8_t cmd[8];

    if (id >= TOWER_MOTOR_NUM) return false;

    cmd[0] = s_tower[id].addr;
    cmd[1] = 0xF6;
    cmd[2] = dir;
    cmd[3] = (uint8_t)(vel >> 8);
    cmd[4] = (uint8_t)(vel >> 0);
    cmd[5] = acc;
    cmd[6] = snF ? 1 : 0;
    cmd[7] = TOWER_CHECK_BYTE;

    return tower_send_frame(s_tower[id].huart, cmd, sizeof(cmd));
}

/** 位置模式运动（0xFD）。snF=1 时缓存待广播统一触发 */
static bool tower_pos_control(TowerMotorID id, uint8_t dir, uint16_t vel, uint8_t acc,
                              uint32_t pulses, uint8_t raf, bool snF)
{
    uint8_t cmd[13];

    if (id >= TOWER_MOTOR_NUM) return false;

    cmd[0]  = s_tower[id].addr;
    cmd[1]  = 0xFD;
    cmd[2]  = dir;
    cmd[3]  = (uint8_t)(vel >> 8);
    cmd[4]  = (uint8_t)(vel >> 0);
    cmd[5]  = acc;
    cmd[6]  = (uint8_t)(pulses >> 24);
    cmd[7]  = (uint8_t)(pulses >> 16);
    cmd[8]  = (uint8_t)(pulses >> 8);
    cmd[9]  = (uint8_t)(pulses >> 0);
    cmd[10] = raf;
    cmd[11] = snF ? 1 : 0;
    cmd[12] = TOWER_CHECK_BYTE;

    return tower_send_frame(s_tower[id].huart, cmd, sizeof(cmd));
}

/** 立即停止命令（0xFE）。snF=1 时缓存待广播统一触发 */
static bool tower_stop_control(TowerMotorID id, bool snF)
{
    uint8_t cmd[5];

    if (id >= TOWER_MOTOR_NUM) return false;

    cmd[0] = s_tower[id].addr;
    cmd[1] = 0xFE;
    cmd[2] = 0x98;
    cmd[3] = snF ? 1 : 0;
    cmd[4] = TOWER_CHECK_BYTE;

    return tower_send_frame(s_tower[id].huart, cmd, sizeof(cmd));
}

/* ============================================================================
 *                            公开接口
 * ========================================================================== */

void TowerMotor_Init(void)
{
    /* 与底盘共用 UART5，这里只清一次接收残留即可 */
    tower_flush_rx(&huart5);
}

bool TowerMotor_Enable(TowerMotorID id, bool enable)
{
    uint8_t cmd[6];

    if (id >= TOWER_MOTOR_NUM) return false;

    cmd[0] = s_tower[id].addr;
    cmd[1] = 0xF3;
    cmd[2] = 0xAB;
    cmd[3] = enable ? 1 : 0;
    cmd[4] = 0;                  /* 同步标志：单机使能，立即执行 */
    cmd[5] = TOWER_CHECK_BYTE;

    return tower_send_frame(s_tower[id].huart, cmd, sizeof(cmd));
}

bool TowerMotor_Stop(TowerMotorID id)
{
    return tower_stop_control(id, false);
}

bool TowerMotor_StopAll(void)
{
    bool ok = true;

    for (int i = 0; i < TOWER_MOTOR_NUM; i++)
    {
        if (!tower_stop_control((TowerMotorID)i, true)) ok = false;
        osDelay(TOWER_SYNC_CMD_GAP_MS);
    }

    if (!tower_sync_trigger()) ok = false;
    return ok;
}

bool TowerMotor_Speed(TowerMotorID id, int16_t rpm)
{
    uint8_t  dir;
    uint16_t vel;

    if (id >= TOWER_MOTOR_NUM) return false;

    dir = ((rpm > 0) ? 0 : 1) ^ s_tower[id].invert;   /* 0=CW 1=CCW */
    vel = (uint16_t)((rpm < 0) ? -rpm : rpm);

    return tower_vel_control(id, dir, vel, TOWER_DEFAULT_ACC, false);
}

/** 返回指定电机的当前转速(RPM)，来自在线可调参数 */
static uint16_t tower_speed(TowerMotorID id)
{
    switch (id)
    {
        case TOWER_TURNTABLE: return (uint16_t)TOWER_TURNTABLE_SPEED;
        case TOWER_ARM:       return (uint16_t)TOWER_ARM_SPEED;
        case TOWER_LIFT:      return (uint16_t)TOWER_LIFT_SPEED;
        default:              return 100;
    }
}

bool TowerMotor_Move(TowerMotorID id, int32_t pulses)
{
    uint8_t  dir;
    uint32_t p;

    if (id >= TOWER_MOTOR_NUM) return false;

    dir = ((pulses > 0) ? 0 : 1) ^ s_tower[id].invert;
    p   = (uint32_t)((pulses < 0) ? -pulses : pulses);

    return tower_pos_control(id, dir, tower_speed(id), TOWER_DEFAULT_ACC,
                             p, TOWER_MOTION_MODE, false);
}

bool TowerMotor_MoveAll(const int32_t pulses[TOWER_MOTOR_NUM])
{
    bool ok = true;

    for (int i = 0; i < TOWER_MOTOR_NUM; i++)
    {
        int32_t  n = pulses[i];
        uint8_t  d = ((n > 0) ? 0 : 1) ^ s_tower[i].invert;
        uint32_t p = (uint32_t)((n < 0) ? -n : n);

        if (!tower_pos_control((TowerMotorID)i, d, tower_speed((TowerMotorID)i),
                               TOWER_DEFAULT_ACC, p, TOWER_MOTION_MODE, true)) ok = false;
        osDelay(TOWER_SYNC_CMD_GAP_MS);
    }

    if (!tower_sync_trigger()) ok = false;
    return ok;
}

/* ============================================================================
 *                视觉坐标闭环 PID（x→转台 / y→作业臂）
 * ========================================================================== */

static PID_t    s_pid_x;
static PID_t    s_pid_y;
static bool     s_pid_inited = false;
static int32_t  s_target_x = TOWER_X_TARGET;
static int32_t  s_target_y = TOWER_Y_TARGET;
static uint32_t s_last_tick = 0;
static bool     s_last_tick_valid = false;

void Tower_PID_Init(void)
{
    PID_Init(&s_pid_x, TOWER_X_KP, TOWER_X_KI, TOWER_X_KD,
             -(float)TOWER_X_MAX_RPM, (float)TOWER_X_MAX_RPM);
    PID_Init(&s_pid_y, TOWER_Y_KP, TOWER_Y_KI, TOWER_Y_KD,
             -(float)TOWER_Y_MAX_RPM, (float)TOWER_Y_MAX_RPM);
    s_pid_inited = true;
}

void Tower_SetTarget(int32_t x, int32_t y)
{
    s_target_x = x;
    s_target_y = y;
    PID_Reset(&s_pid_x);
    PID_Reset(&s_pid_y);
    s_last_tick_valid = false;
}

void Tower_SetTargetX(int32_t x)
{
    s_target_x = x;
    PID_Reset(&s_pid_x);
}

void Tower_SetTargetY(int32_t y)
{
    s_target_y = y;
    PID_Reset(&s_pid_y);
}

bool Tower_PID_Update(int32_t x, int32_t y)
{
    float    dt;
    float    rpm_x, rpm_y;
    int32_t  err_x, err_y;
    uint32_t now;
    bool     arrived_x, arrived_y;

    if (!s_pid_inited) Tower_PID_Init();

    /* 用两次调用之间的时间差作为 PID 的 dt（秒） */
    now = osKernelGetTickCount();
    if (s_last_tick_valid)
    {
        dt = (float)(now - s_last_tick) / 1000.0f;
    }
    else
    {
        dt = 0.001f;
    }
    s_last_tick = now;
    s_last_tick_valid = true;
    if (dt <= 0.0f) dt = 0.001f;

    /* 误差 = 目标值 - 当前坐标 */
    err_x = s_target_x - x;
    err_y = s_target_y - y;

    rpm_x = PID_Compute_Error(&s_pid_x, (float)err_x, dt);
    rpm_y = PID_Compute_Error(&s_pid_y, (float)err_y, dt);

    if (TOWER_X_DIR_INVERT) rpm_x = -rpm_x;
    if (TOWER_Y_DIR_INVERT) rpm_y = -rpm_y;

    /* 死区：误差足够小则转速清零，避免来回抖 */
    arrived_x = (err_x > -TOWER_X_DEADBAND && err_x < TOWER_X_DEADBAND);
    arrived_y = (err_y > -TOWER_Y_DEADBAND && err_y < TOWER_Y_DEADBAND);
    if (arrived_x) rpm_x = 0.0f;
    if (arrived_y) rpm_y = 0.0f;

    /* x 误差驱动转台(圆周)，y 误差驱动作业臂(前后) */
    TowerMotor_Speed(TOWER_TURNTABLE, (int16_t)rpm_x);
    TowerMotor_Speed(TOWER_ARM, (int16_t)rpm_y);

    return arrived_x && arrived_y;
}

void Tower_PID_Stop(void)
{
    TowerMotor_Stop(TOWER_TURNTABLE);
    TowerMotor_Stop(TOWER_ARM);
}
