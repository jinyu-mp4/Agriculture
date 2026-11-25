/**
  ******************************************************************************
  * @file    tower_motor.c
  * @brief   塔台三轴 ZDT X42S 闭环步进电机驱动（UART5 总线，只发不收）
  *
  *          协议与底盘 zdt_motor.c 一致，这里单独成文件，方便塔台独立维护。
  *          写命令使用 DMA 发送，忽略驱动应答。
  ******************************************************************************
  */

#include "tower_motor.h"
#include "cmsis_os.h"
#include <math.h>

/* 校验字节 */
#define TOWER_CHECK_BYTE   0x6B
#define TOWER_PULSES_PER_REV  3200.0f  /* 塔台使用 16 细分 */
#define TOWER_ABSOLUTE_MODE   1U

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

/** 用 DMA 连续发完整帧，结束后留出驱动要求的帧间隔。 */
static bool tower_send_frame(UART_HandleTypeDef *huart, const uint8_t *cmd, uint16_t len)
{
    tower_flush_rx(huart);
    if (HAL_UART_Transmit_DMA(huart, (uint8_t *)cmd, len) != HAL_OK)
    {
        osDelay(TOWER_CMD_GAP_MS);
        return false;
    }

    uint32_t start = HAL_GetTick();
    while (HAL_UART_GetState(huart) != HAL_UART_STATE_READY)
    {
        if (HAL_GetTick() - start >= TOWER_UART_TIMEOUT_MS)
        {
            /* 停止 DMA，避免返回后继续访问调用方的栈缓冲区。 */
            HAL_UART_AbortTransmit(huart);
            osDelay(TOWER_CMD_GAP_MS);
            return false;
        }
        osDelay(1);
    }

    osDelay(TOWER_CMD_GAP_MS);
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
    }

    if (!tower_sync_trigger()) ok = false;
    return ok;
}

static uint8_t tower_acc(TowerMotorID id);   /* 前向声明，供 TowerMotor_Speed 使用 */

bool TowerMotor_Speed(TowerMotorID id, int16_t rpm)
{
    uint8_t  dir;
    uint16_t vel;

    if (id >= TOWER_MOTOR_NUM) return false;

    dir = ((rpm > 0) ? 0 : 1) ^ s_tower[id].invert;   /* 0=CW 1=CCW */
    vel = (uint16_t)((rpm < 0) ? -rpm : rpm);

    return tower_vel_control(id, dir, vel, tower_acc(id), false);
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

/** 返回指定电机的加速度档位，来自在线可调参数 */
static uint8_t tower_acc(TowerMotorID id)
{
    switch (id)
    {
        case TOWER_TURNTABLE: return (uint8_t)TOWER_TURNTABLE_ACC;
        case TOWER_ARM:       return (uint8_t)TOWER_ARM_ACC;
        case TOWER_LIFT:      return (uint8_t)TOWER_LIFT_ACC;
        default:              return (uint8_t)TOWER_DEFAULT_ACC;
    }
}

static bool tower_move(TowerMotorID id, int32_t pulses, uint8_t mode, bool sync)
{
    if (id >= TOWER_MOTOR_NUM) return false;

    uint8_t dir = ((pulses > 0 || (pulses == 0 && mode == TOWER_ABSOLUTE_MODE)) ? 0 : 1)
                  ^ s_tower[id].invert;
    uint32_t magnitude = (pulses < 0) ? (0U - (uint32_t)pulses) : (uint32_t)pulses;
    return tower_pos_control(id, dir, tower_speed(id), tower_acc(id),
                             magnitude, mode, sync);
}

bool TowerMotor_Move(TowerMotorID id, int32_t pulses)
{
    return tower_move(id, pulses, TOWER_MOTION_MODE, false);
}

bool TowerMotor_ZeroPosition(TowerMotorID id)
{
    if (id >= TOWER_MOTOR_NUM) return false;

    const uint8_t cmd[] = {s_tower[id].addr, 0x0A, 0x6D, TOWER_CHECK_BYTE};
    return tower_send_frame(s_tower[id].huart, cmd, sizeof(cmd));
}

bool TowerMotor_MoveAbsolute(TowerMotorID id, int32_t target)
{
    return tower_move(id, target, TOWER_ABSOLUTE_MODE, false);
}

uint32_t TowerMotor_TravelTimeMs(TowerMotorID id, int32_t pulses)
{
    if (id >= TOWER_MOTOR_NUM || pulses == 0) return 0;

    float rpm = (float)tower_speed(id);
    float revolutions = fabsf((float)pulses) / TOWER_PULSES_PER_REV;
    uint8_t acc = tower_acc(id);
    float seconds_per_rpm = (acc == 0) ? 0.0f :
                             (256 - acc) * 0.000050f;
    float ramp_seconds = rpm * seconds_per_rpm;
    float travel_seconds = revolutions * 60.0f / rpm;

    /* Emm 加减速对称。短行程在到达设定转速前就开始减速。 */
    if (travel_seconds < ramp_seconds)
    {
        travel_seconds = 2.0f * sqrtf(revolutions * 60.0f * seconds_per_rpm);
    }
    else
    {
        travel_seconds += ramp_seconds;
    }
    return (uint32_t)ceilf(travel_seconds * 1000.0f);
}

bool TowerMotor_MoveAll(const int32_t pulses[TOWER_MOTOR_NUM])
{
    bool ok = true;

    for (int i = 0; i < TOWER_MOTOR_NUM; i++)
    {
        if (!tower_move((TowerMotorID)i, pulses[i], TOWER_MOTION_MODE, true)) ok = false;
    }

    if (!tower_sync_trigger()) ok = false;
    return ok;
}
