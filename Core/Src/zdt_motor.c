/**
  ******************************************************************************
  * @file    zdt_motor.c
  * @brief   张大头(ZDT)X42S 闭环步进电机四轮麦轮底盘驱动（UART5 单总线，只发不收）
  *
  *          通信协议（Emm V5 固件）：
  *            帧 = 地址 + 功能码 + 数据 + 校验字节(0x6B)
  *            位置模式(0xFD) 13 字节；速度模式(0xF6) 8 字节；
  *            使能(0xF3) 6 字节；停止(0xFE) 5 字节；多机同步触发(0xFF 0x66) 4 字节。
  *
  *          多机同步：每条运动命令带 snF 标志（0xF6->cmd[6]、0xFD->cmd[11]），
  *          snF=1 时只缓存不执行；四台都缓存后发广播 [0x00 0xFF 0x66 0x6B]
  *          统一触发，四轮同时启动/变速。
  *
  *          只发不收：写命令用 DMA 发送、忽略应答，不挂起、不读编码器。
  ******************************************************************************
  */

#include "zdt_motor.h"
#include "laser.h"
#include "pid.h"
#include "cmsis_os.h"

/* ============================================================================
 *                            常量与配置
 * ========================================================================== */

#define ZDT_PI              3.14159265358979f
#define ZDT_CHECK_BYTE      0x6B   /* 校验字节 */

/* 四个电机的硬件映射 */
typedef struct
{
    UART_HandleTypeDef *huart;  /* 该电机使用的串口 */
    uint8_t            addr;    /* 电机从机地址（各电机拨码设置，需与实际一致） */
    uint8_t            invert;  /* 方向反相标志：1=该轮“正向自转”对应电机 CCW，
                                   0=对应 CW。左侧电机通常镜像安装需置 1 */
} ZDT_MotorDef;

static const ZDT_MotorDef s_motor[MOTOR_NUM] =
{
    { &huart5, 1, 0 },   /* 左前 LF，地址 1（方向已按实车翻转） */
    { &huart5, 2, 1 },   /* 右前 RF，地址 2（方向已按实车翻转） */
    { &huart5, 3, 1 },   /* 左后 LR，地址 3 */
    { &huart5, 4, 0 },   /* 右后 RR，地址 4 */
};

/* 每个方向下四个轮子的自转系数：+1 正向自转 / -1 反向自转
 * 行 = 方向(前进/后退/左移/右移)，列 = 电机(LF/RF/LR/RR)
 * 麦轮(X 型布局)的运动规律：前进后退四轮同向；左右平移时对角线两两反向 */
static const int8_t s_dir_table[4][MOTOR_NUM] =
{
    /*           LF   RF   LR   RR */
    /* 前进 */ { 1,  1,  -1,  -1 },
    /* 后退 */ { -1,  -1,  1,  1 },
    /* 左移 */ { -1,  +1,  -1,  +1 },
    /* 右移 */ { +1,  -1,  +1,  -1 },
};

/* 原地旋转：四轮同向自转。行 = 0顺时针 / 1逆时针(俯视)，列 = 电机 */
static const int8_t s_rot_table[2][MOTOR_NUM] =
{
    /*          LF   RF   LR   RR */
    /* 顺时针 */ { +1, +1, +1, +1 },
    /* 逆时针 */ { -1, -1, -1, -1 },
};

/* ============================================================================
 *                            底层收发
 * ========================================================================== */

/** 丢弃串口接收缓冲区里残留的字节，并清溢出标志。 */
static void zdt_flush_rx(UART_HandleTypeDef *huart)
{
    while (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE) != RESET)
    {
        (void)(uint8_t)(huart->Instance->DR & 0xFFU);
    }
    __HAL_UART_CLEAR_OREFLAG(huart);
}

/** 只发不收：DMA 发送一帧，忽略电机应答。发送前清掉接收残留与溢出标志，
 *  避免多机总线上的应答字节在下一次发送前把硬件 RX 搞成溢出(ORE)。
 *  命令帧 buffer 位于上层函数的栈上，因此必须等 DMA 发送完成才能返回。 */
static bool zdt_send_frame(UART_HandleTypeDef *huart, const uint8_t *cmd, uint16_t len)
{
    uint32_t start;

    __HAL_UART_CLEAR_OREFLAG(huart);
    while (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE) != RESET)
    {
        (void)(uint8_t)(huart->Instance->DR & 0xFFU);
    }

    /* 用 DMA 发送（非阻塞），DMA 完成后由传输完成中断把 gState 置回 READY */
    if (HAL_UART_Transmit_DMA(huart, (uint8_t *)cmd, len) != HAL_OK)
    {
        return false;
    }

    start = HAL_GetTick();
    while (HAL_UART_GetState(huart) != HAL_UART_STATE_READY)
    {
        if ((HAL_GetTick() - start) > ZDT_UART_TIMEOUT_MS)
        {
            return false;   /* 超时：DMA 未在预期时间内完成 */
        }
    }

    return true;
}

/* 前置声明：多机同步触发命令 */
static bool zdt_sync_trigger(void);

/** 位置模式运动（0xFD），只发不收。snF=1 时缓存待 zdt_sync_trigger() 统一触发 */
static bool zdt_pos_control(ZDT_MotorID id, uint8_t dir, uint16_t vel,
                            uint8_t acc, uint32_t pulses, uint8_t raf, bool snF)
{
    uint8_t cmd[13];

    cmd[0]  = s_motor[id].addr;                 /* 地址 */
    cmd[1]  = 0xFD;                             /* 功能码 */
    cmd[2]  = dir;                              /* 方向 0=CW 1=CCW */
    cmd[3]  = (uint8_t)(vel >> 8);              /* 速度高字节 */
    cmd[4]  = (uint8_t)(vel >> 0);              /* 速度低字节 */
    cmd[5]  = acc;                              /* 加速度档位 */
    cmd[6]  = (uint8_t)(pulses >> 24);          /* 脉冲数(bit31-24) */
    cmd[7]  = (uint8_t)(pulses >> 16);          /* 脉冲数(bit23-16) */
    cmd[8]  = (uint8_t)(pulses >> 8);           /* 脉冲数(bit15-8)  */
    cmd[9]  = (uint8_t)(pulses >> 0);           /* 脉冲数(bit7-0)   */
    cmd[10] = raf;                              /* 运动模式 */
    cmd[11] = snF ? 1 : 0;                      /* 多机同步标志：1=缓存待触发 */
    cmd[12] = ZDT_CHECK_BYTE;                   /* 校验字节 */

    return zdt_send_frame(s_motor[id].huart, cmd, sizeof(cmd));
}

/** 速度模式运动（0xF6），只发不收。snF=1 时缓存待 zdt_sync_trigger() 统一触发 */
static bool zdt_vel_control(ZDT_MotorID id, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
    uint8_t cmd[8];

    cmd[0] = s_motor[id].addr;                 /* 地址 */
    cmd[1] = 0xF6;                             /* 功能码：速度模式 */
    cmd[2] = dir;                              /* 方向 0=CW 1=CCW */
    cmd[3] = (uint8_t)(vel >> 8);              /* 速度高字节 */
    cmd[4] = (uint8_t)(vel >> 0);              /* 速度低字节 */
    cmd[5] = acc;                              /* 加速度档位 */
    cmd[6] = snF ? 1 : 0;                      /* 多机同步标志：1=缓存待触发 */
    cmd[7] = ZDT_CHECK_BYTE;                   /* 校验字节 */

    return zdt_send_frame(s_motor[id].huart, cmd, sizeof(cmd));
}

/** 多机同步触发命令：广播地址 0 + 0xFF 0x66 + 0x6B，
 *  让所有已用 snF=1 缓存了命令的电机同时开始执行。 */
static bool zdt_sync_trigger(void)
{
    uint8_t cmd[4] = { 0x00, 0xFF, 0x66, ZDT_CHECK_BYTE };
    return zdt_send_frame(&huart5, cmd, sizeof(cmd));
}

/** 立即停止命令（0xFE），只发不收。snF=1 时缓存待 zdt_sync_trigger() 统一触发 */
static bool zdt_stop_control(ZDT_MotorID id, bool snF)
{
    uint8_t cmd[5];

    if (id >= MOTOR_NUM) return false;

    cmd[0] = s_motor[id].addr;
    cmd[1] = 0xFE;
    cmd[2] = 0x98;
    cmd[3] = snF ? 1 : 0;               /* 多机同步标志：1=缓存待触发 */
    cmd[4] = ZDT_CHECK_BYTE;

    return zdt_send_frame(s_motor[id].huart, cmd, sizeof(cmd));
}

/* ============================================================================
 *                            整车同步下发
 * ========================================================================== */

/** 多机同步下发四轮位置命令：逐台发 snF=1 缓存，最后广播触发，四轮同时起步。
 *  net[i] 为第 i 轮的“逻辑脉冲”(带符号，正值对应正向自转)。 */
static bool zdt_chassis_pos_synced(const int32_t *net, uint16_t vel, uint8_t acc)
{
    bool ok = true;

    for (int i = 0; i < MOTOR_NUM; i++)
    {
        int32_t  n = net[i];
        uint8_t  d = ((n > 0) ? 0 : 1) ^ s_motor[i].invert;  /* 0=CW 1=CCW */
        uint32_t p = (uint32_t)((n < 0) ? -n : n);

        /* snF=1 缓存，待广播触发统一启动 */
        if (!zdt_pos_control((ZDT_MotorID)i, d, vel, acc, p, ZDT_MOTION_MODE, true)) ok = false;
        osDelay(ZDT_SYNC_CMD_GAP_MS);   /* 命令间隔，防粘包 */
    }

    if (!zdt_sync_trigger()) ok = false;
    return ok;
}

/** 一次同步下发四轮转速：逐台发 snF=1 缓存，最后广播触发，四轮同时变速。 */
static bool zdt_chassis_speed_synced(const int16_t *rpm)
{
    bool ok = true;

    for (int i = 0; i < MOTOR_NUM; i++)
    {
        int16_t  r   = rpm[i];
        uint8_t  dir = ((r > 0) ? 0 : 1) ^ s_motor[i].invert;
        uint16_t vel = (uint16_t)((r < 0) ? -r : r);

        if (!zdt_vel_control((ZDT_MotorID)i, dir, vel, ZDT_DEFAULT_ACC, true)) ok = false;
        osDelay(ZDT_SYNC_CMD_GAP_MS);   /* 命令间隔，防粘包 */
    }

    if (!zdt_sync_trigger()) ok = false;
    return ok;
}

/* ============================================================================
 *                            换算工具
 * ========================================================================== */

/** 距离(mm) -> 电机脉冲数 */
static uint32_t zdt_mm_to_pulses(float mm)
{
    return (uint32_t)(mm / (ZDT_PI * ZDT_WHEEL_DIAMETER_MM) *
                      ZDT_GEAR_RATIO * ZDT_PULSES_PER_REV);
}

/** 角度(度) -> 电机脉冲数：原地旋转时每个轮子沿半径 R 的圆滚过一段弧长 */
static uint32_t zdt_angle_to_pulses(float angle_deg)
{
    if (angle_deg < 0.0f) angle_deg = -angle_deg;

    return (uint32_t)(angle_deg * ZDT_TURN_RADIUS_MM / ZDT_WHEEL_DIAMETER_MM *
                      ZDT_PULSES_PER_REV / 180.0f);
}

/** 反向：负距离/负角度时把运动方向取反 */
static ZDT_MoveDir zdt_flip_dir(ZDT_MoveDir dir)
{
    switch (dir)
    {
        case MOVE_FORWARD:  return MOVE_BACKWARD;
        case MOVE_BACKWARD: return MOVE_FORWARD;
        case MOVE_LEFT:     return MOVE_RIGHT;
        case MOVE_RIGHT:    return MOVE_LEFT;
        default:            return dir;
    }
}

/* ============================================================================
 *                            公开接口
 * ========================================================================== */

void ZDT_Motor_Init(void)
{
    /* 只发不收：清一次 UART5 的接收残留即可 */
    zdt_flush_rx(&huart5);
}

bool ZDT_Motor_Enable(ZDT_MotorID id, bool enable)
{
    uint8_t cmd[6];

    if (id >= MOTOR_NUM) return false;

    cmd[0] = s_motor[id].addr;
    cmd[1] = 0xF3;
    cmd[2] = 0xAB;
    cmd[3] = enable ? 1 : 0;
    cmd[4] = 0;                  /* 同步标志：单机使能，立即执行 */
    cmd[5] = ZDT_CHECK_BYTE;

    return zdt_send_frame(s_motor[id].huart, cmd, sizeof(cmd));
}

bool ZDT_Motor_Stop(ZDT_MotorID id)
{
    /* 单机立即停止（snF=0，立即执行，不广播） */
    return zdt_stop_control(id, false);
}

bool ZDT_Motor_Speed(ZDT_MotorID id, int16_t rpm)
{
    uint8_t  dir;
    uint16_t vel;

    if (id >= MOTOR_NUM) return false;

    dir = ((rpm > 0) ? 0 : 1) ^ s_motor[id].invert;   /* 0=CW 1=CCW */
    vel = (uint16_t)((rpm < 0) ? -rpm : rpm);

    return zdt_vel_control(id, dir, vel, ZDT_DEFAULT_ACC, false);
}

bool ZDT_Chassis_Stop(void)
{
    bool ok = true;

    /* 多机同步停车：逐台发 snF=1 缓存，最后广播触发，四轮同时停下 */
    for (int i = 0; i < MOTOR_NUM; i++)
    {
        if (!zdt_stop_control((ZDT_MotorID)i, true)) ok = false;
        osDelay(ZDT_SYNC_CMD_GAP_MS);
    }

    if (!zdt_sync_trigger()) ok = false;
    return ok;
}

bool ZDT_Chassis_Move(ZDT_MoveDir dir, float distance_mm)
{
    int32_t  net[MOTOR_NUM];
    uint32_t pulses;

    /* 负距离视为反方向 */
    if (distance_mm < 0.0f)
    {
        distance_mm = -distance_mm;
        dir = zdt_flip_dir(dir);
    }

    pulses = zdt_mm_to_pulses(distance_mm);
    if (pulses == 0) return false;

    for (int i = 0; i < MOTOR_NUM; i++)
    {
        net[i] = (int32_t)s_dir_table[dir][i] * (int32_t)pulses;
    }

    return zdt_chassis_pos_synced(net, ZDT_DEFAULT_SPEED_RPM, ZDT_DEFAULT_ACC);
}

bool ZDT_Chassis_Rotate(float angle_deg)
{
    int32_t  net[MOTOR_NUM];
    uint32_t pulses;
    int      row;

    /* 旋转方向取反开关（实车方向反了改宏即可） */
    if (ZDT_ROTATE_DIR_INVERT) angle_deg = -angle_deg;

    pulses = zdt_angle_to_pulses(angle_deg);
    if (pulses == 0) return false;

    row = (angle_deg > 0.0f) ? 0 : 1;   /* 0 顺时针 / 1 逆时针 */

    for (int i = 0; i < MOTOR_NUM; i++)
    {
        net[i] = (int32_t)s_rot_table[row][i] * (int32_t)pulses;
    }

    return zdt_chassis_pos_synced(net, ZDT_DEFAULT_SPEED_RPM, ZDT_DEFAULT_ACC);
}

bool ZDT_Chassis_Rotate_Speed(int16_t rpm)
{
    bool ok = true;

    if (ZDT_ROTATE_DIR_INVERT) rpm = (int16_t)-rpm;

    for (int i = 0; i < MOTOR_NUM; i++)
    {
        if (!ZDT_Motor_Speed((ZDT_MotorID)i, rpm)) ok = false;
    }

    return ok;
}

uint32_t ZDT_Move_Duration_Ms(float distance_mm)
{
    float wheel_circ_mm;   /* 轮子周长(mm) */
    float sec;             /* 理论运动时间(s) */

    if (distance_mm < 0.0f) distance_mm = -distance_mm;

    /* 时间(s) = 圈数 / 转速(rps) = (距离 / 轮周长 * 减速比) / (RPM / 60) */
    wheel_circ_mm = ZDT_PI * ZDT_WHEEL_DIAMETER_MM;
    sec = distance_mm / wheel_circ_mm * ZDT_GEAR_RATIO * 60.0f / ZDT_DEFAULT_SPEED_RPM;

    /* 加 20% 余量，覆盖加速度爬升与电机响应时间 */
    return (uint32_t)(sec * 1000.0f * 1.2f);
}

uint32_t ZDT_Rotate_Duration_Ms(float angle_deg)
{
    float a = angle_deg < 0.0f ? -angle_deg : angle_deg;
    float wheel_arc_mm;   /* 每个轮子滚过的弧长(mm) */
    float sec;

    /* 弧长 = 半径 * 角度(rad) */
    wheel_arc_mm = a * ZDT_PI / 180.0f * ZDT_TURN_RADIUS_MM;
    sec = wheel_arc_mm / (ZDT_PI * ZDT_WHEEL_DIAMETER_MM) * 60.0f / ZDT_DEFAULT_SPEED_RPM;

    /* 加 20% 余量 */
    return (uint32_t)(sec * 1000.0f * 1.2f);
}

/* ============================================================================
 *                激光横向居中（速度模式 + 右侧激光 PID）
 * ========================================================================== */

/* 横向 PID 记忆：跨多次调用保持，避免每次直行都从零积分开始 */
static PID_t s_lat_pid;
static bool  s_lat_pid_inited = false;

/** 前后直行 + 右侧激光横向居中（闭环）：
 *  向 dir 方向以 base_rpm 连续行驶，每个控制周期读右侧激光做横向 PID，
 *  把横移修正量叠加到四轮转速上（麦轮全向，前后平移与横向平移可同时）。
 *  直到前侧激光满足停止条件才停车。dir 仅用于 MOVE_FORWARD / MOVE_BACKWARD。
 *  阻塞式。
 */
bool ZDT_Chassis_Drive_LaserCenter(ZDT_MoveDir dir, int16_t base_rpm,
                                   LaserID front_laser, bool until_le, uint16_t stop_mm)
{
    uint32_t t_last;

    if (!s_lat_pid_inited)
    {
        PID_Init(&s_lat_pid, ZDT_LAT_KP, ZDT_LAT_KI, ZDT_LAT_KD,
                 -(float)ZDT_LAT_MAX_RPM, (float)ZDT_LAT_MAX_RPM);
        float ilim = (ZDT_LAT_KI > 0.0f)
                     ? (float)ZDT_LAT_MAX_RPM / ZDT_LAT_KI
                     : (float)ZDT_LAT_MAX_RPM;
        PID_SetIntLimit(&s_lat_pid, -ilim, ilim);
        s_lat_pid_inited = true;
    }
    PID_Reset(&s_lat_pid);

    t_last = osKernelGetTickCount();

    for (;;)
    {
        /* 前激光触发则停车（与 chassis_move_until 相同的停止逻辑） */
        uint16_t d_front = Laser_GetDistance(front_laser);
        if (Laser_IsValid(front_laser) &&
            (until_le ? (d_front <= stop_mm) : (d_front >= stop_mm)))
        {
            break;
        }

        uint32_t now = osKernelGetTickCount();
        float    dt  = (float)(now - t_last) / 1000.0f;
        if (dt <= 0.0f) dt = 0.001f;
        t_last = now;

        /* 横向修正：只有右激光在 [20,400]mm 内（侧面有田垄/围栏）才有效；
         * 否则视为无墙，清积分、横移清零，开环直行一小段。 */
        float    strafe = 0.0f;
        uint16_t d_side = Laser_GetDistance(LASER_RIGHT);
        if (Laser_IsValid(LASER_RIGHT) &&
            d_side >= ZDT_LAT_RANGE_MIN_MM && d_side <= ZDT_LAT_RANGE_MAX_MM)
        {
            float err = (float)ZDT_LAT_TARGET_MM - (float)d_side;
            strafe = PID_Compute_Error(&s_lat_pid, err, dt);
            if (ZDT_LAT_DIR_INVERT) strafe = -strafe;
        }
        else
        {
            PID_Reset(&s_lat_pid);
        }

        /* 前后平移分量 + 横向修正分量，合成后限幅 */
        int16_t rpm[MOTOR_NUM];
        for (int i = 0; i < MOTOR_NUM; i++)
        {
            float v = (float)base_rpm * (float)s_dir_table[dir][i]
                    + strafe * (float)s_dir_table[MOVE_LEFT][i];
            if (v >  3000.0f) v =  3000.0f;
            if (v < -3000.0f) v = -3000.0f;
            rpm[i] = (int16_t)v;
        }

        zdt_chassis_speed_synced(rpm);

        osDelay(ZDT_LAT_LOOP_MS);
    }

    ZDT_Chassis_Stop();
    return true;
}

/* ============================================================================
 *                激光纵向居中（速度模式 + 前侧激光 PID）
 * ========================================================================== */

/* 纵向 PID 记忆：跨多次调用保持 */
static PID_t s_lon_pid;
static bool  s_lon_pid_inited = false;

/** 左右平移 + 前侧激光纵向居中（闭环）：向 dir 方向以 base_rpm 连续行驶 distance_mm，
 *  每个控制周期读前侧激光做纵向 PID（目标 ZDT_LON_TARGET_MM，有效范围 20~400mm），
 *  把前后修正量叠加到四轮转速上，行驶完指定距离后停车。dir 仅用于 MOVE_LEFT / MOVE_RIGHT。
 *  阻塞式。
 */
bool ZDT_Chassis_Drive_LongCenter(ZDT_MoveDir dir, int16_t base_rpm, float distance_mm)
{
    uint32_t t_start, t_last, duration_ms;
    float    wheel_circ, sec;
    int16_t  rpm_mag;

    if (!s_lon_pid_inited)
    {
        PID_Init(&s_lon_pid, ZDT_LON_KP, ZDT_LON_KI, ZDT_LON_KD,
                 -(float)ZDT_LON_MAX_RPM, (float)ZDT_LON_MAX_RPM);
        float ilim = (ZDT_LON_KI > 0.0f)
                     ? (float)ZDT_LON_MAX_RPM / ZDT_LON_KI
                     : (float)ZDT_LON_MAX_RPM;
        PID_SetIntLimit(&s_lon_pid, -ilim, ilim);
        s_lon_pid_inited = true;
    }
    PID_Reset(&s_lon_pid);

    /* 匀速段行驶时间：t = 距离 / 轮周长 * 减速比 * 60 / 转速（速度模式匀速，无余量） */
    if (distance_mm < 0.0f) distance_mm = -distance_mm;
    rpm_mag = (base_rpm < 0) ? (int16_t)(-base_rpm) : base_rpm;
    if (rpm_mag == 0) rpm_mag = ZDT_DEFAULT_SPEED_RPM;
    wheel_circ = ZDT_PI * ZDT_WHEEL_DIAMETER_MM;
    sec = distance_mm / wheel_circ * ZDT_GEAR_RATIO * 60.0f / (float)rpm_mag;
    duration_ms = (uint32_t)(sec * 1000.0f);

    t_start = osKernelGetTickCount();
    t_last  = t_start;

    for (;;)
    {
        uint32_t now = osKernelGetTickCount();
        if (now - t_start >= duration_ms) break;

        float dt = (float)(now - t_last) / 1000.0f;
        if (dt <= 0.0f) dt = 0.001f;
        t_last = now;

        /* 纵向修正：只有前激光在 [20,400]mm 内（前方有田垄/围栏）才有效；
         * 否则视为前方无墙，清积分、前后修正清零，开环平移。 */
        float    lon     = 0.0f;
        uint16_t d_front = Laser_GetDistance(LASER_FRONT);
        if (Laser_IsValid(LASER_FRONT) &&
            d_front >= ZDT_LON_RANGE_MIN_MM && d_front <= ZDT_LON_RANGE_MAX_MM)
        {
            float err = (float)ZDT_LON_TARGET_MM - (float)d_front;
            lon = PID_Compute_Error(&s_lon_pid, err, dt);
            if (ZDT_LON_DIR_INVERT) lon = -lon;
        }
        else
        {
            PID_Reset(&s_lon_pid);
        }

        /* 平移分量 + 纵向修正分量，合成后限幅 */
        int16_t rpm[MOTOR_NUM];
        for (int i = 0; i < MOTOR_NUM; i++)
        {
            float v = (float)base_rpm * (float)s_dir_table[dir][i]
                    + lon * (float)s_dir_table[MOVE_BACKWARD][i];
            if (v >  3000.0f) v =  3000.0f;
            if (v < -3000.0f) v = -3000.0f;
            rpm[i] = (int16_t)v;
        }

        zdt_chassis_speed_synced(rpm);

        osDelay(ZDT_LON_LOOP_MS);
    }

    ZDT_Chassis_Stop();
    return true;
}

/* ============================================================================
 *                            HAL 回调（ISR 上下文）
 * ========================================================================== */

/** 单字节中断接收完成：按串口分发给对应驱动，并各自重新武装接收 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* USART2(右侧) / USART3(前侧) = 激光测距 */
    if (huart == &huart2 || huart == &huart3)
    {
        Laser_FeedByte(huart);
        return;
    }
}

/** 串口出错（如溢出 ORE/帧错 FE）：恢复对应驱动的单字节接收，避免断流 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2 || huart == &huart3)
    {
        Laser_UART_Error(huart);
        return;
    }
}
