/**
  ******************************************************************************
  * @file    tower_motor.h
  * @brief   塔台三轴 ZDT X42S 闭环步进电机驱动（与底盘共用 UART5 总线）
  *
  *          地址规划（同一 UART5 总线，按从机地址区分，只发不收）：
  *            地址 5 -> 塔台底座/转台：圆周转动
  *            地址 6 -> 作业臂前后移动（齿轮传动）
  *            地址 7 -> 作业臂升降（丝杆）
  *
  *          通信协议与底盘 zdt_motor.c 相同（Emm V5 固件）：
  *            位置模式 0xFD / 速度模式 0xF6 / 使能 0xF3 / 停止 0xFE / 广播触发 0xFF 0x66
  *
  * @note    UART5 与底盘四个电机共用；底盘与塔台若分属不同任务并发发送，
  *          需自行加互斥，否则会互相打断串口帧。
  ******************************************************************************
  */
#ifndef __TOWER_MOTOR_H
#define __TOWER_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include <stdbool.h>

/* ============================================================================
 *                    用户需要按实际修改的配置
 * ========================================================================== */

/** 三个电机各自的移动速度(RPM)，按实测标定 */
#define TOWER_TURNTABLE_SPEED      (33)    /* 转台（地址5） */
#define TOWER_ARM_SPEED            (100)   /* 作业臂前后（地址6） */
#define TOWER_LIFT_SPEED           (200)   /* 升降（地址7） */

/** 默认加速度档位 0~255，0 表示不用曲线加减速、直接启动 */
#define TOWER_DEFAULT_ACC          (100)

/** 位置模式运动模式：2 = 相对当前位置做相对运动（多次下发不累积误差） */
#define TOWER_MOTION_MODE          (2)

/** 串口发送超时时间(ms) */
#define TOWER_UART_TIMEOUT_MS      (50)

/** 多机同步下发时相邻两条命令的间隔(ms)，防粘包 */
#define TOWER_SYNC_CMD_GAP_MS      (2)

/* —— 视觉坐标闭环 PID（x→转台 / y→作业臂）—— */

/** x/y 目标坐标：期望物块在图像中的位置，按实际标定（图像中心） */
#define TOWER_X_TARGET          (684)
#define TOWER_Y_TARGET          (552)

/** x 轴 PID 增益：误差(坐标单位) -> 修正转速(RPM)。Kp = 每 1 坐标单位误差的转速 */
#define TOWER_X_KP              (0.1f)
#define TOWER_X_KI              (0.0f)
#define TOWER_X_KD              (0.0f)

/** y 轴 PID 增益：误差(坐标单位) -> 修正转速(RPM) */
#define TOWER_Y_KP              (0.1f)
#define TOWER_Y_KI              (0.0f)
#define TOWER_Y_KD              (0.0f)

/** 修正转速限幅(RPM)：避免修正过快导致塔台打摆 */
#define TOWER_X_MAX_RPM         (100)
#define TOWER_Y_MAX_RPM         (100)

/** 死区(坐标单位)：误差绝对值小于该值认为已对准，停车（避免抖动） */
#define TOWER_X_DEADBAND        (3)
#define TOWER_Y_DEADBAND        (3)

/** 方向取反开关：实测「越修越偏」时改 1 */
#define TOWER_X_DIR_INVERT      (0)
#define TOWER_Y_DIR_INVERT      (0)

/* ============================================================================
 *                            类型定义
 * ========================================================================== */

/** 塔台电机编号（地址 5~7，均挂在 UART5 总线上） */
typedef enum
{
    TOWER_TURNTABLE = 0,   /* 地址5 底座/转台：圆周转动 */
    TOWER_ARM,             /* 地址6 作业臂前后移动（齿轮传动） */
    TOWER_LIFT,            /* 地址7 作业臂升降（丝杆） */
    TOWER_MOTOR_NUM
} TowerMotorID;

/* ============================================================================
 *                            接口函数
 * ========================================================================== */

/** 初始化：清一次 UART5 接收残留（与底盘共用总线，重复调用无害） */
void TowerMotor_Init(void);

/** 单电机使能/失能 */
bool TowerMotor_Enable(TowerMotorID id, bool enable);

/** 单电机立即停止 */
bool TowerMotor_Stop(TowerMotorID id);

/** 三个电机同步停止（多机同步广播触发，同时停下） */
bool TowerMotor_StopAll(void);

/** 单电机速度模式连续转动：rpm > 0 一个方向，< 0 反方向，
 *  连续转动直到发送停止命令或新的运动命令。 */
bool TowerMotor_Speed(TowerMotorID id, int16_t rpm);

/** 单电机位置模式相对运动：pulses 带符号，正/负对应两个方向。 */
bool TowerMotor_Move(TowerMotorID id, int32_t pulses);

/** 三个电机位置模式同步相对运动：pulses[3] 依次对应
 *  TOWER_TURNTABLE / TOWER_ARM / TOWER_LIFT，带符号，广播触发同步启动。 */
bool TowerMotor_MoveAll(const int32_t pulses[TOWER_MOTOR_NUM]);

/* —— 视觉坐标闭环控制 —— */

/** 初始化视觉闭环 PID（x/y 两路）。使用前调用一次。 */
void Tower_PID_Init(void);

/** 设置 x/y 目标坐标（同时复位两路 PID 的积分/微分记忆） */
void Tower_SetTarget(int32_t x, int32_t y);

/** 分别设置单个目标坐标 */
void Tower_SetTargetX(int32_t x);
void Tower_SetTargetY(int32_t y);

/** 单次视觉闭环迭代：根据当前坐标(x,y)与目标值的误差做 PID，
 *  结果作为转速驱动电机：x -> 转台(TOWER_TURNTABLE)，y -> 作业臂(TOWER_ARM)。
 *  误差 = 目标值 - 当前坐标。需要周期性调用（随坐标接收节奏，或在任务循环里每 N ms 调一次）。
 *  返回 true 表示两个轴都已进入死区（已对准）。 */
bool Tower_PID_Update(int32_t x, int32_t y);

/** 停车：停止视觉闭环驱动的两个电机（转台 + 作业臂） */
void Tower_PID_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* __TOWER_MOTOR_H */
