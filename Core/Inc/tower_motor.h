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
#define TOWER_TURNTABLE_SPEED      (50)    /* 转台（地址5） */
#define TOWER_ARM_SPEED            (200)   /* 作业臂前后（地址6） */
#define TOWER_LIFT_SPEED           (700)   /* 升降（地址7） */

/** 三个电机各自的加速度档位 0~255，数值越大加速越快（0 表示不加减速、直接启动，冲击最大）。
 *  升降(地址7)行程短、容易被加减速拖慢，单独调高让它爬升更快。 */
#define TOWER_TURNTABLE_ACC        (100)   /* 转台（地址5） */
#define TOWER_ARM_ACC              (100)   /* 作业臂（地址6） */
#define TOWER_LIFT_ACC             (200)   /* 升降（地址7） */

/** 默认加速度档位 0~255，0 表示不用曲线加减速、直接启动 */
#define TOWER_DEFAULT_ACC          (100)

/** 位置模式运动模式：2 = 相对当前位置做相对运动（多次下发不累积误差） */
#define TOWER_MOTION_MODE          (2)

/** 串口发送超时时间(ms) */
#define TOWER_UART_TIMEOUT_MS      (50)

/** 相邻命令的最小间隔(ms)，给驱动留出解析时间 */
#define TOWER_CMD_GAP_MS           (4)

/** x/y 目标坐标：期望物块在图像中的位置，按实际标定（图像中心） */
#define TOWER_X_TARGET          (694)
#define TOWER_Y_TARGET          (581)

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

/** 将静止电机的当前位置设为坐标零点。 */
bool TowerMotor_ZeroPosition(TowerMotorID id);

/** 单电机位置模式绝对运动：target 为相对坐标零点的有符号脉冲数。 */
bool TowerMotor_MoveAbsolute(TowerMotorID id, int32_t target);

/** 用位置模式的速度和加速度估算静止起停的行程时间(ms)。 */
uint32_t TowerMotor_TravelTimeMs(TowerMotorID id, int32_t pulses);

/** 三个电机位置模式同步相对运动：pulses[3] 依次对应
 *  TOWER_TURNTABLE / TOWER_ARM / TOWER_LIFT，带符号，广播触发同步启动。 */
bool TowerMotor_MoveAll(const int32_t pulses[TOWER_MOTOR_NUM]);

#ifdef __cplusplus
}
#endif

#endif /* __TOWER_MOTOR_H */
