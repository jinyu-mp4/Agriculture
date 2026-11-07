/**
  ******************************************************************************
  * @file    zdt_motor.h
  * @brief   张大头(ZDT)X42S 闭环步进电机四轮麦轮底盘驱动
  *
  * @note    四个电机共用 UART5 一条总线、按从机地址区分（只发不收，不读应答）：
  *           左前(LF)  -> 地址 1
  *           右前(RF)  -> 地址 2
  *           左后(LR)  -> 地址 3
  *           右后(RR)  -> 地址 4
  *           麦轮朝向：LF 左上45° / RF 右上45° / LR 右上45° / RR 左上45°
  *           接线：PC12(UART5_TX) 接四台电机的 RX；四台电机 TX 不接。
  *
  * @warning 写命令为【只发不收】（阻塞发送、忽略应答），放在任务上下文里调用即可；
  *          多机同步用「snF=1 缓存 + 0xFF 0x66 广播触发」实现四轮同时启动/变速。
  *
  *          全车串口规划：
  *            USART1  PA9/PA10   -> 陀螺仪(ICM42688 串口姿态模块，见 imu601)
  *            UART5   PC12/PD2   -> 四个 ZDT 电机(本文件)
  *            USART2  PA2/PA3    -> 右侧激光测距(见 laser)
  *            USART3  PB10/PB11  -> 前侧激光测距(见 laser)
  ******************************************************************************
  */
#ifndef __ZDT_MOTOR_H
#define __ZDT_MOTOR_H

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

/** 轮子（麦轮）外直径，单位 mm。务必按实测值修改 */
#define ZDT_WHEEL_DIAMETER_MM        (77.0f)

/** 减速比（电机 : 轮子）。
 *  电机轴直连轮子填 1.0f；若电机转 N 圈轮子才转 1 圈，填 (float)N */
#define ZDT_GEAR_RATIO               (1.0f)

/** 电机每转一圈的脉冲数。默认 16 细分时一圈为 3200 个脉冲 */
#define ZDT_PULSES_PER_REV           (3200)

/** 默认速度(RPM)，范围 0~3000（手册 0x0BB8=3000）。
 *  注意：12V 供电时先降到 200 左右，换 24V 后可调回 500 甚至更高 */
#define ZDT_DEFAULT_SPEED_RPM        (100)

/** 默认加速度档位 0~255，0 表示不用曲线加减速、直接启动（启动冲击电流最大）。
 *  12V 供电时建议用 100 让转速缓慢爬升，避免瞬间堵转触发保护 */
#define ZDT_DEFAULT_ACC              (0)

/** 串口发送超时时间(ms) */
#define ZDT_UART_TIMEOUT_MS          (50)

/** 位置模式运动模式：2 = 相对当前实时位置做相对运动（推荐，多次下发不累积误差） */
#define ZDT_MOTION_MODE              (2)

/** 原地旋转时，底盘中心到轮子的水平距离(mm)。麦轮底盘原地旋转时每个轮子
 *  沿半径为该值的圆滚动，此值需按实车量测（约等于半轮距/轴距对角线的 1/2）。 */
#define ZDT_TURN_RADIUS_MM           (135.0f)

/** 旋转方向取反开关：若航向校正越修越偏（说明原地旋转方向反了），
 *  把 0 改成 1 即可，无需改动方向表。 */
#define ZDT_ROTATE_DIR_INVERT        (0)

/* —— 位置模式直行（分段 + 陀螺仪航向外环 PID）—— */

/** 直行切分的小段长度(mm)。越小航向锁得越紧，但命令更密、耗时略增 */
#define ZDT_SEGMENT_MM               (40.0f)

/** 位置模式航向外环 PID：误差单位为度，输出为本段旋转修正角(度) */
#define ZDT_HEADING_KP               (0.5f)
#define ZDT_HEADING_KI               (0.1f)
#define ZDT_HEADING_KD               (0.0f)

/** 单段允许的最大旋转修正角(度)，防止单段修正过大走出蛇形 */
#define ZDT_HEADING_MAX_CORR_DEG     (8.0f)

/* —— 速度模式航向保持 PID（外环陀螺仪 yaw → 四轮转速修正，用于连续直行）—— */

/** 航向环控制周期(ms)：每个周期读一次陀螺仪 yaw 并同步下发四轮转速 */
#define ZDT_HDG_LOOP_MS              (20)

/** 航向 PID 增益：误差单位为度，输出为修正转速(RPM)。
 *  Kp = 每 1° 航向误差产生的修正转速；Ki = 每 (1°·秒) 产生的修正转速。 */
#define ZDT_HDG_KP                   (3.0f)
#define ZDT_HDG_KI                   (0.0f)
#define ZDT_HDG_KD                   (0.0f)

/** 修正转速限幅(RPM)：避免修正量过大导致某轮反转或整车打转 */
#define ZDT_HDG_MAX_RPM              (40)

/** 多机同步下发时相邻两条电机命令的间隔(ms)。间隔太短可能出现“粘包”
 *  （电机把两帧当一帧）；示例工程用 10ms，这里取 2ms 已足够。 */
#define ZDT_SYNC_CMD_GAP_MS          (2)

/* ============================================================================
 *                            类型定义
 * ========================================================================== */

/** 电机编号（按从机地址 1~4 排列，均挂在 UART5 总线上） */
typedef enum
{
    MOTOR_LF = 0,   /* 左前 -> 地址 1, 麦轮左上 45° */
    MOTOR_RF,       /* 右前 -> 地址 2, 麦轮右上 45° */
    MOTOR_LR,       /* 左后 -> 地址 3, 麦轮右上 45° */
    MOTOR_RR,       /* 右后 -> 地址 4, 麦轮左上 45° */
    MOTOR_NUM
} ZDT_MotorID;

/** 整车运动方向 */
typedef enum
{
    MOVE_FORWARD = 0,   /* 前进 */
    MOVE_BACKWARD,      /* 后退 */
    MOVE_LEFT,          /* 左移（麦轮横向平移） */
    MOVE_RIGHT,         /* 右移（麦轮横向平移） */
} ZDT_MoveDir;

/* ============================================================================
 *                            接口函数
 * ========================================================================== */

/** 初始化：清一次 UART5 接收残留。必须在使用其它接口前调用一次。 */
void ZDT_Motor_Init(void);

/** 单电机使能/失能 */
bool ZDT_Motor_Enable(ZDT_MotorID id, bool enable);

/** 单电机立即停止 */
bool ZDT_Motor_Stop(ZDT_MotorID id);

/** 单电机速度模式运动：rpm > 0 顺时针(CW)，< 0 逆时针(CCW)。
 *  连续转动，直到发送停止命令或新的运动命令。 */
bool ZDT_Motor_Speed(ZDT_MotorID id, int16_t rpm);

/** 整车立即停止（四个电机都发停止命令） */
bool ZDT_Chassis_Stop(void);

/** 整车开环平移（位置模式，无陀螺仪修正）：向 dir 方向行驶 distance_mm。
 *  距离(mm)由单片机换算成轮子转角/脉冲，四轮同步启动。返回四电机是否都收到命令。 */
bool ZDT_Chassis_Move(ZDT_MoveDir dir, float distance_mm);

/** 直行（位置模式 + 陀螺仪航向修正）：把 distance_mm 切成小段，每段下发
 *  「平移脉冲 + 航向修正旋转脉冲」，用陀螺仪 yaw 锁 target_yaw_deg 走直线。
 *  阻塞式：内部循环到各段跑完才返回，返回后即可读激光判断下一步动作。 */
bool ZDT_Chassis_Move_Straight(ZDT_MoveDir dir, float distance_mm, float target_yaw_deg);

/** 整车原地旋转（位置模式）：angle_deg > 0 顺时针(俯视)，< 0 逆时针。 */
bool ZDT_Chassis_Rotate(float angle_deg);

/** 整车原地旋转（速度模式，连续转动）：rpm > 0 顺时针(俯视)，< 0 逆时针。
 *  需调用 ZDT_Chassis_Stop() 停止。 */
bool ZDT_Chassis_Rotate_Speed(int16_t rpm);

/** 速度模式航向保持：向 dir 方向以 base_rpm 平移 duration_ms 毫秒，全程用陀螺仪
 *  yaw 做外环 PID，把四轮转速叠加旋转修正量来锁定 target_yaw_deg 航向。
 *  阻塞式：内部循环到 duration_ms 后自动停车。target_yaw_deg 通常取上电后的当前航向。 */
void ZDT_Chassis_Drive_Heading(ZDT_MoveDir dir, int16_t base_rpm,
                               float target_yaw_deg, uint32_t duration_ms);

/** 估算行驶指定距离所需的运动时间(ms)，按当前默认速度计算并含 20% 余量 */
uint32_t ZDT_Move_Duration_Ms(float distance_mm);

/** 估算原地旋转指定角度所需时间(ms)，按当前默认速度计算并含 20% 余量 */
uint32_t ZDT_Rotate_Duration_Ms(float angle_deg);

#ifdef __cplusplus
}
#endif

#endif /* __ZDT_MOTOR_H */
