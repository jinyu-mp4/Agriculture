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
  * @warning 写命令为【只发不收】（DMA 发送、忽略应答），放在任务上下文里调用即可；
  *          多机同步用「snF=1 缓存 + 0xFF 0x66 广播触发」实现四轮同时启动/变速。
  *
  *          全车串口规划：
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
#include "laser.h"
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

/* —— 激光横向居中 PID（右侧激光 → 麦轮横移分量，用于前后直行贴墙居中）—— */

/** 横向闭环控制周期(ms)。激光出数频率 30Hz（约 33ms/帧），控制环取 33ms 与之匹配 */
#define ZDT_LAT_LOOP_MS              (33)

/** 横向墙距有效范围(mm)：只有右侧激光读数在此范围内才认为“侧面有田垄/围栏”
 *  可做横向修正；低于下限或高于上限视为侧面无墙，横移清零、开环直行一小段。 */
#define ZDT_LAT_RANGE_MIN_MM         (20)
#define ZDT_LAT_RANGE_MAX_MM         (400)

/** 横向居中目标墙距(mm)：误差 = 目标值 - 实测右侧距离。按实车摆正后标定 */
#define ZDT_LAT_TARGET_MM            (85)

/** 横向 PID 增益：误差单位 mm，输出为横移修正转速(RPM)。
 *  Kp = 每 1mm 横向误差产生的修正转速。 */
#define ZDT_LAT_KP                   (1.0f)
#define ZDT_LAT_KI                   (0.0f)
#define ZDT_LAT_KD                   (0.0f)

/** 横移修正转速限幅(RPM)：避免修正过大导致整车横向打摆 */
#define ZDT_LAT_MAX_RPM              (60)

/** 横向修正方向取反开关：若实测“越修越偏”，把 0 改成 1 */
#define ZDT_LAT_DIR_INVERT           (0)

/* —— 激光纵向居中 PID（前侧激光 → 麦轮前后分量，用于左右平移 4m 时的纵向保持）—— */

/** 纵向闭环控制周期(ms)。激光出数频率 30Hz（约 33ms/帧），控制环取 33ms 与之匹配 */
#define ZDT_LON_LOOP_MS              (33)

/** 纵向墙距有效范围(mm)：只有前侧激光读数在此范围内才认为“前方有田垄/围栏”
 *  可做纵向修正；低于下限或高于上限视为前方无墙，前后修正清零、开环平移。 */
#define ZDT_LON_RANGE_MIN_MM         (20)
#define ZDT_LON_RANGE_MAX_MM         (400)

/** 纵向保持目标墙距(mm)：误差 = 目标值 - 实测前侧距离。按实车摆正后标定 */
#define ZDT_LON_TARGET_MM            (50)

/** 纵向 PID 增益：误差单位 mm，输出为前后修正转速(RPM)。 */
#define ZDT_LON_KP                   (1.0f)
#define ZDT_LON_KI                   (0.0f)
#define ZDT_LON_KD                   (0.0f)

/** 前后修正转速限幅(RPM)：避免修正过大导致整车前后打摆 */
#define ZDT_LON_MAX_RPM              (60)

/** 纵向修正方向取反开关：若实测“越修越偏”，把 0 改成 1 */
#define ZDT_LON_DIR_INVERT           (0)

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

/** 整车立即停止（四轮同步：snF=1 缓存 + 广播触发，同时停下） */
bool ZDT_Chassis_Stop(void);

/** 整车开环平移（位置模式，无陀螺仪修正）：向 dir 方向行驶 distance_mm。
 *  距离(mm)由单片机换算成轮子转角/脉冲，四轮同步启动。返回四电机是否都收到命令。 */
bool ZDT_Chassis_Move(ZDT_MoveDir dir, float distance_mm);

/** 整车原地旋转（位置模式）：angle_deg > 0 顺时针(俯视)，< 0 逆时针。 */
bool ZDT_Chassis_Rotate(float angle_deg);

/** 整车原地旋转（速度模式，连续转动）：rpm > 0 顺时针(俯视)，< 0 逆时针。
 *  需调用 ZDT_Chassis_Stop() 停止。 */
bool ZDT_Chassis_Rotate_Speed(int16_t rpm);

/** 前后直行 + 右侧激光横向居中（闭环）：向 dir 方向以 base_rpm 连续行驶，
 *  每个控制周期读右侧激光做横向 PID（目标 ZDT_LAT_TARGET_MM，有效范围 20~400mm），
 *  直到前侧激光满足停止条件才停车。dir 仅用于 MOVE_FORWARD / MOVE_BACKWARD。
 *  阻塞式：内部循环到前激光触发才返回。
 *  until_le = true  : 前激光 <= stop_mm 停车（前进）
 *  until_le = false : 前激光 >= stop_mm 停车（后退） */
bool ZDT_Chassis_Drive_LaserCenter(ZDT_MoveDir dir, int16_t base_rpm,
                                   LaserID front_laser, bool until_le, uint16_t stop_mm);

/** 左右平移 + 前侧激光纵向居中（闭环）：向 dir 方向以 base_rpm 连续行驶 distance_mm，
 *  每个控制周期读前侧激光做纵向 PID（目标 ZDT_LON_TARGET_MM，有效范围 20~400mm），
 *  把前后修正量叠加到四轮转速上，行驶完指定距离后停车。dir 仅用于 MOVE_LEFT / MOVE_RIGHT。
 *  阻塞式。 */
bool ZDT_Chassis_Drive_LongCenter(ZDT_MoveDir dir, int16_t base_rpm, float distance_mm);

/** 估算行驶指定距离所需的运动时间(ms)，按当前默认速度计算并含 20% 余量 */
uint32_t ZDT_Move_Duration_Ms(float distance_mm);

/** 估算原地旋转指定角度所需时间(ms)，按当前默认速度计算并含 20% 余量 */
uint32_t ZDT_Rotate_Duration_Ms(float angle_deg);

#ifdef __cplusplus
}
#endif

#endif /* __ZDT_MOTOR_H */
