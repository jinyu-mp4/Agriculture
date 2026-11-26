/**
  ******************************************************************************
  * @file    test.c
  * @brief   麦轮底盘四方向调试测试（前 -> 后 -> 左 -> 右）
  *
  *          上电后依次执行：
  *            1. 前进
  *            2. 后退
  *            3. 左移
  *            4. 右移
  *          用于验证 zdt_motor.c 里 s_dir_table 四个方向的麦轮滚向是否正确。
  *
  *          集成方法（STM32CubeIDE）：
  *            1. 把 debug/test.c、debug/test.h 加入工程编译
  *               （最简单：把两个文件复制进 Core/Src、Core/Inc 后重新编译，
  *                 CubeIDE 会自动把 Core/Src 下的 .c 纳入构建）。
  *            2. 在 freertos.c 的 MX_FREERTOS_Init() 里创建本测试任务：
  *                   Test_TaskHandle = osThreadNew(StartTest_Task, NULL, &Test_Task_attributes);
  *               调试期间请注释掉原 Chassis_Task 的创建语句，避免两个任务
  *               同时抢 UART5 总线导致串口帧互相打断。
  *            3. 观察小车是否按 前/后/左/右 移动；若某个方向反了，
  *               改 zdt_motor.c 里 s_dir_table 对应行或 s_motor[].invert 位。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "test.h"
#include "zdt_motor.h"
#include "cmsis_os.h"

/* ============================================================================
 *                            可调参数
 * ========================================================================== */

/** 每个方向行驶的距离(mm)。调试时建议先给小车四周留足空间 */
#define TEST_MOVE_DISTANCE_MM    300.0f

/** 上电后等待电机/总线稳定再开始的时间(ms) */
#define TEST_POWERON_DELAY_MS    3000

/** 每个方向动作之间的停顿时间(ms)，便于肉眼确认方向 */
#define TEST_STEP_PAUSE_MS       800

/** 是否循环测试：1 = 一直循环 前/后/左/右；0 = 只跑一遍后停车待机 */
#define TEST_LOOP                0

/* ============================================================================
 *                            任务定义
 * ========================================================================== */

osThreadId_t Test_TaskHandle;
const osThreadAttr_t Test_Task_attributes = {
  .name       = "Test_Task",
  .stack_size = 512 * 4,
  .priority   = (osPriority_t) osPriorityBelowNormal1,
};

/* ============================================================================
 *                            内部函数
 * ========================================================================== */

/** 初始化底盘电机：清 UART5 接收残留，并逐个使能四轮 */
static void TestChassis_Init(void)
{
    ZDT_Motor_Init();

    ZDT_Motor_Enable(MOTOR_LF, true);
    ZDT_Motor_Enable(MOTOR_RF, true);
    ZDT_Motor_Enable(MOTOR_LR, true);
    ZDT_Motor_Enable(MOTOR_RR, true);

    osDelay(200);   /* 使能命令下发后稍作等待 */
}

/** 朝指定方向行驶一段距离，等运动完成后再停车 */
static void TestMoveDir(ZDT_MoveDir dir)
{
    uint32_t dur;

    /* 位置模式：下发后立即返回，四轮经广播触发同步起步 */
    ZDT_Chassis_Move(dir, TEST_MOVE_DISTANCE_MM);

    /* 按当前默认速度估算运动时间（含 20% 余量），等待运动结束 */
    dur = ZDT_Move_Duration_Ms(TEST_MOVE_DISTANCE_MM);
    osDelay(dur);

    /* 四轮同步停车 */
    ZDT_Chassis_Stop();
}

/* ============================================================================
 *                            公开接口
 * ========================================================================== */

/** 麦轮四方向测试任务：前 -> 后 -> 左 -> 右 */
void StartTest_Task(void *argument)
{
    (void)argument;

    TestChassis_Init();

    /* 上电稳定后再开始 */
    osDelay(TEST_POWERON_DELAY_MS);

    do
    {
        /* 1. 前进 */
        TestMoveDir(MOVE_FORWARD);
        osDelay(TEST_STEP_PAUSE_MS);

        /* 2. 后退 */
        TestMoveDir(MOVE_BACKWARD);
        osDelay(TEST_STEP_PAUSE_MS);

        /* 3. 左移（麦轮横向平移） */
        TestMoveDir(MOVE_LEFT);
        osDelay(TEST_STEP_PAUSE_MS);

        /* 4. 右移（麦轮横向平移） */
        TestMoveDir(MOVE_RIGHT);
        osDelay(TEST_STEP_PAUSE_MS);
    }
    while (TEST_LOOP);

    /* 测试完成：停车待机 */
    ZDT_Chassis_Stop();
    for (;;)
    {
        osDelay(1000);
    }
}
