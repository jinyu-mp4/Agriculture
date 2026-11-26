# 麦轮底盘四方向调试测试

本文件夹用于调试 `zdt_motor.c` 里麦轮底盘的前/后/左/右方向。

## 测试内容

上电后依次执行：

1. 前进（`MOVE_FORWARD`）
2. 后退（`MOVE_BACKWARD`）
3. 左移（`MOVE_LEFT`）
4. 右移（`MOVE_RIGHT`）

每个方向行驶 `TEST_MOVE_DISTANCE_MM`（默认 300mm），跑完后停车待机。

## 集成步骤

1. 把 `test.c` 复制进 `Core/Src`、`test.h` 复制进 `Core/Inc`，CubeIDE 重新构建即可自动编译。
2. 在 `Core/Src/freertos.c` 的 `MX_FREERTOS_Init()` 里创建测试任务：
   ```c
   #include "test.h"
   ...
   Test_TaskHandle = osThreadNew(StartTest_Task, NULL, &Test_Task_attributes);
   ```
3. **调试期间注释掉原 `Chassis_Task` 的创建语句**，避免两个任务同时抢 UART5 总线。

## 方向反了怎么办

- 前进/后退反了：改 `zdt_motor.c` 里 `s_dir_table` 的 `MOVE_FORWARD` / `MOVE_BACKWARD` 两行。
- 左移/右移反了：改 `s_dir_table` 的 `MOVE_LEFT` / `MOVE_RIGHT` 两行。
- 单个轮子方向反了：改 `zdt_motor.c` 里 `s_motor[].invert` 对应位。
