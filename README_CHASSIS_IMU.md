# Agriculture 底盘与 ICM42688 调试说明

## 先看这里：实车要核对和调整的参数

工程目录是 `Agriculture`，CubeMX 配置在 [Agriculture.ioc](Agriculture.ioc)，编译使用 [Makefile](Makefile)。下表的文件都属于这个固件工程。`others/ICM42688资料包-iic/` 是移植 I2C 驱动时参考的例程。

| 优先核对或调整 | 文件和位置 | 当前值及调整方法 |
| --- | --- | --- |
| **陀螺仪 PID 目标角度** | [Core/Src/freertos.c](Core/Src/freertos.c)，`chassis_drive_leg()` 中的 `yaw_error` | **当前固定为 `0.0°`。把车摆到期望朝向，读取 LCD 上的 `YAW`，再按下节设置目标角。** |
| YAW 左右方向 | [Core/Src/icm42688.c](Core/Src/icm42688.c)，`ICM42688_YAW_SIGN` | 默认 `-1.0f`，对应芯片 Z 轴朝上。车头右转 1° 应显示约 `1.0°`，左转 1° 应显示约 `359.0°`。传感器方向相反时改符号。 |
| 四轮旋转方向 | [Core/Src/zdt_motor.c](Core/Src/zdt_motor.c)，`s_clockwise`；[Core/Inc/zdt_motor.h](Core/Inc/zdt_motor.h)，`ZDT_ROTATE_DIR_INVERT` | 先架空按地址 `1–4` 核对旋转轮序。当前 `{ -1, +1, +1, -1 }` 是按后排左右地址互换推定的。轮序正确但整体转向相反时，把 `ZDT_ROTATE_DIR_INVERT` 改为 `1`。 |
| 提前减速 | [Core/Src/freertos.c](Core/Src/freertos.c)，`CHASSIS_BRAKE_LEAD_MM`、`CHASSIS_BRAKE_RPM_PER_S` | 默认提前 `250 mm`，按 `400 RPM/s` 降速。`case 0/1` 的终点主阈值是 `80 mm`，因此主激光读数达到 `330 mm` 或更近时开始减速。若到停车点还没降到 `75 RPM`，先增大提前量。 |
| 前、右激光位置目标 | [Core/Inc/zdt_motor.h](Core/Inc/zdt_motor.h)，`ZDT_LON_TARGET_MM`、`ZDT_LAT_TARGET_MM`；[Core/Src/freertos.c](Core/Src/freertos.c)，`s_route` | `case 0` 的前距目标是 `40 mm`，`case 1` 的右距目标是 `80 mm`。后续前后行驶段的右距目标在各自表项末尾，按实测垄间位置调整。 |
| 航向 PID 强度 | [Core/Src/freertos.c](Core/Src/freertos.c)，`CHASSIS_YAW_KP`、`CHASSIS_YAW_KI`、`CHASSIS_YAW_KD`、`CHASSIS_YAW_MAX_RPM`、`CHASSIS_YAW_DEADBAND_DEG` | 当前 `Kp=0.8`、`Ki=0`、`Kd=0.06`，死区 `0.8°`，修正限幅 `±10 RPM`。轮子抖动时可先减小 `Kp` 或适当放宽死区。 |
| 巡航速度和加速度 | [Core/Src/freertos.c](Core/Src/freertos.c)，`CHASSIS_FAST_RPM`、`CHASSIS_WORK_RPM`；[Core/Inc/zdt_motor.h](Core/Inc/zdt_motor.h)，`ZDT_DEFAULT_ACC` | 快速段 `200 RPM`，遍历段和快速段末尾 `75 RPM`，电机加速度档位 `100`。 |
| PC1 按键 | [Agriculture.ioc](Agriculture.ioc)、[Core/Src/gpio.c](Core/Src/gpio.c)、[Core/Src/freertos.c](Core/Src/freertos.c) | 内部上拉，按下接地。`START_BUTTON_ACTIVE` 为 `GPIO_PIN_RESET`，消抖 `30 ms`，`START_DELAY_MS` 为 `3000 ms`。接线电平改动时同步核对 GPIO 上下拉和有效电平。 |

## 重点：按屏幕的 YAW 设置陀螺仪 PID 目标

1. 让车以平时的起始朝向上电，保持静止至少 1 秒，等 LCD 的 `YAW` 从 `CAL` 变成数值。本次静止校准定义了航向 `0°`。如果每次都以期望行驶朝向上电，当前默认目标 `0.0°` 即可。
2. 校准结束后，先别按 PC1。把车头摆到希望整条路线保持的朝向，等读数稳定，记下 LCD 上的 `YAW`。例如希望比开机朝向左偏约 `1.4°`，屏幕显示 `358.6`，目标值就写 `358.6f`。
3. 在 [Core/Src/freertos.c](Core/Src/freertos.c) 的 `USER CODE BEGIN PD` 区域加入下面这个常量。当前版本把目标 `0°` 写在误差公式里，第一次需要同时改下一步的公式。

   ```c
   #define CHASSIS_YAW_TARGET_DEG 358.6f  /* 填期望朝向对应的 LCD YAW */
   ```

4. 在同一个文件的 `chassis_drive_leg()` 里找到 `float yaw_error = yaw > 180.0f ? 360.0f - yaw : -yaw;`，将这一行替换为以下代码。后面的死区和 PID 计算照常执行。

   ```c
   float yaw_error = CHASSIS_YAW_TARGET_DEG - yaw;
   if (yaw_error > 180.0f) yaw_error -= 360.0f;
   if (yaw_error < -180.0f) yaw_error += 360.0f;
   ```

5. 重新编译、烧录，并以相同的起始朝向上电。以后更改目标角，只需改 `CHASSIS_YAW_TARGET_DEG`，数值使用 LCD 的 `0.0–359.9°` 范围。例如目标 `358.6°`、当前 `0.3°`，最短修正角是向左 `1.7°`。

LCD 显示的是**测量角度**。PID 目标是以**本次开机朝向**为零点的相对角度，起始摆车角度一变，同一个目标数值对应的场地朝向也会变。航向目标修改后会应用到 `case 0–11`。

## 本次固件改动

- **ICM42688 和显示**：新增 [icm42688.c](Core/Src/icm42688.c) 和 [icm42688.h](Core/Inc/icm42688.h)。I2C3 使用 `PA8/SCL` 和 `PC9/SDA`，开启内部上拉，配置为标准模式 `80 kHz`，兼容 `0x69` 与 `0x68` 两个 7 位地址。按 STC 主板的 `D9_ICM42688` I²C 例程，启用低噪声模式，陀螺仪量程 `±500°/s`，传感器输出频率 `1 kHz`。上电静止采样约 1 秒，用这段时间的角度变化估算零偏。`IMU_Task` 每 `10 ms` 读取一次 Z 轴角速度，扣除零偏并低通滤波，再积分得到 YAW。LCD 右下角显示一位小数的 YAW。
- **启动流程**：移除上电 5 秒自动行走。PC1 稳定按下后延时 3 秒，在 YAW 有效时开始路线。每段完成后停车等待 `700 ms`。
- **纠偏**：航向环每 `20 ms` 更新，激光位置环每 `40 ms` 更新。`case 0` 用 `Front` 产生前后修正，前进和后退段用 `Right` 产生侧移修正。后续左移段使用航向修正。航向、前后和侧移分量由 `ZDT_Chassis_SetVelocity()` 合成为四轮速度，经 UART5 同步下发。
- **路线和减速**：`case 0` 向右至 `Right ≤ 80 mm` 且 `Back ≥ 200 mm`，`case 1` 后退至 `Back ≤ 80 mm`。两段以 `200 RPM` 快速接近，提前降到 `75 RPM`，达到各自停车条件后停。`case 2–11` 以 `75 RPM` 逐垄往返，左移各段的右距终点依次为 `775 / 1455 / 2130 / 2900 / 3540 mm`。具体停车基准和辅助条件都在 `s_route`。
- **塔台与底盘交接**：进入后续遍历段后，符合视觉条件时塔台请求作业。底盘先停车，塔台再执行动作，完成后底盘继续。作业臂前后轴（地址 6）的 `TOWER_ARM_SPEED` 在 [Core/Inc/tower_motor.h](Core/Inc/tower_motor.h) 中设为 `75 RPM`。
- **传感器状态**：YAW 还没有有效角度时，屏幕显示下表中的状态。行驶期间若 YAW 或当前路段的主停车激光超时，底盘进入停车待机，LCD 显示 `State = -1`。若所有停车条件一直未满足，即使传感器持续出数，当前代码也没有整段行驶超时。

| YAW 栏 | 含义及检查位置 |
| --- | --- |
| `INIT` / `CAL` | 正在启动或采集零偏。保持车体静止，校准完成后会显示数值。 |
| `SCL0` | 控制器释放引脚后，`PA8/SCL` 仍为低。检查 SCL 是否接错、短路或被模块拉低。 |
| `SDA0` | 控制器尝试总线恢复后，`PC9/SDA` 仍为低。检查 SDA 接线、模块供电和外部上拉。 |
| `BUSY` | 两条线均为高，复位 I2C3 后控制器仍报告总线忙。 |
| `Bxxxx` | 复位重试后仍有 HAL 总线错误。`B0020` 表示超时，`B0200` 表示控制器未能发出 START。 |
| `NACK` | 读取 `0x69` 和 `0x68` 上的芯片标识时都收到未应答。检查供电、共地，以及模块的 `CS` 是否保持在 `VDDIO` 电平以使用 I²C。 |
| `ID` / `CFG` | 有应答但芯片标识与 `0x47` 不符，或传感器寄存器配置失败。分别检查模块型号和总线连接。 |
| `STILL` | 静止校准期间角速度波动或零偏超限。保持静止，固件会重试。 |
| `DATA` | 陀螺仪样本无效或读数超时。停车状态下固件会重新尝试初始化和校准。 |

SCL、SDA 的上拉应接到与传感器 `VDDIO` 相同的逻辑电源。模块没有板载上拉时，两根信号线都需要外部上拉。 [ICM-42688-P 数据手册](https://invensense.tdk.com/wp-content/uploads/2022/12/DS-000347-ICM-42688-P-v1.7.pdf)

识别陀螺仪前会检查总线空闲电平。如果 SCL 可以释放、SDA 被拉低，固件会尝试发最多 9 个恢复时钟，再复位 I2C3 并重试识别。若仍显示 `SCL0` 或 `SDA0`，可先断电拔掉陀螺仪模块，再给主板上电。此时显示 `NACK` 说明主板端两根线已释放，可继续排查模块和线束。 [I²C 总线规范，Bus clear](https://www.nxp.com/docs/en/user-guide/UM10204.pdf)

## 编译与验证

在工程根目录运行 `mingw32-make -j8`，固件生成在 `build/Agriculture.bin`。DAP-Link 烧录使用 `mingw32-make download_dap`。

本次固件已编译通过，模拟验证覆盖静止零偏、YAW 跨零、两段减速、组合停车条件和 IMU 数据丢失。实车还需核对传感器方向、旋转轮序及减速距离。
