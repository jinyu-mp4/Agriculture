# Agriculture 底盘与 ICM42688 调试说明

## 当前底盘模式

底盘已恢复到 `c1d2175` 的三路激光开环路线。**上电等待 5 秒后自动走车**。各段使用 `ZDT_DEFAULT_SPEED_RPM`（`50 RPM`）和 `ZDT_DEFAULT_ACC`（`100`），按原激光组合条件停车，每段之间等待 `700 ms`。

| 调试项目 | 工程位置 |
| --- | --- |
| 12 段路线、启动延时和各段停车条件 | [Core/Src/freertos.c](Core/Src/freertos.c) 的 `StartChassis_Task()` |
| 底盘速度和加速度 | [Core/Inc/zdt_motor.h](Core/Inc/zdt_motor.h) 的 `ZDT_DEFAULT_SPEED_RPM`、`ZDT_DEFAULT_ACC` |
| YAW 左右方向 | [Core/Src/icm42688.c](Core/Src/icm42688.c) 的 `ICM42688_YAW_SIGN`，Z 轴向上时车头右转 1° 应显示约 `1.0°`，左转 1° 应显示约 `359.0°` |

塔台仍可在后续遍历段触发。底盘确认停车后，塔台接管 UART5，完成后底盘继续当前路段。作业臂前后轴（地址 6）速度为 `75 RPM`。

## 陀螺仪显示与故障诊断

ICM42688 继续通过 I2C3 采样并在 LCD 右下角显示 YAW。上电静止校准后的方向为 `0°`。当前底盘按激光条件运行，YAW 用于观察航向，屏幕显示的总线状态可独立排查。

I2C3 使用 `PA8/SCL` 和 `PC9/SDA`，开启内部上拉并配置为 `80 kHz`，兼容 `0x69` 与 `0x68` 两个地址。驱动在 [Core/Src/icm42688.c](Core/Src/icm42688.c)，CubeMX 配置在 [Agriculture.ioc](Agriculture.ioc)。`others/ICM42688资料包-iic/` 和 `others/STC32G12K128主板DEMOV1.2/D9_ICM42688/` 为参考例程。

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

陀螺仪诊断已通过模拟总线故障测试。实车仍需核对接线和传感器方向。
