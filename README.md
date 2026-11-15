# Agriculture —— 四轮麦轮底盘控制程序

基于 **STM32F407VET6 + FreeRTOS** 的四轮麦克纳姆轮底盘，用于农业场景下的「贴垄行驶 + 激光测距定位」作业。底盘按状态机依次执行一系列平移动作，每个动作由三路激光测距模块触发停车，并用 PID 做行驶方向的偏差纠正。

---

## 1. 硬件组成

| 部件 | 型号 / 说明 | 接口 |
|------|------------|------|
| 主控 | STM32F407VET6，168MHz（HSI + PLL） | — |
| 电机 | 4 × 张大头 ZDT X42S 闭环步进电机 | UART5（单总线，只发不收） |
| 激光测距 | 3 × 单线红外激光模块（MINIF / DMP0501C1） | USART1 / USART2 / USART3 |
| 显示屏 | 1.54 寸 SPI 液晶，240×240 | SPI3 |
| 下载/调试 | DAP-Link 或 J-Link | SWD |

### 串口分配

| 串口 | 引脚 | 用途 | 方向 |
|------|------|------|------|
| UART5 | PC12(TX) / PD2(RX) | 4 个 ZDT 电机（地址 1~4） | 只发 |
| USART1 | PA9(TX) / PA10(RX) | **后侧**激光测距 | 只收 |
| USART2 | PA2(TX) / PA3(RX) | **右侧**激光测距 | 只收 |
| USART3 | PB10(TX) / PB11(RX) | **前侧**激光测距 | 只收 |

激光模块接线时注意：模块的 **TX 接 MCU 的 RX**（PA10 / PA3 / PB11），波特率 115200 8N1。

### 电机地址与安装朝向

| 电机 | 编号 | 从机地址 | 麦轮朝向 |
|------|------|---------|---------|
| 左前 LF | `MOTOR_LF` | 1 | 左上 45° |
| 右前 RF | `MOTOR_RF` | 2 | 右上 45° |
| 左后 LR | `MOTOR_LR` | 3 | 右上 45° |
| 右后 RR | `MOTOR_RR` | 4 | 左上 45° |

四个电机共用一条 UART5 总线，写命令为**只发不收**（DMA 发送、忽略应答）。多机同步通过 `snF=1` 缓存 + `0xFF 0x66` 广播触发实现，保证四轮同时启动/变速。

---

## 2. 目录结构

```
Agriculture/
├── Core/
│   ├── Inc/                  # 头文件
│   │   ├── zdt_motor.h       # 底盘运动与 PID 参数（最常改的文件）
│   │   ├── laser.h           # 三路激光测距驱动
│   │   ├── pid.h             # 通用位置式 PID
│   │   └── lcd_spi_154.h     # LCD 驱动
│   └── Src/
│       ├── main.c            # 时钟、外设初始化、启动调度器
│       ├── freertos.c        # ★ 任务与底盘状态机（主要业务逻辑）
│       ├── zdt_motor.c       # 电机协议、整车运动、激光纠偏
│       ├── laser.c           # 激光串口解析状态机
│       ├── pid.c             # PID 实现
│       └── usart.c           # 串口初始化（CubeMX 生成）
├── Drivers/                  # STM32 HAL + CMSIS
├── Middlewares/Third_Party/FreeRTOS/
├── others/                   # 参考工程与资料（不参与编译，已被 .gitignore 忽略）
├── Agriculture.ioc           # CubeMX 工程配置
├── Makefile                  # GCC 构建脚本
├── STM32F407xx_FLASH.ld      # 链接脚本
└── openocd_dap.cfg / openocd_jlink.cfg
```

---

## 3. 编译与下载

### 依赖

- `arm-none-eabi-gcc` 工具链（需在 `PATH` 中，或用 `make GCC_PATH=...` 指定路径）
- `make`
- `openocd`（DAP-Link 下载）或 J-Link 工具（JFlash）

### 命令

```bash
make                 # 编译，产物在 build/（.elf / .hex / .bin）
make clean           # 清理 build/（注意：内部用的是 Windows 的 rd 命令）
make download_dap    # 用 DAP-Link 通过 openocd 烧录
make download_jlink  # 用 J-Link 烧录
```

也可以在 VS Code 里用 `.vscode/tasks.json` / `.vscode/launch.json` 配置好的任务和调试会话。

---

## 4. 软件架构

### 任务

| 任务 | 优先级 | 职责 |
|------|--------|------|
| `Chassis_Task` | BelowNormal1 | 底盘状态机：调用阻塞式的运动函数，逐个执行动作 |
| `Data_Task` | Normal | 50Hz 读取三路激光，刷新共享变量，在 LCD 上显示 |

共享变量（`freertos.c` 顶部）：`g_front_mm` / `g_right_mm` / `g_back_mm` 及对应的 `*_valid` 标志，由 `Data_Task` 写入、其他任务只读。

LCD 显示内容：

```
Front: 1234
Right:  567
Back:  8901
```

### 底盘状态机（`StartChassis_Task`）

上电后 `ZDT_Motor_Init()` + `Laser_Init()`，**等待 5 秒**（等激光出数稳定）再开始动作。

| state | 动作 | 纵向纠偏 | 停车条件 |
|-------|------|---------|---------|
| 0 | 向右平移 | front，目标 40mm | right ≤ 3595mm |
| 1 | 向后直行 + 横向居中 | — | back ≤ 50mm |
| 2 | 向右平移 | back，目标 75mm | right ≤ 2895mm |
| 3 | 向前直行 + 横向居中 | — | front ≤ 50mm |
| 4 | 向右平移 | front，目标 40mm | right ≤ 2195mm |
| 5 | 向后直行 + 横向居中 | — | back ≤ 50mm |
| 6 | 向右平移 | back，目标 75mm | right ≤ 1495mm |
| 7 | 向前直行 + 横向居中 | — | front ≤ 50mm |
| 8 | 向右平移 | front，目标 40mm | right ≤ 795mm |
| 9 | 向后直行 + 横向居中 | — | back ≤ 50mm |
| 10 | 向右平移 | back，目标 75mm | right ≤ 95mm |
| 11 | 向前直行 + 横向居中 | — | front ≤ 50mm |
| 12 | 向左平移 4200mm | front，目标 40mm | 行驶距离到达 |
| 默认 | 停车待机 | — | — |

每完成一个 state 停车并等待 500ms，然后进入下一个。

### 三个运动函数

| 函数 | 用途 | 停止方式 |
|------|------|---------|
| `ZDT_Chassis_Drive_LatCenter(dir, rpm, lon_laser, stop_mm)` | 左右平移 + **纵向**激光居中（前或后激光） | 右侧激光 ≤ `stop_mm` |
| `ZDT_Chassis_Drive_LaserCenter(dir, rpm, stop_laser, until_le, stop_mm)` | 前后直行 + **横向**激光居中（右激光） | 指定激光满足阈值 |
| `ZDT_Chassis_Drive_LongCenter(dir, rpm, distance_mm)` | 左右平移 + 纵向居中（固定用前激光） | 行驶距离到达 |

三者都是**阻塞式**：内部循环到触发条件才返回。控制周期 33ms，与激光 30Hz 出数频率匹配。

---

## 5. 参数标定（`Core/Inc/zdt_motor.h`）

改参数基本只需要动这一个文件。

### 机械参数

| 宏 | 当前值 | 说明 |
|----|--------|------|
| `ZDT_WHEEL_DIAMETER_MM` | 77 | 轮子外径，**务必按实测修改** |
| `ZDT_GEAR_RATIO` | 1.0 | 电机轴直连轮子填 1.0 |
| `ZDT_PULSES_PER_REV` | 3200 | 16 细分时一圈脉冲数 |
| `ZDT_TURN_RADIUS_MM` | 135 | 原地旋转时底盘中心到轮子的水平距离 |
| `ZDT_DEFAULT_SPEED_RPM` | 100 | 默认转速（12V 供电建议 ≤200） |
| `ZDT_DEFAULT_ACC` | 0 | 0 = 不用加减速曲线，直接启动 |

### 横向 PID（右侧激光 → 横移分量）

| 宏 | 当前值 | 说明 |
|----|--------|------|
| `ZDT_LAT_TARGET_MM` | **52.5** | 目标墙距（原 45mm，再 +7.5mm） |
| `ZDT_LAT_RANGE_MIN/MAX_MM` | 20 / 400 | 有效墙距范围，超范围视为无墙、开环直行 |
| `ZDT_LAT_KP/KI/KD` | 1.0 / 0 / 0 | 增益（目前只用 P） |
| `ZDT_LAT_MAX_RPM` | 60 | 修正量限幅 |
| `ZDT_LAT_DIR_INVERT` | 0 | 方向反了就改 1 |

### 纵向 PID（前/后激光 → 前后分量）

| 宏 | 当前值 | 说明 |
|----|--------|------|
| `ZDT_LON_TARGET_MM` | **40** | 目标墙距（前侧激光，state 0/4/8/12） |
| `ZDT_LON_TARGET_BACK_MM` | **75** | 目标墙距（后侧激光 back 作参考时） |
| `ZDT_LON_RANGE_MIN/MAX_MM` | 20 / 400 | 有效墙距范围 |
| `ZDT_LON_KP/KI/KD` | 1.0 / 0 / 0 | 增益 |
| `ZDT_LON_MAX_RPM` | 60 | 修正量限幅 |
| `ZDT_LON_DIR_INVERT` | 1 | 同时反转前侧和后侧的纵向纠偏方向 |

> **注意**：纵向 PID 对「前侧」和「后侧」激光在误差计算上做了符号区分（车太近时，用前激光该后退、用后激光该前进），因此 `ZDT_LON_DIR_INVERT` 会把两者的方向**一起**反转。若只出现单侧方向反了，需要改代码而不是这个开关。

---

## 6. 已知问题与注意事项

1. **CubeMX 重新生成会重复创建 `Data_Task`**
   - `.ioc` 里配置了 `Data_Task`，但生成的 `freertos.c` 曾缺少线程注册，已手动在 `RTOS_THREADS` 的 USER 区补上。
   - 若再次用 CubeMX 生成代码，CubeMX 会自动生成 `osThreadNew(StartData_Task, ...)`，与手动那份**重复** → 会同时跑两个 DataTask，需要删掉手动那份。
   - 命名也不一致：CubeMX 生成的是 `Data_Task`，手动补的是 `DataTask`，注意区分。

2. **运动函数没有超时保护**
   - 所有阻塞式运动函数都只靠激光触发停车。若某个激光失效（接线松、无数据），车会**一直开下去**直到撞墙。
   - 上电显示页面时，若某一路读数一直是 0 或不变化，先检查对应串口接线和激光供电。

3. **激光无效时不会误停车**
   - 驱动内 `Laser_IsValid()` 为 false 时一律视为「条件未满足、继续行驶」，避免把上电初值的 0 当成「很近」。

4. **`make clean` 依赖 Windows 的 `rd` 命令**，在纯 Linux/macOS 环境下需要改成 `rm -rf`。

5. **`others/` 目录**是参考工程与比赛资料，不参与编译，已被 `.gitignore` 忽略。

---

## 7. 扩展示例

### 新增一个动作

在 `StartChassis_Task` 的 `switch` 中加一个 `case`，然后调用对应运动函数即可：

```c
case 13:  /* 向右平移 + back 纵向居中，直到 right <= 60mm */
    ZDT_Chassis_Drive_LatCenter(MOVE_RIGHT, ZDT_DEFAULT_SPEED_RPM, LASER_BACK, 60);
    break;
```

注意 `state` 是在 `switch` 之后自增的，新加的 `case` 要放在 `default` 之前。

### 新增一路激光

1. 在 `laser.h` 的 `LaserID` 枚举里加一项；
2. 在 `laser.c` 的 `s_laser[]` 数组、`laser_uart_to_id()`、`Laser_Init()` 中补上对应串口；
3. 在 `zdt_motor.c` 的 `HAL_UART_RxCpltCallback` / `HAL_UART_ErrorCallback` 里把新串口加入分发判断。
