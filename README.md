# 工训赛智能搬运车

Jetson 负责感知、定位、路径和底盘速度指令，STM32F407 执行底盘与车上机构。仓库分成 `visual/` 和 `driver/` 两棵目录。这是当前开发代码的快照，不含单元测试、台架脚本、历史 F103/2025 工程和编译产物。

`python3 -m jetson.main` 目前做单站导航。`visual/jetson/mission/` 里有初赛任务状态机，主程序没有调用它，也没有把前置相机、OpenMV 和 STP-23L 接到这个入口。F407 默认 `DYNAMIC_MAP_NAVIGATION_ONLY` 为 1，旧自动路线、塔吊回零和遥控接管留在条件编译的另一支。默认电机协议是 `ZDT_EMM_CAN`，`ZDT_CAN_COMMISSIONED` 为 0，底盘初始化会关闭电机电源开关。这些宏保持原样。

`path_planning.py` 开头写的赛项是「河北省大学生工程实践与创新能力大赛“智能搬运”」。本页标题沿用仓库原有的「工训赛」。两处原文都没有改。

## 目录

| 路径 | 内容 |
| --- | --- |
| `visual/jetson/main.py` | Jetson 主程序。`--plan-only` 只检查配置和规划 |
| `visual/jetson/runtime.py` | 一个控制周期：IMU、雷达、重规划、速度指令 |
| `visual/jetson/camera/` | 前置 UVC 与二维码 |
| `visual/jetson/lidar/` | YDLIDAR X3 Pro、障碍聚类、边界定位 |
| `visual/jetson/localization/` | IMU 滤波与平面位姿 |
| `visual/jetson/planning/` | 把雷达障碍送进场地 A* |
| `visual/jetson/control/` | 路径跟踪 |
| `visual/jetson/hardware/` | F407、IMU、OpenMV、STP-23L |
| `visual/jetson/mission/` | 任务码、步骤、状态机、执行器接口 |
| `visual/jetson/config/vehicle.json` | 串口、相机、雷达和运行参数 |
| `visual/path_planning/` | 2400 mm × 2400 mm 场地、A*、离线动作导出、串口帧 |
| `visual/path_planning/PROTOCOL.md` | Jetson 与 F407 的动态地图帧 |
| `visual/openmv/` | 末端颜色、中心和同心圆。`terminal_vision.py` 依赖 OpenMV 固件库 |
| `visual/requirements-jetson.txt` | PyPI 依赖，不含 YDLIDAR 官方扩展 |
| `driver/MDK-ARM/Logistics_Vehicle_F407.uvprojx` | Keil 工程 |
| `driver/Logistics_Vehicle_F407.ioc` | CubeMX 配置 |
| `driver/Core/` | 时钟、外设、中断、FreeRTOS 对象 |
| `driver/RTOS_APP/` | 底盘、输出、主流程、塔吊任务 |
| `driver/USER_Code/` | 电机、底盘、地图协议、视觉、遥控、OLED |
| `driver/Drivers/`、`driver/Middlewares/` | HAL、CMSIS、FreeRTOS |

更细的编译说明和上位机命令分别在 [driver/README.md](driver/README.md) 和 [visual/README.md](visual/README.md)。

## 软件怎么接在一起

Jetson 按 [PROTOCOL.md](visual/path_planning/PROTOCOL.md) 发送底盘速度或停车。默认物理口是 USART6：PC6 为 MCU TX（H27-3），PC7 为 MCU RX（H27-4），地为 H27-2，115200 8N1，逻辑电平 3.3 V。`vehicle.json` 写明 H27-1 是板上 5 V，不要接到 Jetson 的 UART 电源。F407 上电后先接受停车帧，应答之后才接受速度。看门狗、序号和故障锁定以协议文档和 `driver/USER_Code/dynamic_map_link.c` 为准。

离线规划可以导出 C 动作数组。当前 F407 源码拒绝 `ACTION_BATCH`。`path_planning.py --serial-port` 在打开串口之前就会退出。

## STM32 工程里有什么

芯片是 STM32F407VETx。Keil 目标名为 `Logistics_Vehicle_F407`，器件包 `Keil.STM32F4xx_DFP.2.17.1`，编译器记录为 ARM Compiler 5.06 update 7 (build 960)。

`SystemClock_Config()` 默认使用 HSE。旁边注释写整车晶振为 8 MHz，PLL 参数是 PLLM=4、PLLN=168、PLLP=2。`uvprojx` 器件栏里的 `CLOCK(25000000)` 是 Keil 的器件字段，和这段时钟代码不是同一个数。

FreeRTOS 线程：`defaultTask`、`CHASSIS_TASK`、`OUTPU_TASK`、`UART_RX_TASK`、`MAIN_TASK`、`TOWER_TASK`。`freertos.c` 对串口的注释如下。

| 串口 | 引脚 | 代码中的用途 |
| --- | --- | --- |
| USART1 | PA9 / PA10 | NANO，DMA 空闲接收，`NANO_Data_Unpack` |
| USART2 | PA2 / PA3 | OPS 坐标，DMA 空闲接收；`fputc` 也走这条口 |
| UART4 | PA0 / PA1 | 注释写底盘。`zdt_motor.c` 在非 CAN 协议下用它收发 |
| UART5 | PC12 / PD2 | 注释写遥控，DMA 空闲接收。`DYNAMIC_MAP_LINK_UART5` 为 0，默认不把动态地图切到这里 |
| USART6 | PC6 / PC7 | Jetson 动态地图 |

CAN1 在 PA11 / PA12。`Logistics_Vehicle_F407.ioc` 里 CAN1 标称 1 Mbps。轮子地址写在 `zdt_motor.h`。塔吊电机在 `USER_Code/tower/`。TIM1 通道 1 在 PE9，`tower.h` 用它做夹爪 PWM。

板上 IO 来自 `main.h` 和 `io_devices.h`：LED1 在 PB5，LED2 在 PB4，LED3 在 PB3，低电平点亮；蜂鸣器 PC0，低电平响；启动键 PD3；相机灯 PD1；电机电源开关 PD0。硬件 SPI1（PA5 SCK、PA7 MOSI，CS/DC/RST 在 PD5/PD4/PD6）驱动 `OLED_SPI`，`Output_task` 会初始化这块屏。`OLED_SoftSPI` 用软件时序，引脚沿用 SPI2 的名字：PB13、PC3、PE2、PE3、PE4。隔离打开时 `Main_task` 不会走到这块屏的初始化。

CubeMX 还初始化了 I2C2（PB10/PB11）和 TIM2。应用代码没有再使用 `hi2c2`，也没有启动 TIM2。HAL 的 1 ms 时基在 TIM4。

## 编译和烧录 STM32

用 Keil MDK 打开 `driver/MDK-ARM/Logistics_Vehicle_F407.uvprojx`，构建目标 `Logistics_Vehicle_F407`。工程打开了 HEX 输出，目录是 `driver/MDK-ARM/Logistics_Vehicle_F407/`，文件名 `Logistics_Vehicle_F407.hex`。该目录已在 `.gitignore` 中。

仓库没有烧录脚本，也没有提交 `.uvoptx`。下载使用 Keil 的 Download，具体调试器以本机工程选项为准。`uvprojx` 的 Flash 项使用 Target DLL（`DriverSelection` 4101）。用 CubeMX 重新生成代码时，需要对照 `USER CODE` 段以及 `USER_Code/`、`RTOS_APP/` 里已有的改动。

## 运行 Jetson

Python 3.10 或更高。在 `visual/` 下运行，以保持 `jetson` 和 `path_planning` 的包导入关系。

```bash
cd visual
python3 -m pip install -r requirements-jetson.txt
python3 -m jetson.main --plan-only
```

`--plan-only` 读取 `jetson/config/vehicle.json`，用内存流规划一次，不打开串口、相机和雷达。已在本仓库用 Python 3.12 跑通这条命令。实车入口：

```bash
python3 -m jetson.main
```

常用参数：`--goal`（站点名来自 `path_planning.py` 的 `STATIONS`）、同时给出的 `--goal-x` 与 `--goal-y`、`--target-yaw`、`--config`、`-v`。实车前在 `vehicle.json` 填写 F407、IMU 和 YDLIDAR 的串口，三个口不能相同。雷达需要在 Jetson 上单独安装官方 YDLIDAR SDK 的 `ydlidar` 扩展。

离线场地规划：

```bash
cd visual
python3 path_planning/path_planning.py --pretty
```

请直接运行脚本。`python3 -m path_planning.path_planning` 会打印 RuntimeWarning，因为 `path_planning/__init__.py` 已经导入了这个模块；规划仍会输出。

默认日志是 `visual/jetson/logs/runtime.log`。配置里的 `jetson/logs/runtime.log` 相对 `visual/` 解析。

## OpenMV

把 `visual/openmv/terminal_vision.py` 作为 OpenMV 上的 `main.py`，`concentric_circle.py` 放在同一目录。程序使用 `UART(3, 115200)`，每次稳定观测写一行 JSON。文件中的 LAB 阈值标明为示例，需要在 OpenMV IDE 里按车上光照标定 ROI 和比例。

## 变更记录

2026-10-28

- 补上本页：目录、板上外设和任务的对应关系、Keil 的 HEX 位置、Jetson 与 OpenMV 的启动方式。叙述只依据仓库里的源码和原有文档。
- `.gitignore` 补了编辑器与系统杂项、Keil 调试残留、CubeMX 的 `.mxproject`，以及 Python 工具缓存。已跟踪的文件里没有编译产物或编辑器临时文件，因此没有删除文件。
- 改正已经对不上的文档路径：`PROTOCOL.md` 里的 F407 源文件、`requirements-jetson.txt` 指向的 README，以及 `path_planning.py` 对 `driver/Drivers/BSP/map.h` 的说明。当前工程没有这个头文件。`driver/README.md` 不再把本目录写成不存在的 `F407_Replica_DynamicMap` 子目录。
