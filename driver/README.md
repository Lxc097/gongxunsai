# STM32F407 固件核心代码

本目录来自当前 `driver/F407_Replica_DynamicMap` 主工程，保留主工程直接引用的源文件、头文件、启动汇编和必要依赖。

## 编译

使用 Keil MDK / ARM Compiler 5，打开 `MDK-ARM/Logistics_Vehicle_F407.uvprojx` 并构建 `Logistics_Vehicle_F407` 目标。工程声明 STM32F407VET6，使用 STM32F4xx DFP 2.17.1。`Logistics_Vehicle_F407.ioc` 为 CubeMX 配置；若重新生成代码，需核对现有用户代码和依赖。

## 模块

- `Core/`：时钟、外设、中断和 FreeRTOS 初始化。
- `RTOS_APP/`：底盘、输出及现有塔吊/流程任务。
- `USER_Code/`：动态地图协议、电机、底盘、塔吊、视觉通信、遥控和 OLED。
- `Drivers/`、`Middlewares/`：HAL、CMSIS、FreeRTOS 必要源文件、头文件与许可证。

仅保留主工程，移除了独立维护桥接程序及其入口分支、维护/静态台架工程、测试目录与编译产物。正常 RTOS 分支及已有导航隔离、电机验收门控保持原有逻辑。仍被主工程引用的任务源文件作为依赖保留，其存在不代表已适配完整本届赛程。

Jetson 链路为 USART6 / PC6 TX / PC7 RX，115200-8-N-1。详细协议见 [上位机通信协议](../visual/path_planning/PROTOCOL.md)。
