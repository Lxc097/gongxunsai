# Jetson—F407 动态地图串口协议

实车使用控制板 H27 排线的 USART6：H27-3=`MCU_TX6/PC6`，
H27-4=`MCU_RX6/PC7`，H27-2=GND。串口为 `115200-8-N-1`，逻辑电平为
3.3 V；H27-1 是板载 5 V，不接 Jetson UART 电源。

## 帧格式

```text
A5 5A | version:u8 | type:u8 | sequence:u16 | payload_length:u16 | payload | crc16:u16
```

- 多字节整数均为小端序，当前版本为 `1`。
- CRC-16/CCITT-FALSE：poly=`0x1021`、init=`0xFFFF`，覆盖 `version` 到
  `payload`，不含帧头与 CRC 字段。
- 本文的会话握手/故障锁定语义对应 2026-08-26 更新后的 F407；旧固件及
  F103 不保证这些语义。必须同步更新 Jetson 与 F407 后再联调。

| 方向 | 类型 | 值 | 载荷 |
| --- | --- | --- | --- |
| Jetson→F407 | `CHASSIS_VELOCITY` | `0x02` | `x:i16, y:i16, z:i16, watchdog_ms:u16` |
| Jetson→F407 | `CHASSIS_STOP` | `0x03` | 空 |
| F407→Jetson | `ACK` | `0x80` | `code:u8, execution_state:u8` |
| F407→Jetson | `NACK` | `0x81` | `reason:u8, execution_state:u8` |

速度轴与复刻工程的 `chassis_RC()` 一致：x 向前，y 向左，z 逆时针；
每轴范围 `-100..100`。看门狗允许 `50..1000 ms`，现场默认 300 ms。
## 会话、反馈与故障锁定

1. F407 上电不接受速度，Jetson 必须先发送 STOP 并收到 `ACK/WAITING`。
2. 每条速度使用新序号并等待 `ACK/RUNNING`；正常 STOP 返回 `ACK/WAITING`。
   ACK code：0=接受，1=重复。状态：0=WAITING，2=RUNNING，5=FAILED。
   RUNNING 仅表示命令已接受，不是电机实际转动的反馈。
3. 速度序号按 uint16 前进（差值 1～32767），支持 65535→0；同序号且同载荷
   仅回 ACK，不重设速度、不刷新看门狗。过期序号、同序号异载荷被拒绝。
4. 显式 STOP 无论原底盘模式如何，都接管到底盘动态模式并清零目标；
   STOP 允许任何序号，建立新的序号基准。不自动恢复上一次速度。
5. 看门狗到期、缓冲溢出、半帧超过 100 ms，以及 CRC/版本/长度/速度/类型
   错误立即清零并锁定；锁定后速度被 NACK，只有新的有效 STOP 可以重建会话。
   看门狗在处理接收队列前检查，已排队速度不能掩盖过期。
   2026-08-27 增加电机故障持久锁：电机总线 TX 失败，或 MODBUS 应答/状态/
   配置检查失败后，STOP 仍会清目标，但返回 `ACK/FAILED`，不能解除电机故障。
   后续速度返回 `NACK/6/FAILED`；排障后需重启 MCU，不能自动继续上次运动。
6. Jetson 实际串口默认要求 ACK，逐帧有界等待（默认 120 ms，必须小于看门狗）；
   验证 CRC、版本、序号和状态，忽略其他序号反馈。一次超时、NACK、错误状态、
   短写或串口异常就锁定，尝试发 STOP，后续不再发送速度、不自动重传运动。
   主机 STOP 不清除主机故障锁，需排查后重新启动会话。断线时 STOP 不保证送达，
依赖 MCU 看门狗，必须实测。

NACK：1=CRC，2=版本，3=不支持类型，4=长度，5=非法速度/看门狗，
6=会话未就绪/序号非法。溢出、半帧超时和看门狗无可靠待回复序号，直接锁定停车。

`DriverLink.statistics` 记录发送/确认/超时/拒绝/忽略反馈/IO 错误；退出时写入日志。
`require_ack=False` 仅用于离线内存流或测试，实车应使用 `open_serial()`。

这里的 STOP 是**底盘停车**，不是已验证的塔吊/夹爪急停。
默认 `DYNAMIC_MAP_NAVIGATION_ONLY=1` 隔离旧自动任务、塔吊初始化及遥控处理；
在真实机械臂反馈和安全中止接口完成之前，不得解除隔离用于整车赛程。

`ACTION_BATCH (0x01)` 编解码仅保留为离线/历史工具；当前 F407 和 F103
源码均拒绝动作批。`path_planning.py --serial-port` 已在打开串口前明确报错，
实车实时行驶使用 `python -m jetson.main`，离线 JSON/C 导出不受影响。

## 代码对应

- Jetson 编解码：`path_planning/path_protocol.py`
- Jetson 串口发送：`jetson/hardware/driver_link.py`
- F407 接收与看门狗：`driver/USER_Code/dynamic_map_link.c`
- F407 底盘执行：`driver/RTOS_APP/Chassis_task.c`
