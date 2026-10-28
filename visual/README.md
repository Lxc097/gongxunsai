# 视觉与上位机核心代码

- `jetson/camera/`：UVC 相机与二维码识别。
- `jetson/lidar/`、`localization/`：雷达障碍物检测、边界定位与 IMU 融合。
- `jetson/control/`、`planning/`、`runtime.py`、`main.py`：动态重规划、底盘跟踪与主程序。
- `jetson/hardware/`：F407、OpenMV、IMU、STP-23L 通信。
- `jetson/mission/`：任务码、任务计划、状态机及执行器接口。
- `path_planning/`：比赛场地、A* 路径规划与通信协议。
- `openmv/`：物料颜色/中心识别及同心圆定位。

## Jetson

使用 Python 3.10 或更高版本。从本目录运行，保持现有 Python 包名和相互导入关系：

```bash
cd visual
python3 -m pip install -r requirements-jetson.txt
python3 -m jetson.main --plan-only
```

`--plan-only` 只检查配置与规划，不打开硬件。实际导航入口为 `python3 -m jetson.main`；此前须完成 `jetson/config/vehicle.json` 的串口、相机、传感器和底盘参数配置及实机标定。雷达还需要在 Jetson 安装官方 YDLIDAR SDK 的 `ydlidar` Python 扩展。

离线场地规划在本目录执行 `python3 path_planning/path_planning.py`。`python3 -m path_planning.path_planning` 会因包初始化已经载入该模块而打印 RuntimeWarning。

## OpenMV

将 `openmv/terminal_vision.py` 作为 OpenMV 的 `main.py`，将 `openmv/concentric_circle.py` 放在板上同目录，使用 OpenMV IDE 标定 LAB 阈值、ROI、像素/毫米比例和夹爪中心。

导航/串口协议见 [PROTOCOL.md](path_planning/PROTOCOL.md)。完整搬运任务尚未完成整车接入，详见仓库首页当前状态。
