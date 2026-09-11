#!/usr/bin/env python3
"""Jetson vehicle entry point.

The main thread is the sole owner of ``VehicleRuntime``.  Sensor workers only
perform blocking I/O and publish timestamped samples, which keeps planning and
control deterministic and makes every failure converge on a chassis stop.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
import logging
from logging.handlers import RotatingFileHandler
import math
from pathlib import Path
import queue
import signal
import sys
import threading
import time
from typing import Any, Callable, Sequence

from jetson.hardware import DriverLink, Stp23lConfig
from jetson.hardware.imu_link import ImuConfig, ImuJsonDecoder, YahboomImuDecoder
from jetson.lidar import YdlidarX3Pro
from jetson.models import ImuSample, Pose2D
from jetson.runtime import VehicleRuntime
from path_planning import STATIONS


LOGGER = logging.getLogger("jetson")
DEFAULT_CONFIG = Path(__file__).resolve().parent / "config" / "vehicle.json"


class ConfigurationError(ValueError):
    """The vehicle configuration is absent or unsafe."""


class SafetyStop(RuntimeError):
    """A runtime health rule requested an immediate latched stop."""


@dataclass(frozen=True)
class RuntimeSettings:
    control_hz: float
    startup_timeout_s: float
    imu_timeout_s: float
    lidar_timeout_s: float
    emergency_stop_mm: float
    masked_sectors: tuple[tuple[float, float], ...]
    initial_x_mm: float
    initial_y_mm: float
    initial_yaw_deg: float
    default_goal: str
    target_yaw_deg: float | None
    log_file: Path


@dataclass(frozen=True)
class VehicleConfig:
    raw: dict[str, Any]
    settings: RuntimeSettings
    f407_port: str | None
    f407_baudrate: int
    f407_watchdog_ms: int
    imu_port: str | None
    imu_baudrate: int
    lidar_port: str | None
    lidar_baudrate: int
    imu_config: ImuConfig

    def require_navigation_hardware(self) -> None:
        missing = [
            name for name, value in (
                ("f407.port", self.f407_port),
                ("imu.port", self.imu_port),
                ("ydlidar_x3_pro.port", self.lidar_port),
            ) if not value
        ]
        if missing:
            raise ConfigurationError("以下实车串口尚未配置：" + ", ".join(missing))
        ports = [self.f407_port, self.imu_port, self.lidar_port]
        if len(set(ports)) != len(ports):
            raise ConfigurationError("F407、IMU 和雷达不能复用同一个串口")
        try:
            self.imu_config.require_supported_transport()
        except ValueError as exc:
            raise ConfigurationError(str(exc)) from exc


def _mapping(parent: dict[str, Any], key: str) -> dict[str, Any]:
    value = parent.get(key)
    if not isinstance(value, dict):
        raise ConfigurationError(f"配置项 {key} 必须是对象")
    return value


def _finite_number(parent: dict[str, Any], key: str, *, positive: bool = False) -> float:
    try:
        value = float(parent[key])
    except (KeyError, TypeError, ValueError) as exc:
        raise ConfigurationError(f"配置项 {key} 必须是数字") from exc
    if not math.isfinite(value) or (positive and value <= 0.0):
        qualifier = "有限正数" if positive else "有限数字"
        raise ConfigurationError(f"配置项 {key} 必须是{qualifier}")
    return value


def _optional_port(section: dict[str, Any], name: str) -> str | None:
    value = section.get("port")
    if value is None:
        return None
    if not isinstance(value, str) or not value.strip():
        raise ConfigurationError(f"配置项 {name}.port 必须是非空字符串或 null")
    return value.strip()


def load_vehicle_config(path: Path) -> VehicleConfig:
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise ConfigurationError(f"无法读取配置文件 {path}: {exc}") from exc
    except json.JSONDecodeError as exc:
        raise ConfigurationError(f"配置文件不是合法 JSON: {exc}") from exc
    if not isinstance(raw, dict):
        raise ConfigurationError("配置文件根节点必须是对象")

    runtime = _mapping(raw, "runtime")
    initial = _mapping(runtime, "initial_pose")
    f407 = _mapping(raw, "f407")
    imu = _mapping(raw, "imu")
    try:
        imu_config = ImuConfig.from_mapping(imu)
    except (ValueError, TypeError) as exc:
        raise ConfigurationError(f"IMU 配置错误: {exc}") from exc
    lidar = _mapping(raw, "ydlidar_x3_pro")
    try:
        Stp23lConfig.from_mapping(_mapping(raw, "stp23l"))
    except (ValueError, TypeError) as exc:
        raise ConfigurationError(f"STP-23L 配置错误: {exc}") from exc

    masks: list[tuple[float, float]] = []
    mask_items = runtime.get("lidar_masked_sectors_deg", [])
    if not isinstance(mask_items, list):
        raise ConfigurationError("lidar_masked_sectors_deg 必须是数组")
    for index, item in enumerate(mask_items):
        if not isinstance(item, list) or len(item) != 2:
            raise ConfigurationError(f"雷达屏蔽扇区 #{index} 必须是 [start, end]")
        start, end = float(item[0]), float(item[1])
        if not (0.0 <= start <= end <= 360.0):
            raise ConfigurationError(f"雷达屏蔽扇区 #{index} 范围必须满足 0 <= start <= end <= 360")
        masks.append((start, end))

    default_goal = runtime.get("default_goal", "qr")
    if default_goal not in STATIONS:
        raise ConfigurationError(f"default_goal 必须是已知站点：{', '.join(sorted(STATIONS))}")
    target_yaw = runtime.get("target_yaw_deg")
    if target_yaw is not None:
        target_yaw = float(target_yaw)
        if not math.isfinite(target_yaw):
            raise ConfigurationError("target_yaw_deg 必须是有限数字或 null")

    log_value = runtime.get("log_file", "jetson/logs/runtime.log")
    if not isinstance(log_value, str) or not log_value.strip():
        raise ConfigurationError("log_file 必须是非空路径")
    log_path = Path(log_value)
    if not log_path.is_absolute():
        log_path = Path(__file__).resolve().parents[1] / log_path

    settings = RuntimeSettings(
        control_hz=_finite_number(runtime, "control_hz", positive=True),
        startup_timeout_s=_finite_number(runtime, "startup_timeout_s", positive=True),
        imu_timeout_s=_finite_number(runtime, "imu_timeout_s", positive=True),
        lidar_timeout_s=_finite_number(runtime, "lidar_timeout_s", positive=True),
        emergency_stop_mm=_finite_number(runtime, "emergency_stop_mm", positive=True),
        masked_sectors=tuple(masks),
        initial_x_mm=_finite_number(initial, "x_mm"),
        initial_y_mm=_finite_number(initial, "y_mm"),
        initial_yaw_deg=_finite_number(initial, "yaw_deg"),
        default_goal=default_goal,
        target_yaw_deg=target_yaw,
        log_file=log_path,
    )
    if settings.control_hz > 100.0:
        raise ConfigurationError("control_hz 不能高于 100 Hz")
    if settings.startup_timeout_s <= imu_config.settle_s + imu_config.calibration_duration_s:
        raise ConfigurationError("startup_timeout_s 必须大于 IMU 等待与标定总时长")
    imu_baudrate = imu.get("baudrate", 115200)
    if type(imu_baudrate) is not int or imu_baudrate <= 0:
        raise ConfigurationError("imu.baudrate 必须为正整数")
    if not 50 <= int(f407.get("watchdog_ms", 300)) <= 1000:
        raise ConfigurationError("f407.watchdog_ms 必须在 50..1000 ms")

    return VehicleConfig(
        raw=raw,
        settings=settings,
        f407_port=_optional_port(f407, "f407"),
        f407_baudrate=int(f407.get("baudrate", 115200)),
        f407_watchdog_ms=int(f407.get("watchdog_ms", 300)),
        imu_port=_optional_port(imu, "imu"),
        imu_baudrate=imu_baudrate,
        lidar_port=_optional_port(lidar, "ydlidar_x3_pro"),
        lidar_baudrate=int(lidar.get("baudrate", 115200)),
        imu_config=imu_config,
    )


def _put_latest(target: queue.Queue[Any], item: Any) -> None:
    try:
        target.put_nowait(item)
    except queue.Full:
        try:
            target.get_nowait()
        except queue.Empty:
            pass
        target.put_nowait(item)


def _is_masked(angle_deg: float, masks: Sequence[tuple[float, float]]) -> bool:
    normalized = angle_deg % 360.0
    return any(start <= normalized <= end for start, end in masks)


def nearest_unmasked_range(
    angles_deg: Sequence[float],
    ranges_mm: Sequence[float],
    masks: Sequence[tuple[float, float]],
) -> float | None:
    values = [
        distance for angle, distance in zip(angles_deg, ranges_mm)
        if math.isfinite(distance) and distance > 30.0 and not _is_masked(angle, masks)
    ]
    return min(values, default=None)


class SensorWorker:
    def __init__(
        self,
        name: str,
        target: Callable[[threading.Event], None],
        stop_event: threading.Event,
        errors: queue.Queue[tuple[str, BaseException]],
    ) -> None:
        self.name = name
        self._target = target
        self._stop_event = stop_event
        self._errors = errors
        self._thread = threading.Thread(target=self._run, name=name, daemon=True)

    def _run(self) -> None:
        try:
            self._target(self._stop_event)
        except BaseException as exc:
            self._errors.put((self.name, exc))
            self._stop_event.set()

    def start(self) -> None:
        self._thread.start()

    def join(self, timeout: float = 2.0) -> None:
        self._thread.join(timeout)


class JetsonApplication:
    def __init__(self, config: VehicleConfig, goal: tuple[float, float], target_yaw_deg: float | None) -> None:
        self.config = config
        self.goal = goal
        self.target_yaw_deg = target_yaw_deg
        self.stop_event = threading.Event()
        self.errors: queue.Queue[tuple[str, BaseException]] = queue.Queue()
        self.imu_samples: queue.Queue[ImuSample] = queue.Queue(maxsize=128)
        self.lidar_scans: queue.Queue[tuple[list[float], list[float], float]] = queue.Queue(maxsize=1)
        self.workers: list[SensorWorker] = []
        self.driver: DriverLink | None = None
        self.lidar: YdlidarX3Pro | None = None
        self.imu_stream: Any | None = None
        self._imu_calibrated = False
        self._imu_first_sample_s: float | None = None
        self._imu_calibration_samples: list[ImuSample] = []
        self._closed = False

    def request_stop(self) -> None:
        self.stop_event.set()

    def _open_hardware(self) -> None:
        self.config.require_navigation_hardware()
        self.driver = DriverLink.open_serial(
            self.config.f407_port or "",
            baudrate=self.config.f407_baudrate,
            watchdog_ms=self.config.f407_watchdog_ms,
        )
        # Stop before bringing perception online; no stale chassis command may
        # survive a Jetson process restart.
        self.driver.stop()
        try:
            import serial
        except ImportError as exc:
            raise RuntimeError("IMU serial link requires pyserial") from exc
        self.imu_stream = serial.Serial(
            self.config.imu_port,
            baudrate=self.config.imu_baudrate,
            timeout=0.1,
        )
        self.imu_stream.reset_input_buffer()
        self.lidar = YdlidarX3Pro(
            self.config.lidar_port or "",
            baudrate=self.config.lidar_baudrate,
        )
        self.lidar.start()

    def _imu_loop(self, stop_event: threading.Event) -> None:
        assert self.imu_stream is not None
        native = self.config.imu_config.protocol == "yahboom_native"
        decoder = YahboomImuDecoder() if native else ImuJsonDecoder()
        while not stop_event.is_set():
            # Both decoders bound their framing state and survive split reads.
            data = self.imu_stream.read(max(1, min(self.imu_stream.in_waiting, 4096)))
            if not data:
                continue
            rejected_name = "rejected_frames" if native else "rejected_lines"
            previous_rejected = getattr(decoder, rejected_name)
            sample = decoder.feed(data, time.monotonic())
            if sample is not None:
                _put_latest(self.imu_samples, sample)
            if previous_rejected == 0 and getattr(decoder, rejected_name):
                LOGGER.warning("IMU 串口出现校验错误、无效帧或重复数据；持续无新样本将停车")

    def _calibrate_imu(self, runtime: VehicleRuntime, sample: ImuSample) -> bool:
        config = self.config.imu_config
        if self._imu_first_sample_s is None:
            self._imu_first_sample_s = sample.timestamp_s
        if sample.timestamp_s - self._imu_first_sample_s < config.settle_s:
            return False
        samples = self._imu_calibration_samples
        if samples and not 0 < sample.timestamp_s - samples[-1].timestamp_s <= config.max_gap_s:
            samples.clear()
        samples.append(sample)
        if len(samples) > 2000:
            del samples[0]
        if len(samples) < 20 or sample.timestamp_s - samples[0].timestamp_s < config.calibration_duration_s:
            return False
        try:
            bias = runtime.imu.calibrate_stationary(samples)
        except ValueError as exc:
            LOGGER.warning("IMU 静止标定未通过: %s", exc)
            samples.clear()
            return False
        samples.clear()
        self._imu_calibrated = True
        LOGGER.info("6 轴 IMU 标定完成: gyro bias=%.4f deg/s, 初始场地航向=%.1f deg",
                    bias, self.config.settings.initial_yaw_deg)
        return True

    def _lidar_loop(self, stop_event: threading.Event) -> None:
        assert self.lidar is not None
        while not stop_event.is_set():
            angles, ranges = self.lidar.read_scan()
            if angles:
                _put_latest(self.lidar_scans, (angles, ranges, time.monotonic()))

    def _start_workers(self) -> None:
        self.workers = [
            SensorWorker("imu-reader", self._imu_loop, self.stop_event, self.errors),
            SensorWorker("lidar-reader", self._lidar_loop, self.stop_event, self.errors),
        ]
        for worker in self.workers:
            worker.start()

    def _raise_worker_error(self) -> None:
        try:
            name, exc = self.errors.get_nowait()
        except queue.Empty:
            return
        raise SafetyStop(f"{name} 异常退出: {exc}") from exc

    def _consume_sensors(self, runtime: VehicleRuntime) -> tuple[float | None, float | None]:
        last_imu: float | None = None
        events = []
        while True:
            try:
                sample = self.imu_samples.get_nowait()
            except queue.Empty:
                break
            events.append((sample.timestamp_s, "imu", sample))

        last_lidar: float | None = None
        while True:
            try:
                angles, ranges, timestamp = self.lidar_scans.get_nowait()
            except queue.Empty:
                break
            nearest = nearest_unmasked_range(angles, ranges, self.config.settings.masked_sectors)
            if nearest is not None and nearest < self.config.settings.emergency_stop_mm:
                raise SafetyStop(f"雷达近障 {nearest:.0f} mm，小于停车阈值")
            events.append((timestamp, "lidar", (angles, ranges)))
        # Merge the two queues: draining all IMU first makes queued lidar stale.
        for timestamp, kind, data in sorted(events, key=lambda event: event[0]):
            if kind == "imu":
                if not self._imu_calibrated and not self._calibrate_imu(runtime, data):
                    continue
                try:
                    runtime.ingest_imu(data)
                except ValueError as exc:
                    raise SafetyStop(f"IMU 姿态异常: {exc}") from exc
                # Only a fused observation counts as fresh, not a discarded old one.
                if runtime.imu.valid and runtime.imu.last_timestamp_s == timestamp:
                    last_imu = timestamp
            else:
                runtime.ingest_lidar(*data, timestamp, masked_sectors=self.config.settings.masked_sectors)
                last_lidar = timestamp
        return last_imu, last_lidar

    def run(self) -> int:
        self._open_hardware()
        assert self.driver is not None
        now = time.monotonic()
        settings = self.config.settings
        runtime = VehicleRuntime(Pose2D(
            settings.initial_x_mm,
            settings.initial_y_mm,
            settings.initial_yaw_deg,
            now,
        ), self.driver, imu_yaw_sign=self.config.imu_config.yaw_sign,
           imu_max_tilt_deg=self.config.imu_config.max_tilt_deg,
           imu_max_gap_s=self.config.imu_config.max_gap_s)
        runtime.set_goal(self.goal, self.target_yaw_deg)
        LOGGER.info("目标设为 (%.1f, %.1f), 初始路径 %.1f mm", self.goal[0], self.goal[1],
                    runtime.planner.last_plan.length_mm if runtime.planner.last_plan else -1.0)
        self._start_workers()
        LOGGER.info("保持车辆静止，等待 IMU 稳定及连续静止标定通过后才允许行驶")

        last_imu: float | None = None
        last_lidar: float | None = None
        startup_deadline = time.monotonic() + settings.startup_timeout_s
        next_tick = time.monotonic()
        try:
            while not self.stop_event.is_set():
                self._raise_worker_error()
                imu_time, lidar_time = self._consume_sensors(runtime)
                last_imu = imu_time if imu_time is not None else last_imu
                last_lidar = lidar_time if lidar_time is not None else last_lidar
                now = time.monotonic()

                if last_imu is None or last_lidar is None:
                    self.driver.stop()
                    if now >= startup_deadline:
                        missing = []
                        if last_imu is None:
                            missing.append("IMU 连续静止标定/新鲜数据")
                        if last_lidar is None:
                            missing.append("雷达")
                        raise SafetyStop("启动自检超时，未收到：" + "、".join(missing))
                else:
                    if now - last_imu > settings.imu_timeout_s:
                        raise SafetyStop(f"IMU 数据超时 {now - last_imu:.2f} s")
                    if now - last_lidar > settings.lidar_timeout_s:
                        raise SafetyStop(f"雷达数据超时 {now - last_lidar:.2f} s")
                    command = runtime.step(now)
                    LOGGER.debug(
                        "pose=(%.1f, %.1f, %.1f, %.2f) command=%s obstacles=%d",
                        runtime.pose.x_mm, runtime.pose.y_mm, runtime.pose.yaw_deg,
                        runtime.pose.confidence, command, len(runtime.obstacles),
                    )
                    if runtime.follower.finished:
                        LOGGER.info("到达目标 (%.1f, %.1f)", self.goal[0], self.goal[1])
                        return 0

                next_tick += 1.0 / settings.control_hz
                delay = next_tick - time.monotonic()
                if delay > 0.0:
                    self.stop_event.wait(delay)
                else:
                    next_tick = time.monotonic()
            return 130
        finally:
            try:
                runtime.emergency_stop()
            finally:
                self.close()

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self.stop_event.set()
        # Closing streams unblocks vendor/serial reads before joining workers.
        if self.imu_stream is not None:
            try:
                self.imu_stream.close()
            except Exception:
                LOGGER.exception("关闭 IMU 串口失败")
        if self.lidar is not None:
            try:
                self.lidar.close()
            except Exception:
                LOGGER.exception("关闭雷达失败")
        for worker in self.workers:
            worker.join()
        if self.driver is not None:
            try:
                self.driver.stop()
            except Exception:
                LOGGER.exception("最终停车帧发送失败")
            LOGGER.info("F407 链路统计 %s; fault=%s",
                        asdict(self.driver.statistics), self.driver.fault_reason)
            try:
                self.driver.close()
            except Exception:
                LOGGER.exception("关闭 F407 串口失败")


class _MemoryStream:
    def __init__(self) -> None:
        self.frames: list[bytes] = []

    def write(self, data: bytes) -> int:
        self.frames.append(data)
        return len(data)


def configure_logging(log_file: Path, verbose: bool = False) -> None:
    log_file.parent.mkdir(parents=True, exist_ok=True)
    formatter = logging.Formatter("%(asctime)s %(levelname)s %(threadName)s %(message)s")
    file_handler = RotatingFileHandler(
        log_file, maxBytes=5 * 1024 * 1024, backupCount=3, encoding="utf-8"
    )
    file_handler.setFormatter(formatter)
    console = logging.StreamHandler()
    console.setFormatter(formatter)
    LOGGER.handlers.clear()
    LOGGER.addHandler(file_handler)
    LOGGER.addHandler(console)
    LOGGER.setLevel(logging.DEBUG if verbose else logging.INFO)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Jetson 动态地图与底盘控制主程序")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG, help="车辆 JSON 配置")
    parser.add_argument("--goal", choices=sorted(STATIONS), help="目标站点")
    parser.add_argument("--goal-x", type=float, help="自定义目标场地 X/mm")
    parser.add_argument("--goal-y", type=float, help="自定义目标场地 Y/mm")
    parser.add_argument("--target-yaw", type=float, help="到达时目标航向/deg")
    parser.add_argument("--plan-only", action="store_true", help="仅检查配置和规划，不打开硬件")
    parser.add_argument("-v", "--verbose", action="store_true", help="输出每周期调试信息")
    return parser


def resolve_goal(args: argparse.Namespace, config: VehicleConfig) -> tuple[float, float]:
    if (args.goal_x is None) != (args.goal_y is None):
        raise ConfigurationError("--goal-x 与 --goal-y 必须同时提供")
    if args.goal and args.goal_x is not None:
        raise ConfigurationError("--goal 与自定义坐标不能同时提供")
    if args.goal_x is not None:
        goal = (float(args.goal_x), float(args.goal_y))
        if not all(math.isfinite(value) for value in goal):
            raise ConfigurationError("目标坐标必须是有限数字")
        return goal
    return STATIONS[args.goal or config.settings.default_goal]


def plan_only(config: VehicleConfig, goal: tuple[float, float], target_yaw: float | None) -> int:
    stream = _MemoryStream()
    now = time.monotonic()
    runtime = VehicleRuntime(Pose2D(
        config.settings.initial_x_mm,
        config.settings.initial_y_mm,
        config.settings.initial_yaw_deg,
        now,
    ), DriverLink(stream, watchdog_ms=config.f407_watchdog_ms))
    runtime.set_goal(goal, target_yaw)
    assert runtime.planner.last_plan is not None
    print(json.dumps(runtime.planner.last_plan.to_dict(), ensure_ascii=False, indent=2))
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    app: JetsonApplication | None = None
    previous_handlers: dict[int, Any] = {}
    try:
        config = load_vehicle_config(args.config.resolve())
        goal = resolve_goal(args, config)
        target_yaw = args.target_yaw if args.target_yaw is not None else config.settings.target_yaw_deg
        if target_yaw is not None and not math.isfinite(target_yaw):
            raise ConfigurationError("目标航向必须是有限数字")
        configure_logging(config.settings.log_file, args.verbose)
        if args.plan_only:
            return plan_only(config, goal, target_yaw)

        app = JetsonApplication(config, goal, target_yaw)

        def handle_signal(signum: int, _frame: Any) -> None:
            LOGGER.warning("收到信号 %s，执行安全停车", signum)
            app.request_stop()

        for signum in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[signum] = signal.signal(signum, handle_signal)
        return app.run()
    except (ConfigurationError, SafetyStop, RuntimeError, OSError, ValueError) as exc:
        if LOGGER.handlers:
            LOGGER.error("主程序安全退出：%s", exc)
        else:
            print(f"Jetson 主程序启动失败：{exc}", file=sys.stderr)
        if app is not None:
            app.close()
        return 2
    except Exception:
        if LOGGER.handlers:
            LOGGER.exception("未处理异常，执行安全停车")
        else:
            print("Jetson 主程序发生未处理异常", file=sys.stderr)
        if app is not None:
            app.close()
        return 3
    finally:
        for signum, previous in previous_handlers.items():
            signal.signal(signum, previous)


if __name__ == "__main__":
    raise SystemExit(main())
