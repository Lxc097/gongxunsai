from __future__ import annotations

import math
from statistics import median, pstdev

from jetson.models import ImuSample


def normalize_angle(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


class ImuFilter:
    """Planar, upright IMU heading; six-axis yaw is not an absolute reference.

    By default integrate bias-corrected gyro only. Module yaw checks discontinuities
    but cannot remove long-term drift. Nonzero yaw_gain is an explicit opt-in for
    an independently referenced heading source, not for this six-axis module.
    """

    def __init__(self, *, max_jump_deg: float = 35.0, yaw_gain: float = 0.0,
                 initial_yaw_deg: float | None = None, yaw_sign: int = 1,
                 max_tilt_deg: float = 15.0, max_gap_s: float = 0.5) -> None:
        if yaw_sign not in (-1, 1) or isinstance(yaw_sign, bool):
            raise ValueError("yaw_sign must be +1 or -1")
        if not all(math.isfinite(v) and v > 0 for v in (max_jump_deg, max_tilt_deg, max_gap_s)):
            raise ValueError("IMU limits must be finite and positive")
        if not math.isfinite(yaw_gain) or not 0 <= yaw_gain <= 1:
            raise ValueError("yaw_gain must be in 0..1")
        if initial_yaw_deg is not None and not math.isfinite(initial_yaw_deg):
            raise ValueError("initial_yaw_deg must be finite")
        self.max_jump_deg = max_jump_deg
        self.yaw_gain = yaw_gain
        self.initial_yaw_deg = initial_yaw_deg
        self.yaw_sign = yaw_sign
        self.max_tilt_deg = max_tilt_deg
        self.max_gap_s = max_gap_s
        self.gyro_bias_deg_s = 0.0
        self.yaw_deg: float | None = None
        self.last_timestamp_s: float | None = None
        self._last_sensor_yaw: float | None = None
        self._last_gyro: float | None = None
        self._yaw_offset = 0.0
        self.valid = False

    def calibrate_stationary(self, samples: list[ImuSample]) -> float:
        # Do not discard moving/tilted samples and accidentally accept the rest.
        if len(samples) < 20:
            raise ValueError("IMU 静止标定至少需要 20 个连续样本")
        for sample in samples:
            self.validate_sample(sample, max_tilt_deg=min(5.0, self.max_tilt_deg))
        if any(not 0 < b.timestamp_s - a.timestamp_s <= self.max_gap_s
               for a, b in zip(samples, samples[1:])):
            raise ValueError("IMU 标定时间戳必须递增且采样连续")
        usable = [self.yaw_sign * s.gyro_z_deg_s for s in samples]
        yaw_travel = sum(abs(normalize_angle(b.yaw_deg - a.yaw_deg))
                         for a, b in zip(samples, samples[1:]))
        if max(map(abs, usable)) > 3.0 or pstdev(usable) > 0.35 or yaw_travel > 3.0:
            raise ValueError("IMU 标定检测到转动或振动，请保持车体静止")
        self.gyro_bias_deg_s = median(usable)
        # Calibration starts a new heading session; no old integration time survives.
        self.yaw_deg = None
        self.last_timestamp_s = None
        self._last_sensor_yaw = None
        self._last_gyro = None
        self.valid = False
        return self.gyro_bias_deg_s

    @staticmethod
    def validate_sample(sample: ImuSample, *, max_tilt_deg: float = 15.0) -> None:
        if not all(math.isfinite(v) for v in vars(sample).values()):
            raise ValueError("IMU 样本包含非有限值")
        if abs(sample.roll_deg) > max_tilt_deg or abs(sample.pitch_deg) > max_tilt_deg:
            raise ValueError("IMU 倾角超限，平面航向模型不再适用")

    def update(self, sample: ImuSample) -> float:
        self.valid = False
        self.validate_sample(sample, max_tilt_deg=self.max_tilt_deg)
        sensor_yaw = normalize_angle(self.yaw_sign * sample.yaw_deg)
        gyro = self.yaw_sign * sample.gyro_z_deg_s
        if self.yaw_deg is None or self.last_timestamp_s is None:
            self.yaw_deg = normalize_angle(sensor_yaw if self.initial_yaw_deg is None
                                           else self.initial_yaw_deg)
            self._yaw_offset = normalize_angle(self.yaw_deg - sensor_yaw)
            self.last_timestamp_s = sample.timestamp_s
            self._last_sensor_yaw = sensor_yaw
            self._last_gyro = gyro
            self.valid = True
            return self.yaw_deg

        dt = sample.timestamp_s - self.last_timestamp_s
        if not 0.0 < dt <= self.max_gap_s:
            raise ValueError("IMU 时间倒退、重复或采样间隔超限")

        assert self._last_gyro is not None and self._last_sensor_yaw is not None
        gyro_delta = 0.5 * (self._last_gyro + gyro) * dt
        sensor_delta = normalize_angle(sensor_yaw - self._last_sensor_yaw)
        if abs(normalize_angle(sensor_delta - gyro_delta)) > self.max_jump_deg:
            raise ValueError("IMU 航向跳变，可能重启或数据异常；停车后重新标定")
        predicted = normalize_angle(self.yaw_deg + gyro_delta - self.gyro_bias_deg_s * dt)
        if self.yaw_gain:
            innovation = normalize_angle(sensor_yaw + self._yaw_offset - predicted)
            if abs(innovation) > self.max_jump_deg:
                raise ValueError("IMU 外部航向与预测不一致")
            predicted = normalize_angle(predicted + self.yaw_gain * innovation)
        self.valid = True
        self.yaw_deg = predicted
        self.last_timestamp_s = sample.timestamp_s
        self._last_sensor_yaw = sensor_yaw
        self._last_gyro = gyro
        return predicted
