from __future__ import annotations

import math

from jetson.models import BodyVelocity, Pose2D
from .imu_filter import normalize_angle


class PoseEstimator:
    """指令里程计预测 + IMU 航向 + 雷达边界位置修正。"""

    def __init__(self, initial_pose: Pose2D, *, correction_gain: float = 0.35) -> None:
        self._pose = initial_pose
        self.correction_gain = correction_gain
        self._last_position_correction_s = initial_pose.timestamp_s

    @property
    def pose(self) -> Pose2D:
        return self._pose

    def predict(self, command: BodyVelocity, timestamp_s: float) -> Pose2D:
        dt = timestamp_s - self._pose.timestamp_s
        if dt <= 0:
            return self._pose
        dt = min(dt, 0.2)
        yaw = math.radians(self._pose.yaw_deg)
        field_vx = math.cos(yaw) * command.forward_mm_s - math.sin(yaw) * command.left_mm_s
        field_vy = math.sin(yaw) * command.forward_mm_s + math.cos(yaw) * command.left_mm_s
        age = timestamp_s - self._last_position_correction_s
        confidence = max(0.15, math.exp(-age / 8.0))
        self._pose = Pose2D(
            self._pose.x_mm + field_vx * dt,
            self._pose.y_mm + field_vy * dt,
            normalize_angle(self._pose.yaw_deg + command.yaw_deg_s * dt),
            timestamp_s,
            confidence,
        )
        return self._pose

    def update_imu_yaw(self, yaw_deg: float, timestamp_s: float) -> Pose2D:
        if timestamp_s < self._pose.timestamp_s:
            return self._pose
        # Only predict() owns the integration clock. Runtime predicts to the
        # observation time before correcting; a yaw update never consumes XY dt.
        self._pose = Pose2D(
            self._pose.x_mm,
            self._pose.y_mm,
            normalize_angle(yaw_deg),
            self._pose.timestamp_s,
            self._pose.confidence,
        )
        return self._pose

    def correct_from_lidar(self, x_mm: float | None, y_mm: float | None, timestamp_s: float) -> Pose2D:
        if timestamp_s < self._pose.timestamp_s or (x_mm is None and y_mm is None):
            return self._pose
        x = self._pose.x_mm if x_mm is None else (
            (1.0 - self.correction_gain) * self._pose.x_mm + self.correction_gain * x_mm
        )
        y = self._pose.y_mm if y_mm is None else (
            (1.0 - self.correction_gain) * self._pose.y_mm + self.correction_gain * y_mm
        )
        if x_mm is not None or y_mm is not None:
            self._last_position_correction_s = timestamp_s
        self._pose = Pose2D(x, y, self._pose.yaw_deg, self._pose.timestamp_s, 1.0)
        return self._pose
