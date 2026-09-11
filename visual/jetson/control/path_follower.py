from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from jetson.models import BodyVelocity, Pose2D


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _angle_error(target: float, current: float) -> float:
    return (target - current + 180.0) % 360.0 - 180.0


@dataclass(frozen=True)
class FollowerConfig:
    position_gain: float = 1.25
    yaw_gain: float = 2.0
    max_linear_mm_s: float = 600.0
    max_yaw_deg_s: float = 100.0
    waypoint_tolerance_mm: float = 45.0
    stop_tolerance_mm: float = 25.0
    yaw_tolerance_deg: float = 3.0
    minimum_pose_confidence: float = 0.2


class PathFollower:
    """Holonomic waypoint follower; output axes match chassis_RC."""

    def __init__(self, config: FollowerConfig | None = None) -> None:
        self.config = config or FollowerConfig()
        self._waypoints: list[tuple[float, float]] = []
        self._index = 0
        self._target_yaw_deg: float | None = None

    def set_path(self, waypoints: Sequence[tuple[float, float]], target_yaw_deg: float | None = None) -> None:
        if not waypoints:
            raise ValueError("path must contain at least one waypoint")
        self._waypoints = list(waypoints)
        self._index = 1 if len(self._waypoints) > 1 else 0
        self._target_yaw_deg = target_yaw_deg

    @property
    def finished(self) -> bool:
        return bool(self._waypoints) and self._index >= len(self._waypoints)

    @property
    def target_yaw_deg(self) -> float | None:
        return self._target_yaw_deg

    def update(self, pose: Pose2D) -> BodyVelocity:
        if not self._waypoints or pose.confidence < self.config.minimum_pose_confidence:
            return BodyVelocity(0.0, 0.0, 0.0)
        while self._index < len(self._waypoints):
            target = self._waypoints[self._index]
            distance = math.hypot(target[0] - pose.x_mm, target[1] - pose.y_mm)
            tolerance = (self.config.stop_tolerance_mm if self._index == len(self._waypoints) - 1
                         else self.config.waypoint_tolerance_mm)
            if distance > tolerance:
                break
            if (
                self._index == len(self._waypoints) - 1
                and self._target_yaw_deg is not None
                and abs(_angle_error(self._target_yaw_deg, pose.yaw_deg)) > self.config.yaw_tolerance_deg
            ):
                # Keep the final XY target active until the docking heading is reached.
                break
            self._index += 1
        if self.finished:
            return BodyVelocity(0.0, 0.0, 0.0)

        dx = self._waypoints[self._index][0] - pose.x_mm
        dy = self._waypoints[self._index][1] - pose.y_mm
        yaw = math.radians(pose.yaw_deg)
        forward_error = math.cos(yaw) * dx + math.sin(yaw) * dy
        left_error = -math.sin(yaw) * dx + math.cos(yaw) * dy
        target_yaw = self._target_yaw_deg
        if target_yaw is None:
            target_yaw = math.degrees(math.atan2(dy, dx))
        return BodyVelocity(
            _clamp(self.config.position_gain * forward_error, self.config.max_linear_mm_s),
            _clamp(self.config.position_gain * left_error, self.config.max_linear_mm_s),
            _clamp(self.config.yaw_gain * _angle_error(target_yaw, pose.yaw_deg), self.config.max_yaw_deg_s),
        )

    def normalized_command(self, pose: Pose2D) -> tuple[int, int, int]:
        return self.normalize_velocity(self.update(pose))

    def normalize_velocity(self, velocity: BodyVelocity) -> tuple[int, int, int]:
        """Convert one already-computed velocity to the F407 -100..100 axes.

        Keeping this conversion separate avoids advancing waypoint state twice
        in one control cycle when callers also need the physical velocity for
        command-odometry prediction.
        """

        return (
            int(round(100.0 * velocity.forward_mm_s / self.config.max_linear_mm_s)),
            int(round(100.0 * velocity.left_mm_s / self.config.max_linear_mm_s)),
            int(round(100.0 * velocity.yaw_deg_s / self.config.max_yaw_deg_s)),
        )
