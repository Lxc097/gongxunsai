from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Pose2D:
    x_mm: float
    y_mm: float
    yaw_deg: float
    timestamp_s: float
    confidence: float = 1.0


@dataclass(frozen=True)
class BodyVelocity:
    forward_mm_s: float
    left_mm_s: float
    yaw_deg_s: float


@dataclass(frozen=True)
class ImuSample:
    yaw_deg: float
    gyro_z_deg_s: float
    roll_deg: float
    pitch_deg: float
    timestamp_s: float


@dataclass(frozen=True)
class LidarPoint:
    x_mm: float
    y_mm: float


@dataclass(frozen=True)
class CircleDetection:
    x_mm: float
    y_mm: float
    radius_mm: float
    confidence: float
