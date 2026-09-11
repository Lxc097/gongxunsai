from __future__ import annotations

import math
from statistics import median
from typing import Sequence

from jetson.models import LidarPoint, Pose2D


class BoundaryLocalizer:
    """用场地四边界的轴向回波对指令里程计 X/Y 做鲁棒修正。"""

    def __init__(self, width_mm: float = 2400.0, height_mm: float = 2400.0) -> None:
        self.width_mm = width_mm
        self.height_mm = height_mm

    def estimate(self, points: Sequence[LidarPoint], pose: Pose2D) -> tuple[float | None, float | None]:
        yaw = math.radians(pose.yaw_deg)
        x_candidates: list[float] = []
        y_candidates: list[float] = []
        for point in points:
            dx = math.cos(yaw) * point.x_mm - math.sin(yaw) * point.y_mm
            dy = math.sin(yaw) * point.x_mm + math.cos(yaw) * point.y_mm
            angle = math.degrees(math.atan2(dy, dx))
            if abs(angle) < 12.0 and dx > 300.0:
                x_candidates.append(self.width_mm - dx)
            elif abs(abs(angle) - 180.0) < 12.0 and dx < -300.0:
                x_candidates.append(-dx)
            if abs(angle - 90.0) < 12.0 and dy > 300.0:
                y_candidates.append(self.height_mm - dy)
            elif abs(angle + 90.0) < 12.0 and dy < -300.0:
                y_candidates.append(-dy)

        def robust(values: list[float], limit: float) -> float | None:
            if len(values) < 5:
                return None
            center = median(values)
            inliers = [value for value in values if abs(value - center) <= 120.0]
            if len(inliers) < 5:
                return None
            result = median(inliers)
            return result if 0.0 <= result <= limit else None

        return robust(x_candidates, self.width_mm), robust(y_candidates, self.height_mm)
