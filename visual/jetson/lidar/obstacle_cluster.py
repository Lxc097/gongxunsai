from __future__ import annotations

from collections import deque
import math
from statistics import median
from typing import Iterable, Sequence

from jetson.models import CircleDetection, LidarPoint, Pose2D


class ObstacleDetector:
    """X3 Pro 扫描过滤、相邻聚类和圆柱中心估计。"""

    def __init__(
        self,
        *,
        minimum_range_mm: float = 100.0,
        maximum_range_mm: float = 3000.0,
        cluster_gap_mm: float = 90.0,
        pillar_radius_mm: float = 25.0,
        minimum_points: int = 3,
    ) -> None:
        self.minimum_range_mm = minimum_range_mm
        self.maximum_range_mm = maximum_range_mm
        self.cluster_gap_mm = cluster_gap_mm
        self.pillar_radius_mm = pillar_radius_mm
        self.minimum_points = minimum_points

    def polar_to_body(
        self,
        angles_deg: Sequence[float],
        ranges_mm: Sequence[float],
        *,
        masked_sectors: Iterable[tuple[float, float]] = (),
    ) -> list[LidarPoint]:
        if len(angles_deg) != len(ranges_mm):
            raise ValueError("雷达角度与距离数量不一致")
        masks = list(masked_sectors)
        points: list[LidarPoint] = []
        for angle, distance in zip(angles_deg, ranges_mm):
            normalized = angle % 360.0
            if not math.isfinite(distance) or not self.minimum_range_mm <= distance <= self.maximum_range_mm:
                continue
            if any(start <= normalized <= end for start, end in masks):
                continue
            radians = math.radians(angle)
            points.append(LidarPoint(distance * math.cos(radians), distance * math.sin(radians)))
        return points

    def detect(self, points: Sequence[LidarPoint], pose: Pose2D) -> list[CircleDetection]:
        if not points:
            return []
        clusters: list[list[LidarPoint]] = [[points[0]]]
        for point in points[1:]:
            previous = clusters[-1][-1]
            if math.hypot(point.x_mm - previous.x_mm, point.y_mm - previous.y_mm) <= self.cluster_gap_mm:
                clusters[-1].append(point)
            else:
                clusters.append([point])

        yaw = math.radians(pose.yaw_deg)
        detections: list[CircleDetection] = []
        for cluster in clusters:
            if len(cluster) < self.minimum_points:
                continue
            spread = max(
                math.hypot(a.x_mm - b.x_mm, a.y_mm - b.y_mm)
                for a in cluster for b in cluster
            )
            if not 8.0 <= spread <= 180.0:
                continue
            body_x = median([point.x_mm for point in cluster])
            body_y = median([point.y_mm for point in cluster])
            length = math.hypot(body_x, body_y)
            if length <= 1.0:
                continue
            body_x += self.pillar_radius_mm * body_x / length
            body_y += self.pillar_radius_mm * body_y / length
            field_x = pose.x_mm + math.cos(yaw) * body_x - math.sin(yaw) * body_y
            field_y = pose.y_mm + math.sin(yaw) * body_x + math.cos(yaw) * body_y
            confidence = min(1.0, len(cluster) / 8.0)
            detections.append(CircleDetection(field_x, field_y, self.pillar_radius_mm, confidence))
        return detections


class ObstacleTracker:
    """连续多帧确认后才向 A* 发布圆柱障碍。"""

    def __init__(self, *, match_radius_mm: float = 100.0, confirm_frames: int = 3) -> None:
        self.match_radius_mm = match_radius_mm
        self.confirm_frames = confirm_frames
        self._tracks: deque[tuple[float, float, int, float]] = deque(maxlen=32)

    def update(self, detections: Sequence[CircleDetection]) -> list[CircleDetection]:
        updated: list[tuple[float, float, int, float]] = []
        used: set[int] = set()
        for detection in detections:
            best = None
            best_distance = self.match_radius_mm
            for index, track in enumerate(self._tracks):
                if index in used:
                    continue
                distance = math.hypot(detection.x_mm - track[0], detection.y_mm - track[1])
                if distance < best_distance:
                    best, best_distance = index, distance
            if best is None:
                updated.append((detection.x_mm, detection.y_mm, 1, detection.confidence))
            else:
                used.add(best)
                old = self._tracks[best]
                updated.append((0.6 * old[0] + 0.4 * detection.x_mm,
                                0.6 * old[1] + 0.4 * detection.y_mm,
                                old[2] + 1,
                                max(old[3], detection.confidence)))
        self._tracks = deque(updated, maxlen=32)
        return [CircleDetection(x, y, 25.0, confidence)
                for x, y, count, confidence in self._tracks
                if count >= self.confirm_frames]
