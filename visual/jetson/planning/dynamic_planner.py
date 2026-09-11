from __future__ import annotations

import math
from typing import Sequence

from jetson.models import CircleDetection, Pose2D
from path_planning.path_planning import AStarPlanner, CircleObstacle, CompetitionArena, PlanResult, PlannerConfig


class DynamicPlanner:
    """Adapter that replans the existing arena map when X3 Pro obstacles move."""

    def __init__(self, config: PlannerConfig | None = None, *, replan_distance_mm: float = 80.0) -> None:
        self.config = config or PlannerConfig()
        self.replan_distance_mm = replan_distance_mm
        self._obstacles: tuple[CircleObstacle, ...] = ()
        self.last_plan: PlanResult | None = None

    @staticmethod
    def _convert(detections: Sequence[CircleDetection]) -> tuple[CircleObstacle, ...]:
        return tuple(CircleObstacle(item.x_mm, item.y_mm, item.radius_mm) for item in detections)

    def obstacles_changed(self, detections: Sequence[CircleDetection]) -> bool:
        current = self._convert(detections)
        if len(current) != len(self._obstacles):
            return True
        unmatched = list(current)
        for old in self._obstacles:
            match = min(unmatched, key=lambda item: math.hypot(item.x - old.x, item.y - old.y), default=None)
            if match is None or math.hypot(match.x - old.x, match.y - old.y) > self.replan_distance_mm:
                return True
            unmatched.remove(match)
        return False

    def plan(self, pose: Pose2D, goal: tuple[float, float], detections: Sequence[CircleDetection]) -> PlanResult:
        self._obstacles = self._convert(detections)
        arena = CompetitionArena(self.config, self._obstacles)
        self.last_plan = AStarPlanner(arena).plan((pose.x_mm, pose.y_mm), goal, pose.yaw_deg)
        return self.last_plan
