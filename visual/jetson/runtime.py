from __future__ import annotations

from jetson.control import PathFollower
from jetson.hardware import DriverLink
from jetson.lidar import BoundaryLocalizer, ObstacleDetector, ObstacleTracker
from jetson.localization import ImuFilter, PoseEstimator
from jetson.models import BodyVelocity, ImuSample, Pose2D
from jetson.planning import DynamicPlanner


class VehicleRuntime:
    """Hardware-shaped dynamic-map control loop for the Jetson."""

    def __init__(self, initial_pose: Pose2D, driver: DriverLink, *,
                 imu_yaw_sign: int = 1, imu_max_tilt_deg: float = 15.0,
                 imu_max_gap_s: float = 0.5) -> None:
        self.driver = driver
        self.imu = ImuFilter(initial_yaw_deg=initial_pose.yaw_deg,
                             yaw_sign=imu_yaw_sign, max_tilt_deg=imu_max_tilt_deg,
                             max_gap_s=imu_max_gap_s)
        self.pose_estimator = PoseEstimator(initial_pose)
        self.obstacle_detector = ObstacleDetector()
        self.obstacle_tracker = ObstacleTracker()
        self.boundary_localizer = BoundaryLocalizer()
        self.planner = DynamicPlanner()
        self.follower = PathFollower()
        self.goal: tuple[float, float] | None = None
        self.obstacles = []
        self._last_velocity = BodyVelocity(0.0, 0.0, 0.0)
        self._target_yaw_deg: float | None = None

    @property
    def pose(self) -> Pose2D:
        return self.pose_estimator.pose

    def set_goal(self, goal: tuple[float, float], target_yaw_deg: float | None = None) -> None:
        self.goal = goal
        self._target_yaw_deg = target_yaw_deg
        result = self.planner.plan(self.pose, goal, self.obstacles)
        self.follower.set_path(result.waypoints, target_yaw_deg)

    def ingest_imu(self, sample: ImuSample) -> Pose2D:
        if sample.timestamp_s < self.pose.timestamp_s:
            return self.pose
        if self.imu.last_timestamp_s is not None and sample.timestamp_s <= self.imu.last_timestamp_s:
            return self.pose
        try:
            yaw = self.imu.update(sample)
        except ValueError:
            self.emergency_stop()
            raise
        self.pose_estimator.predict(self._last_velocity, sample.timestamp_s)
        return self.pose_estimator.update_imu_yaw(yaw, sample.timestamp_s)

    def ingest_lidar(self, angles_deg: list[float], ranges_mm: list[float], timestamp_s: float,
                     *, masked_sectors: tuple[tuple[float, float], ...] = ()) -> Pose2D:
        # Without a pose-history buffer, delayed scans must not correct a newer
        # pose or project obstacle coordinates using a pose from another time.
        if timestamp_s < self.pose.timestamp_s:
            return self.pose
        self.pose_estimator.predict(self._last_velocity, timestamp_s)
        points = self.obstacle_detector.polar_to_body(
            angles_deg, ranges_mm, masked_sectors=masked_sectors
        )
        x, y = self.boundary_localizer.estimate(points, self.pose)
        pose = self.pose_estimator.correct_from_lidar(x, y, timestamp_s)
        detections = self.obstacle_detector.detect(points, pose)
        confirmed = self.obstacle_tracker.update(detections)
        changed = self.planner.obstacles_changed(confirmed)
        self.obstacles = confirmed
        if changed and self.goal is not None:
            result = self.planner.plan(pose, self.goal, confirmed)
            self.follower.set_path(result.waypoints, self._target_yaw_deg)
        return pose

    def step(self, timestamp_s: float) -> tuple[int, int, int]:
        pose = self.pose_estimator.predict(self._last_velocity, timestamp_s)
        self._last_velocity = self.follower.update(pose)
        x, y, z = self.follower.normalize_velocity(self._last_velocity)
        if self.follower.finished or pose.confidence < self.follower.config.minimum_pose_confidence:
            self.driver.stop()
            self._last_velocity = BodyVelocity(0.0, 0.0, 0.0)
            return 0, 0, 0
        self.driver.send_velocity(x, y, z)
        return x, y, z

    def command_velocity_normalized(self, x: int, y: int, z: int) -> None:
        """Issue a bounded task-layer search/servo command outside path following."""

        if not all(-100 <= value <= 100 for value in (x, y, z)):
            raise ValueError("normalized velocity must be in -100..100")
        cfg = self.follower.config
        self._last_velocity = BodyVelocity(
            x * cfg.max_linear_mm_s / 100.0,
            y * cfg.max_linear_mm_s / 100.0,
            z * cfg.max_yaw_deg_s / 100.0,
        )
        self.driver.send_velocity(x, y, z)

    def emergency_stop(self) -> None:
        self._last_velocity = BodyVelocity(0.0, 0.0, 0.0)
        self.driver.stop()
