from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Protocol

from jetson.hardware import OpenMvObservation
from jetson.models import ImuSample, Pose2D
from path_planning.path_planning import STATIONS

from .actuator import MissionManipulator
from .models import (
    ActionState, MaterialColor, MissionCode, MissionState, MissionStep,
    MissionStepKind,
)
from .plan import build_preliminary_steps
from .qr_parser import MissionCodeError, parse_mission_code
from .statistics import MissionStatistics


# Face the front camera/arm toward each boundary-side work area.
STATION_TARGET_YAW_DEG = {
    "qr": 0.0,
    "raw_material": 90.0,
    "rough_processing": -90.0,
    "temporary_storage": 180.0,
    "start_1": 0.0,
    "start_2": 0.0,
}


class NavigationRuntime(Protocol):
    pose: Pose2D
    follower: object

    def set_goal(self, goal: tuple[float, float], target_yaw_deg: float | None = None) -> None: ...
    def step(self, timestamp_s: float) -> tuple[int, int, int]: ...
    def emergency_stop(self) -> None: ...
    def ingest_imu(self, sample: ImuSample) -> Pose2D: ...
    def ingest_lidar(
        self, angles_deg: list[float], ranges_mm: list[float], timestamp_s: float,
        *, masked_sectors: tuple[tuple[float, float], ...] = (),
    ) -> Pose2D: ...


@dataclass(frozen=True)
class MissionConfig:
    sensor_stale_s: float = 1.0
    tof_stale_s: float = 0.5
    qr_timeout_s: float = 20.0
    navigation_timeout_s: float = 35.0
    alignment_timeout_s: float = 8.0
    action_timeout_s: float = 20.0
    stable_alignment_frames: int = 3
    material_tolerance_px: float = 8.0
    ring_tolerance_px: float = 6.0
    minimum_tof_mm: int = 70
    maximum_tof_mm: int = 7500
    max_step_retries: int = 2
    qr_search_yaw_command: int = 12
    require_imu: bool = True
    require_lidar: bool = True

    def __post_init__(self) -> None:
        if min(
            self.sensor_stale_s, self.tof_stale_s, self.qr_timeout_s, self.navigation_timeout_s,
            self.alignment_timeout_s, self.action_timeout_s,
        ) <= 0:
            raise ValueError("mission timeouts must be positive")
        if self.stable_alignment_frames < 1 or self.max_step_retries < 0:
            raise ValueError("invalid mission retry/stability parameters")
        if not math.isfinite(self.tof_stale_s) or not 30 <= self.minimum_tof_mm < self.maximum_tof_mm <= 7500:
            raise ValueError("invalid ToF timeout or working range")


class MissionController:
    """Non-blocking competition task layer joining perception, planning and actuation."""

    def __init__(
        self,
        runtime: NavigationRuntime,
        manipulator: MissionManipulator,
        *,
        config: MissionConfig | None = None,
        home_station: str = "start_1",
    ) -> None:
        if home_station not in STATIONS:
            raise ValueError(f"unknown home station: {home_station}")
        self.runtime = runtime
        self.manipulator = manipulator
        self.config = config or MissionConfig()
        self.home_station = home_station
        self.state = MissionState.BOOT_SELF_TEST
        self.task_code: MissionCode | None = None
        self.statistics = MissionStatistics()
        self.failure_reason: str | None = None
        self.steps: list[MissionStep] = []
        self.step_index = -1
        self._deadline_s: float | None = None
        self._last_imu_s: float | None = None
        self._last_lidar_s: float | None = None
        self._last_openmv_s: float | None = None
        self._last_tof_s: float | None = None
        self._last_openmv: OpenMvObservation | None = None
        self._last_tof_mm: int | None = None
        self._tof_stable = False
        self._alignment_count = 0
        self._step_retries = 0

    @property
    def current_step(self) -> MissionStep | None:
        if 0 <= self.step_index < len(self.steps):
            return self.steps[self.step_index]
        return None

    def complete_self_test(self, ok: bool, timestamp_s: float, reason: str = "") -> None:
        if self.state is not MissionState.BOOT_SELF_TEST:
            raise RuntimeError("self test can only complete once")
        if not ok:
            self._safe_stop(timestamp_s, reason or "boot self test failed")
            return
        self.state = MissionState.WAIT_START
        self.statistics.event(timestamp_s, "self_test_passed")

    def ingest_imu(self, sample: ImuSample) -> Pose2D:
        self._last_imu_s = sample.timestamp_s
        return self.runtime.ingest_imu(sample)

    def ingest_lidar(
        self,
        angles_deg: list[float],
        ranges_mm: list[float],
        timestamp_s: float,
        *,
        masked_sectors: tuple[tuple[float, float], ...] = (),
    ) -> Pose2D:
        self._last_lidar_s = timestamp_s
        return self.runtime.ingest_lidar(
            angles_deg, ranges_mm, timestamp_s, masked_sectors=masked_sectors
        )

    def ingest_openmv(self, observation: OpenMvObservation, timestamp_s: float) -> None:
        self._last_openmv = observation
        self._last_openmv_s = timestamp_s
        step = self.current_step
        if step is None:
            return

        aligned = False
        if self.state is MissionState.FINE_ALIGN_MATERIAL:
            if observation.material_color is not None and step.color is not None:
                try:
                    matches = MaterialColor.from_name(observation.material_color) == step.color
                except ValueError:
                    matches = False
                dx, dy = observation.material_dx_px, observation.material_dy_px
                if matches and dx is not None and dy is not None:
                    self.manipulator.align_xy(dx, dy, target="material")
                    aligned = abs(dx) <= self.config.material_tolerance_px and abs(dy) <= self.config.material_tolerance_px
        elif self.state is MissionState.FINE_ALIGN_RING:
            dx, dy = observation.ring_dx_px, observation.ring_dy_px
            if dx is not None and dy is not None:
                self.manipulator.align_xy(dx, dy, target="ring", ring_number=step.ring_number)
                aligned = abs(dx) <= self.config.ring_tolerance_px and abs(dy) <= self.config.ring_tolerance_px

        self._alignment_count = self._alignment_count + 1 if aligned else 0

    def ingest_tof(self, distance_mm: int | None, stable: bool, timestamp_s: float) -> None:
        if not math.isfinite(timestamp_s):
            self._last_tof_mm = None
            self._tof_stable = False
            return
        if self._last_tof_s is not None and timestamp_s < self._last_tof_s:
            return  # An old queued reading cannot restore a newer invalid state.
        self._last_tof_s = timestamp_s
        valid = (
            type(distance_mm) is int
            and self.config.minimum_tof_mm <= distance_mm <= self.config.maximum_tof_mm
        )
        self._last_tof_mm = int(distance_mm) if valid else None
        self._tof_stable = bool(valid and stable)

    def ingest_qr(self, payload: str, stable: bool, timestamp_s: float) -> bool:
        if self.state is not MissionState.SEARCH_QR or not stable:
            return False
        self.runtime.emergency_stop()
        self.state = MissionState.READ_AND_VALIDATE_QR
        try:
            self.task_code = parse_mission_code(payload)
        except MissionCodeError as exc:
            self.state = MissionState.SEARCH_QR
            self.statistics.event(timestamp_s, "qr_rejected", str(exc))
            return False
        self.statistics.event(timestamp_s, "qr_accepted", self.task_code.canonical)
        self.state = MissionState.BUILD_OBSTACLE_MAP
        self._deadline_s = timestamp_s + self.config.sensor_stale_s
        return True

    def ingest_qr_observation(self, observation: object | None, timestamp_s: float) -> bool:
        """Accept the existing camera ``QrObservation`` directly."""

        if observation is None:
            return False
        return self.ingest_qr(
            str(getattr(observation, "payload", "")),
            bool(getattr(observation, "stable", False)),
            timestamp_s,
        )

    def start(self, timestamp_s: float) -> None:
        if self.state is not MissionState.WAIT_START:
            raise RuntimeError("mission is not waiting for start")
        if not self._global_sensors_fresh(timestamp_s):
            self._safe_stop(timestamp_s, "IMU or lidar is not ready at start")
            return
        self.statistics.started_at_s = timestamp_s
        self.statistics.event(timestamp_s, "mission_started")
        self.runtime.set_goal(STATIONS["qr"], STATION_TARGET_YAW_DEG["qr"])
        self.state = MissionState.NAVIGATE_QR
        self._deadline_s = timestamp_s + self.config.navigation_timeout_s

    def _global_sensors_fresh(self, timestamp_s: float) -> bool:
        imu_ok = not self.config.require_imu or (
            self._last_imu_s is not None and timestamp_s - self._last_imu_s <= self.config.sensor_stale_s
        )
        lidar_ok = not self.config.require_lidar or (
            self._last_lidar_s is not None and timestamp_s - self._last_lidar_s <= self.config.sensor_stale_s
        )
        return imu_ok and lidar_ok

    def _tof_ready(self, timestamp_s: float) -> bool:
        return (
            self._tof_stable
            and self._last_tof_mm is not None
            and self._last_tof_s is not None
            and 0 <= timestamp_s - self._last_tof_s <= self.config.tof_stale_s
        )

    def _terminal_observation_fresh(self, timestamp_s: float) -> bool:
        return (
            self._last_openmv is not None
            and self._last_openmv_s is not None
            and timestamp_s - self._last_openmv_s <= self.config.sensor_stale_s
        )

    def _timed_out(self, timestamp_s: float) -> bool:
        return self._deadline_s is not None and timestamp_s > self._deadline_s

    def _start_current_step(self, timestamp_s: float) -> None:
        step = self.current_step
        if step is None:
            self.state = MissionState.MISSION_COMPLETE
            self.statistics.finished_at_s = timestamp_s
            self.statistics.event(timestamp_s, "mission_complete")
            try:
                self.manipulator.safe_retract()
            finally:
                self.runtime.emergency_stop()
            return
        self._alignment_count = 0
        self._last_openmv = None
        self._tof_stable = False
        if step.kind is MissionStepKind.NAVIGATE:
            assert step.station is not None
            self.runtime.set_goal(
                STATIONS[step.station], STATION_TARGET_YAW_DEG.get(step.station)
            )
            self.state = (
                MissionState.RETURN_HOME
                if step.station == self.home_station and self.step_index == len(self.steps) - 1
                else MissionState.NAVIGATE
            )
            self._deadline_s = timestamp_s + self.config.navigation_timeout_s
        elif step.kind is MissionStepKind.PICK_RAW:
            self.state = MissionState.FINE_ALIGN_MATERIAL
            self._deadline_s = timestamp_s + self.config.alignment_timeout_s
        else:
            self.state = MissionState.FINE_ALIGN_RING
            self._deadline_s = timestamp_s + self.config.alignment_timeout_s

    def _advance_step(self, timestamp_s: float) -> None:
        self.step_index += 1
        self._step_retries = 0
        self._start_current_step(timestamp_s)

    def _begin_manipulation(self, timestamp_s: float) -> None:
        step = self.current_step
        observation = self._last_openmv
        distance = self._last_tof_mm
        if step is None or observation is None or distance is None:
            return
        assert step.color is not None and step.vehicle_slot is not None
        if step.kind in {MissionStepKind.PICK_RAW, MissionStepKind.PICK_ROUGH}:
            self.manipulator.start_pick(
                color=step.color,
                ring_number=step.ring_number,
                vehicle_slot=step.vehicle_slot,
                from_turntable=step.kind is MissionStepKind.PICK_RAW,
                observation=observation,
                distance_mm=distance,
            )
            self.state = MissionState.PICK_MATERIAL
        else:
            assert step.ring_number is not None
            self.manipulator.start_place(
                color=step.color,
                ring_number=step.ring_number,
                vehicle_slot=step.vehicle_slot,
                stack_level=step.stack_level,
                observation=observation,
                distance_mm=distance,
            )
            self.state = MissionState.PLACE_OR_STACK
        self._deadline_s = timestamp_s + self.config.action_timeout_s

    def _retry_or_stop(self, timestamp_s: float, reason: str) -> None:
        self.statistics.failures += 1
        self._step_retries += 1
        try:
            self.manipulator.safe_retract()
        except Exception as exc:
            self._safe_stop(timestamp_s, f"safe retract failed: {exc}")
            return
        if self._step_retries > self.config.max_step_retries:
            self._safe_stop(timestamp_s, reason)
            return
        self.statistics.retries += 1
        self.statistics.event(timestamp_s, "step_retry", reason)
        step = self.current_step
        self._alignment_count = 0
        self._tof_stable = False
        self.state = (
            MissionState.FINE_ALIGN_MATERIAL
            if step is not None and step.kind is MissionStepKind.PICK_RAW
            else MissionState.FINE_ALIGN_RING
        )
        self._deadline_s = timestamp_s + self.config.alignment_timeout_s

    def _safe_stop(self, timestamp_s: float, reason: str) -> None:
        if self.state is MissionState.SAFE_STOP:
            return
        self.failure_reason = reason
        self.state = MissionState.SAFE_STOP
        self.statistics.finished_at_s = timestamp_s
        self.statistics.event(timestamp_s, "safe_stop", reason)
        try:
            self.manipulator.abort()
        finally:
            self.runtime.emergency_stop()

    def emergency_stop(self, timestamp_s: float, reason: str = "external emergency stop") -> None:
        self._safe_stop(timestamp_s, reason)

    def step(self, timestamp_s: float) -> MissionState:
        if self.state in {MissionState.SAFE_STOP, MissionState.MISSION_COMPLETE}:
            return self.state

        if self.state in {MissionState.NAVIGATE_QR, MissionState.NAVIGATE, MissionState.RETURN_HOME}:
            if not self._global_sensors_fresh(timestamp_s):
                self._safe_stop(timestamp_s, "IMU or lidar became stale while moving")
                return self.state
            self.runtime.step(timestamp_s)
            if bool(getattr(self.runtime.follower, "finished", False)):
                self.runtime.emergency_stop()
                if self.state is MissionState.NAVIGATE_QR:
                    self.state = MissionState.SEARCH_QR
                    self._deadline_s = timestamp_s + self.config.qr_timeout_s
                else:
                    self._advance_step(timestamp_s)
            elif self._timed_out(timestamp_s):
                self._safe_stop(timestamp_s, "navigation timeout")

        elif self.state is MissionState.SEARCH_QR:
            if not self._global_sensors_fresh(timestamp_s):
                self._safe_stop(timestamp_s, "IMU or lidar became stale during QR search")
                return self.state
            search = getattr(self.runtime, "command_velocity_normalized", None)
            if callable(search):
                search(0, 0, self.config.qr_search_yaw_command)
            if self._timed_out(timestamp_s):
                self._safe_stop(timestamp_s, "QR search timeout")

        elif self.state is MissionState.BUILD_OBSTACLE_MAP:
            if self.task_code is not None and self._global_sensors_fresh(timestamp_s):
                self.steps = build_preliminary_steps(self.task_code, home_station=self.home_station)
                self.step_index = 0
                self._start_current_step(timestamp_s)
            elif self._timed_out(timestamp_s):
                self._safe_stop(timestamp_s, "no fresh lidar map after QR")

        elif self.state in {MissionState.FINE_ALIGN_MATERIAL, MissionState.FINE_ALIGN_RING}:
            if (
                self._alignment_count >= self.config.stable_alignment_frames
                and self._terminal_observation_fresh(timestamp_s)
                and self._tof_ready(timestamp_s)
            ):
                self._begin_manipulation(timestamp_s)
            elif self._timed_out(timestamp_s):
                self._retry_or_stop(timestamp_s, "terminal alignment or ToF timeout")

        elif self.state in {MissionState.PICK_MATERIAL, MissionState.PLACE_OR_STACK}:
            result = self.manipulator.poll(timestamp_s)
            if result.state is ActionState.SUCCEEDED and result.verified:
                self.state = (
                    MissionState.VERIFY_PICK
                    if self.state is MissionState.PICK_MATERIAL
                    else MissionState.VERIFY_PLACE
                )
            elif result.state is ActionState.FAILED or (
                result.state is ActionState.SUCCEEDED and not result.verified
            ):
                self._retry_or_stop(timestamp_s, result.error or "manipulator feedback not verified")
            elif self._timed_out(timestamp_s):
                self._retry_or_stop(timestamp_s, "manipulator action timeout")

        elif self.state in {MissionState.VERIFY_PICK, MissionState.VERIFY_PLACE}:
            current = self.current_step
            if current is not None:
                self.statistics.record_success(timestamp_s, current)
            self._advance_step(timestamp_s)

        return self.state
