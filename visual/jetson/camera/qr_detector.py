from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from statistics import median
from typing import Any


@dataclass(frozen=True)
class QrObservation:
    payload: str
    center_x_px: float
    center_y_px: float
    stable: bool = False


class QrConsensus:
    """Reject one-frame QR false positives before accepting a task code."""

    def __init__(self, stable_frames: int = 3, max_center_jump_px: float = 20.0) -> None:
        if stable_frames < 1 or max_center_jump_px <= 0:
            raise ValueError("invalid QR consensus parameters")
        self.stable_frames = stable_frames
        self.max_center_jump_px = max_center_jump_px
        self._history: deque[QrObservation] = deque(maxlen=stable_frames)

    def update(self, observation: QrObservation | None) -> QrObservation | None:
        if observation is None or not observation.payload.strip():
            self._history.clear()
            return None
        if self._history and self._history[-1].payload != observation.payload:
            self._history.clear()
        if self._history:
            previous = self._history[-1]
            jump = ((previous.center_x_px - observation.center_x_px) ** 2 +
                    (previous.center_y_px - observation.center_y_px) ** 2) ** 0.5
            if jump > self.max_center_jump_px:
                self._history.clear()
        self._history.append(observation)
        stable = len(self._history) == self.stable_frames
        return QrObservation(
            observation.payload,
            median(item.center_x_px for item in self._history),
            median(item.center_y_px for item in self._history),
            stable,
        )


class FrontRgbQrDetector:
    """OpenCV QR detector used with the front UVC RGB camera on Jetson."""

    def __init__(self, stable_frames: int = 3) -> None:
        try:
            import cv2
        except ImportError as exc:
            raise RuntimeError("front RGB camera QR recognition requires opencv-python") from exc
        self._cv2 = cv2
        self._detector = cv2.QRCodeDetector()
        self._consensus = QrConsensus(stable_frames=stable_frames)

    @staticmethod
    def _center(points: Any) -> tuple[float, float]:
        flattened = points.reshape(-1, 2)
        return (float(flattened[:, 0].mean()), float(flattened[:, 1].mean()))

    def detect(self, frame: Any) -> QrObservation | None:
        payload, points, _ = self._detector.detectAndDecode(frame)
        observation = None
        if payload and points is not None:
            x, y = self._center(points)
            observation = QrObservation(payload.strip(), x, y)
        return self._consensus.update(observation)


# Compatibility for older imports while the mission entry point is migrated.
RogQrDetector = FrontRgbQrDetector
