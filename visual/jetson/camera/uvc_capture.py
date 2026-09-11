"""Reliable V4L2/UVC frame capture for the front RGB camera."""

from __future__ import annotations

from dataclasses import dataclass
import queue
import threading
import time
from typing import Any, Callable, Mapping


class CameraConfigurationError(ValueError):
    """The front-camera configuration is missing or unsafe."""


class CameraOpenError(RuntimeError):
    """The configured camera could not be opened with the requested mode."""


class CameraReadError(RuntimeError):
    """No frame could be read after the bounded recovery attempts."""


@dataclass(frozen=True)
class UvcCameraConfig:
    device: str
    width: int = 1280
    height: int = 720
    fps: float = 30.0
    pixel_format: str = "MJPG"
    frame_timeout_s: float = 0.5
    reconnect_attempts: int = 3
    reconnect_delay_s: float = 0.25

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "UvcCameraConfig":
        if not isinstance(value, Mapping):
            raise CameraConfigurationError("front_rgb_camera must be an object")
        device = value.get("device")
        if not isinstance(device, str) or not device.strip():
            raise CameraConfigurationError("front_rgb_camera.device must be a non-empty path")
        config = cls(
            device=device.strip(),
            width=value.get("width", 1280),
            height=value.get("height", 720),
            fps=value.get("fps", 30.0),
            pixel_format=value.get("pixel_format", "MJPG"),
            frame_timeout_s=value.get("frame_timeout_s", 0.5),
            reconnect_attempts=value.get("reconnect_attempts", 3),
            reconnect_delay_s=value.get("reconnect_delay_s", 0.25),
        )
        config.validate()
        return config

    def validate(self) -> None:
        if not isinstance(self.width, int) or isinstance(self.width, bool) or self.width <= 0:
            raise CameraConfigurationError("camera width must be a positive integer")
        if not isinstance(self.height, int) or isinstance(self.height, bool) or self.height <= 0:
            raise CameraConfigurationError("camera height must be a positive integer")
        if not isinstance(self.fps, (int, float)) or isinstance(self.fps, bool) or self.fps <= 0:
            raise CameraConfigurationError("camera fps must be positive")
        if not isinstance(self.pixel_format, str) or len(self.pixel_format) != 4:
            raise CameraConfigurationError("camera pixel_format must contain four characters")
        if self.pixel_format.upper() != "MJPG":
            raise CameraConfigurationError("front RGB camera must use MJPG on USB 2.0")
        if not isinstance(self.frame_timeout_s, (int, float)) or self.frame_timeout_s <= 0:
            raise CameraConfigurationError("camera frame_timeout_s must be positive")
        if (
            not isinstance(self.reconnect_attempts, int)
            or isinstance(self.reconnect_attempts, bool)
            or self.reconnect_attempts < 0
        ):
            raise CameraConfigurationError("camera reconnect_attempts must be a non-negative integer")
        if not isinstance(self.reconnect_delay_s, (int, float)) or self.reconnect_delay_s < 0:
            raise CameraConfigurationError("camera reconnect_delay_s cannot be negative")


@dataclass(frozen=True)
class CameraFrame:
    image: Any
    timestamp_s: float
    sequence: int


@dataclass
class UvcCaptureStatistics:
    frames: int = 0
    empty_frames: int = 0
    open_failures: int = 0
    reconnects: int = 0
    actual_width: int = 0
    actual_height: int = 0
    actual_fps: float = 0.0
    actual_pixel_format: str = ""
    last_frame_s: float | None = None


def _decode_fourcc(value: float) -> str:
    integer = int(value)
    if integer <= 0:
        return ""
    return "".join(chr((integer >> (8 * index)) & 0xFF) for index in range(4)).rstrip("\x00")


class UvcCapture:
    """Own one OpenCV V4L2 handle and recover from bounded read failures."""

    def __init__(
        self,
        config: UvcCameraConfig,
        cv2_module: Any | None = None,
        *,
        clock: Callable[[], float] = time.monotonic,
        sleeper: Callable[[float], None] = time.sleep,
        capture_factory: Callable[[str, int], Any] | None = None,
    ) -> None:
        config.validate()
        if cv2_module is None:
            try:
                import cv2 as cv2_module
            except ImportError as exc:
                raise CameraOpenError("front RGB capture requires opencv-python") from exc
        self.config = config
        self.cv2 = cv2_module
        self._clock = clock
        self._sleep = sleeper
        self._capture_factory = capture_factory
        self._capture: Any | None = None
        self._sequence = 0
        self.statistics = UvcCaptureStatistics()

    @property
    def is_open(self) -> bool:
        if self._capture is None:
            return False
        try:
            return bool(self._capture.isOpened())
        except Exception:
            return False

    def _new_capture(self) -> Any:
        backend = getattr(self.cv2, "CAP_V4L2", 0)
        if self._capture_factory is not None:
            return self._capture_factory(self.config.device, backend)
        return self.cv2.VideoCapture(self.config.device, backend)

    def _set(self, property_name: str, value: float) -> None:
        property_id = getattr(self.cv2, property_name, None)
        if property_id is not None:
            self._capture.set(property_id, value)

    def _get(self, property_name: str) -> float:
        property_id = getattr(self.cv2, property_name, None)
        if property_id is None:
            return 0.0
        try:
            return float(self._capture.get(property_id))
        except Exception:
            return 0.0

    def open(self) -> None:
        self.close()
        try:
            self._capture = self._new_capture()
        except Exception as exc:
            self.statistics.open_failures += 1
            raise CameraOpenError(f"cannot create camera {self.config.device}: {exc}") from exc
        if not self.is_open:
            self.statistics.open_failures += 1
            self.close()
            raise CameraOpenError(f"cannot open camera {self.config.device}")

        fourcc = self.cv2.VideoWriter_fourcc(*self.config.pixel_format.upper())
        self._set("CAP_PROP_FOURCC", float(fourcc))
        self._set("CAP_PROP_FRAME_WIDTH", float(self.config.width))
        self._set("CAP_PROP_FRAME_HEIGHT", float(self.config.height))
        self._set("CAP_PROP_FPS", float(self.config.fps))
        self._set("CAP_PROP_BUFFERSIZE", 1.0)

        actual_width = int(round(self._get("CAP_PROP_FRAME_WIDTH")))
        actual_height = int(round(self._get("CAP_PROP_FRAME_HEIGHT")))
        actual_fps = self._get("CAP_PROP_FPS")
        actual_format = _decode_fourcc(self._get("CAP_PROP_FOURCC"))
        self.statistics.actual_width = actual_width
        self.statistics.actual_height = actual_height
        self.statistics.actual_fps = actual_fps
        self.statistics.actual_pixel_format = actual_format

        failures = []
        if actual_width and actual_width != self.config.width:
            failures.append(f"width={actual_width}")
        if actual_height and actual_height != self.config.height:
            failures.append(f"height={actual_height}")
        if actual_format and actual_format != self.config.pixel_format.upper():
            failures.append(f"pixel_format={actual_format}")
        if actual_fps > 0.5 and actual_fps + 0.5 < float(self.config.fps):
            failures.append(f"fps={actual_fps:g}")
        if failures:
            requested = (
                f"{self.config.width}x{self.config.height} "
                f"{self.config.pixel_format.upper()} {self.config.fps:g}fps"
            )
            self.close()
            raise CameraOpenError(
                f"camera rejected requested mode {requested}; actual " + ", ".join(failures)
            )

    def _read_once(self) -> CameraFrame | None:
        try:
            ok, image = self._capture.read()
        except Exception:
            ok, image = False, None
        if not ok or image is None:
            self.statistics.empty_frames += 1
            return None
        timestamp = self._clock()
        self._sequence += 1
        self.statistics.frames += 1
        self.statistics.last_frame_s = timestamp
        return CameraFrame(image, timestamp, self._sequence)

    def read(self) -> CameraFrame:
        last_error: Exception | None = None
        for attempt in range(self.config.reconnect_attempts + 1):
            if not self.is_open:
                try:
                    self.open()
                except CameraOpenError as exc:
                    last_error = exc
                else:
                    frame = self._read_once()
                    if frame is not None:
                        return frame
            else:
                frame = self._read_once()
                if frame is not None:
                    return frame

            self.close()
            if attempt < self.config.reconnect_attempts:
                self.statistics.reconnects += 1
                self._sleep(float(self.config.reconnect_delay_s))

        detail = f": {last_error}" if last_error is not None else ""
        raise CameraReadError(
            f"no frame after {self.config.reconnect_attempts} reconnect attempts{detail}"
        )

    def close(self) -> None:
        capture, self._capture = self._capture, None
        if capture is not None:
            try:
                capture.release()
            except Exception:
                pass

    def __enter__(self) -> "UvcCapture":
        self.open()
        return self

    def __exit__(self, _type: Any, _value: Any, _traceback: Any) -> None:
        self.close()


class UvcFrameWorker:
    """Continuously publish only the newest frame from a ``UvcCapture``."""

    def __init__(self, capture: UvcCapture) -> None:
        self.capture = capture
        self._frames: queue.Queue[CameraFrame] = queue.Queue(maxsize=1)
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._fault: BaseException | None = None

    @property
    def fault(self) -> BaseException | None:
        return self._fault

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            raise RuntimeError("camera worker is already running")
        self._fault = None
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, name="front-rgb-reader", daemon=True)
        self._thread.start()

    def _publish(self, frame: CameraFrame) -> None:
        try:
            self._frames.put_nowait(frame)
        except queue.Full:
            try:
                self._frames.get_nowait()
            except queue.Empty:
                pass
            self._frames.put_nowait(frame)

    def _run(self) -> None:
        try:
            while not self._stop_event.is_set():
                self._publish(self.capture.read())
        except BaseException as exc:
            self._fault = exc
            self._stop_event.set()
        finally:
            self.capture.close()

    def wait_for_frame(self, timeout: float | None = None) -> CameraFrame:
        wait = self.capture.config.frame_timeout_s if timeout is None else timeout
        try:
            return self._frames.get(timeout=wait)
        except queue.Empty as exc:
            if self._fault is not None:
                raise CameraReadError(f"camera worker failed: {self._fault}") from self._fault
            raise CameraReadError(f"no fresh camera frame within {wait:.3f} s") from exc

    def stop(self, timeout: float = 2.0) -> None:
        self._stop_event.set()
        self.capture.close()
        if self._thread is not None:
            self._thread.join(timeout)

