from __future__ import annotations

import json
import math
import struct
from dataclasses import dataclass, replace
from typing import Any, Mapping

from jetson.models import ImuSample


@dataclass(frozen=True)
class ImuConfig:
    """Yahboom six-axis IMU installed in the vehicle body frame."""

    protocol: str = "yahboom_native"
    variant: str = "six_axis"
    mounting: str = "x_forward_y_left_z_up"
    yaw_sign: int = 1
    yaw_sign_verified: bool = False
    max_tilt_deg: float = 15.0
    max_gap_s: float = 0.5
    settle_s: float = 1.0
    calibration_duration_s: float = 2.0

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> ImuConfig:
        fields = cls.__dataclass_fields__
        config = cls(**{key: value[key] for key in fields if key in value})
        if config.protocol not in ("yahboom_native", "json_adapter"):
            raise ValueError("imu.protocol 必须是 yahboom_native 或 json_adapter")
        if config.variant != "six_axis":
            raise ValueError("当前仅支持 imu.variant=six_axis")
        if config.mounting != "x_forward_y_left_z_up":
            raise ValueError("当前安装必须为 x_forward_y_left_z_up（车头 +X、车左 +Y、朝上 +Z）")
        if type(config.yaw_sign) is not int or config.yaw_sign not in (-1, 1):
            raise ValueError("imu.yaw_sign 必须为整数 +1 或 -1")
        if type(config.yaw_sign_verified) is not bool:
            raise ValueError("imu.yaw_sign_verified 必须为布尔值")
        for name in ("max_tilt_deg", "max_gap_s", "settle_s", "calibration_duration_s"):
            number = getattr(config, name)
            if isinstance(number, bool) or not isinstance(number, (int, float)) or not math.isfinite(number):
                raise ValueError(f"imu.{name} 必须为有限数字")
        if not 0 < config.max_tilt_deg <= 30 or not 0 < config.max_gap_s <= 0.5:
            raise ValueError("IMU max_tilt_deg 应为 (0,30]，max_gap_s 应为 (0,0.5]")
        if config.settle_s < 0 or not 1 <= config.calibration_duration_s <= 10:
            raise ValueError("IMU settle_s 应非负，calibration_duration_s 应为 1..10 秒")
        return config

    def require_supported_transport(self) -> None:
        if not self.yaw_sign_verified:
            raise ValueError("IMU yaw 正负方向尚未实测；完成原地逆时针转动车头测试后，"
                             "设置 imu.yaw_sign 为 +1/-1，并将 yaw_sign_verified 设为 true")


class YahboomImuDecoder:
    """Receive-only decoder for the native YbImuLib serial protocol.

    The module reports independent RAW and Euler frames. A sample is published
    only when a new Euler frame has a recent RAW gyro frame; cached getters and
    fabricated acquisition timestamps are deliberately avoided.
    """

    HEADER = b"\x7e\x23"
    MAX_FRAME_BYTES = 40
    MAX_RAW_EULER_GAP_S = 0.25
    FUNC_VERSION = 0x01
    FUNC_RAW = 0x04
    FUNC_EULER = 0x26
    GYRO_SCALE_DEG_S = 2000.0 / 32767.0

    def __init__(self) -> None:
        self._buffer = bytearray()
        self._gyro_z_deg_s: float | None = None
        self._gyro_timestamp_s: float | None = None
        self.valid_frames = 0
        self.rejected_frames = 0
        self.unpaired_euler_frames = 0
        self.firmware_version: str | None = None

    def feed(self, data: bytes, timestamp_s: float) -> ImuSample | None:
        if not math.isfinite(timestamp_s):
            raise ValueError("host timestamp must be finite")
        if not isinstance(data, (bytes, bytearray, memoryview)):
            raise TypeError("IMU data must be bytes-like")
        self._buffer.extend(data)
        latest: ImuSample | None = None
        while True:
            start = self._buffer.find(self.HEADER)
            if start < 0:
                # Keep a possible first header byte split across serial reads.
                if self._buffer[-1:] == self.HEADER[:1]:
                    del self._buffer[:-1]
                else:
                    self._buffer.clear()
                break
            if start:
                del self._buffer[:start]
            if len(self._buffer) < 3:
                break
            frame_bytes = self._buffer[2]
            if not 5 <= frame_bytes <= self.MAX_FRAME_BYTES:
                self.rejected_frames += 1
                del self._buffer[0]
                continue
            if len(self._buffer) < frame_bytes:
                break
            frame = bytes(self._buffer[:frame_bytes])
            if (sum(frame[:-1]) & 0xFF) != frame[-1]:
                self.rejected_frames += 1
                # Drop one byte, not the claimed frame, so an embedded valid
                # header can still be recovered after a corrupt length/payload.
                del self._buffer[0]
                continue
            del self._buffer[:frame_bytes]
            self.valid_frames += 1
            sample = self._parse_frame(frame[3], frame[4:-1], timestamp_s)
            if sample is not None:
                latest = sample
        return latest

    def _parse_frame(self, function: int, payload: bytes, timestamp_s: float) -> ImuSample | None:
        try:
            if function == self.FUNC_VERSION:
                if len(payload) < 3:
                    raise ValueError("short version frame")
                self.firmware_version = f"V{payload[0]}.{payload[1]}.{payload[2]}"
            elif function == self.FUNC_RAW:
                if len(payload) < 12:
                    raise ValueError("short RAW frame")
                raw_values = struct.unpack_from("<hhhhhh", payload)
                self._gyro_z_deg_s = raw_values[5] * self.GYRO_SCALE_DEG_S
                self._gyro_timestamp_s = timestamp_s
            elif function == self.FUNC_EULER:
                if len(payload) < 12:
                    raise ValueError("short Euler frame")
                roll_rad, pitch_rad, yaw_rad = struct.unpack_from("<fff", payload)
                values = (roll_rad, pitch_rad, yaw_rad)
                if not all(math.isfinite(value) for value in values):
                    raise ValueError("non-finite Euler data")
                raw_gap_s = (None if self._gyro_timestamp_s is None
                             else timestamp_s - self._gyro_timestamp_s)
                if (self._gyro_z_deg_s is None or raw_gap_s is None
                        or not 0.0 <= raw_gap_s <= self.MAX_RAW_EULER_GAP_S):
                    self.unpaired_euler_frames += 1
                    return None
                # Mounting is identity: sensor +X/+Y/+Z are vehicle
                # forward/left/up. yaw_sign is applied later by ImuFilter.
                return ImuSample(
                    yaw_deg=math.degrees(yaw_rad),
                    gyro_z_deg_s=self._gyro_z_deg_s,
                    roll_deg=math.degrees(roll_rad),
                    pitch_deg=math.degrees(pitch_rad),
                    timestamp_s=timestamp_s,
                )
        except (struct.error, ValueError, OverflowError):
            self.rejected_frames += 1
        return None


def parse_imu_json_line(data: bytes | str) -> ImuSample:
    """Adapter contract in degrees/deg/s, NOT the Yahboom module's wire format."""

    if isinstance(data, bytes):
        data = data.decode("utf-8")
    item = json.loads(data)
    if not isinstance(item, dict):
        raise ValueError("IMU line must be an object")
    fields = ("yaw_deg", "gyro_z_deg_s", "roll_deg", "pitch_deg", "timestamp_s")
    if any(isinstance(item.get(name), bool) for name in fields):
        raise ValueError("IMU values must be numbers, not booleans")
    values = [float(item[name]) for name in fields]
    if not all(math.isfinite(value) for value in values):
        raise ValueError("IMU line contains non-finite values")
    return ImuSample(*values)


class ImuJsonDecoder:
    """Bounded newline framing, fragmented reads, and source replay rejection.

    Return only the newest observation per read: a burst has no known host-side
    acquisition intervals. Never invent timestamps to integrate queued history.
    The adapter must timestamp each NEW sensor observation, not each cached poll.
    """

    MAX_LINE_BYTES = 1024

    def __init__(self) -> None:
        self._line = bytearray()
        self._discard_until_newline = False
        self._last_source_timestamp: float | None = None
        self.rejected_lines = 0

    def feed(self, data: bytes, timestamp_s: float) -> ImuSample | None:
        if not math.isfinite(timestamp_s):
            raise ValueError("host timestamp must be finite")
        latest = None
        for byte in data:
            if byte == 10:
                if not self._discard_until_newline and self._line:
                    try:
                        sample = parse_imu_json_line(bytes(self._line))
                        if (self._last_source_timestamp is not None
                                and sample.timestamp_s <= self._last_source_timestamp):
                            raise ValueError("repeated or reversed source timestamp")
                    except (UnicodeDecodeError, ValueError, KeyError, TypeError, OverflowError):
                        self.rejected_lines += 1
                    else:
                        self._last_source_timestamp = sample.timestamp_s
                        latest = replace(sample, timestamp_s=timestamp_s)
                self._line.clear()
                self._discard_until_newline = False
            elif not self._discard_until_newline:
                self._line.append(byte)
                if len(self._line) > self.MAX_LINE_BYTES:
                    self.rejected_lines += 1
                    self._line.clear()
                    self._discard_until_newline = True
        return latest
