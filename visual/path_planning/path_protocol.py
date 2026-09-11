"""Jetson 到 STM32 底盘的可靠串口协议。

帧格式（全部小端）：
``A5 5A | version:u8 | type:u8 | sequence:u16 | length:u16 |
payload | crc16_ccitt_false:u16``。

CRC 覆盖 version 至 payload，不包含帧头和 CRC 本身。动态定位模式使用
车体速度帧；动作批仅保留为离线导出和旧设备兼容格式。
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import math
import secrets
import struct
import time
from typing import Iterable, Protocol

try:
    from .path_planning import Action, ActionType
except ImportError:  # 支持直接运行 path_planning.py 时的同目录导入。
    from path_planning import Action, ActionType


WIRE_PROTOCOL_VERSION = 1
FRAME_HEAD = b"\xA5\x5A"
ACTION_VALUE_SCALE = 100
MAX_ACTIONS = 64
MAX_ACTION_PAYLOAD_SIZE = 2 + MAX_ACTIONS * 5
_HEADER_FORMAT = "<BBHH"
_HEADER_SIZE = struct.calcsize(_HEADER_FORMAT)


class PathProtocolError(RuntimeError):
    """串口帧或传输流程不符合协议。"""


class PathMessageType(IntEnum):
    ACTION_BATCH = 0x01
    CHASSIS_VELOCITY = 0x02
    CHASSIS_STOP = 0x03
    ACK = 0x80
    NACK = 0x81
    STATUS = 0x82


class PathAckCode(IntEnum):
    ACCEPTED = 0
    DUPLICATE = 1


class PathNackCode(IntEnum):
    BAD_CRC = 1
    BAD_VERSION = 2
    BAD_MESSAGE = 3
    BAD_LENGTH = 4
    BAD_ACTION = 5
    BUSY = 6
    RX_OVERFLOW = 7
    FRAME_TIMEOUT = 8


class PathExecutionState(IntEnum):
    WAITING = 0
    READY = 1
    RUNNING = 2
    COMPLETED = 3
    RECEIVE_TIMEOUT = 4
    FAILED = 5


@dataclass(frozen=True)
class Frame:
    version: int
    message_type: int
    sequence: int
    payload: bytes


@dataclass(frozen=True)
class Ack:
    sequence: int
    code: PathAckCode
    execution_state: PathExecutionState


@dataclass(frozen=True)
class Nack:
    sequence: int
    code: PathNackCode
    execution_state: PathExecutionState


@dataclass(frozen=True)
class ExecutionStatus:
    sequence: int
    state: PathExecutionState
    current_action_index: int
    action_count: int


@dataclass(frozen=True)
class UploadReceipt:
    sequence: int
    acknowledgement: Ack
    attempts: int


@dataclass(frozen=True)
class ChassisVelocity:
    """车体速度指令；x 向前、y 向左、z 逆时针，范围 -100..100。"""

    x: int
    y: int
    z: int
    watchdog_ms: int = 300


class ByteStream(Protocol):
    def read(self, size: int = 1) -> bytes: ...

    def write(self, data: bytes) -> int | None: ...


def crc16_ccitt_false(data: bytes, initial: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE：poly=0x1021、init=0xFFFF、无反射。"""

    crc = initial
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode_frame(message_type: int | PathMessageType, sequence: int, payload: bytes = b"") -> bytes:
    if not 0 <= sequence <= 0xFFFF:
        raise ValueError("序号必须在 0 到 65535 之间")
    if len(payload) > 0xFFFF:
        raise ValueError("帧载荷超过 65535 字节")
    body = struct.pack(
        _HEADER_FORMAT,
        WIRE_PROTOCOL_VERSION,
        int(message_type),
        sequence,
        len(payload),
    ) + payload
    return FRAME_HEAD + body + struct.pack("<H", crc16_ccitt_false(body))


def decode_frame(data: bytes) -> Frame:
    if len(data) < len(FRAME_HEAD) + _HEADER_SIZE + 2 or not data.startswith(FRAME_HEAD):
        raise PathProtocolError("无效帧头或帧长度")
    version, message_type, sequence, length = struct.unpack_from(_HEADER_FORMAT, data, 2)
    expected_length = len(FRAME_HEAD) + _HEADER_SIZE + length + 2
    if len(data) != expected_length:
        raise PathProtocolError("帧长度字段与实际数据不一致")
    expected_crc = struct.unpack_from("<H", data, expected_length - 2)[0]
    actual_crc = crc16_ccitt_false(data[2:-2])
    if actual_crc != expected_crc:
        raise PathProtocolError("CRC 校验失败")
    return Frame(version, message_type, sequence, data[2 + _HEADER_SIZE : -2])


class FrameDecoder:
    """可处理任意分段串口读取的增量解码器。"""

    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, data: bytes) -> list[Frame]:
        self._buffer.extend(data)
        frames: list[Frame] = []
        while True:
            head_index = self._buffer.find(FRAME_HEAD)
            if head_index < 0:
                self._buffer[:] = self._buffer[-1:]
                return frames
            if head_index > 0:
                del self._buffer[:head_index]
            if len(self._buffer) < len(FRAME_HEAD) + _HEADER_SIZE:
                return frames
            _, _, _, length = struct.unpack_from(_HEADER_FORMAT, self._buffer, 2)
            if length > MAX_ACTION_PAYLOAD_SIZE:
                del self._buffer[0]
                continue
            total_length = len(FRAME_HEAD) + _HEADER_SIZE + length + 2
            if len(self._buffer) < total_length:
                return frames
            candidate = bytes(self._buffer[:total_length])
            del self._buffer[:total_length]
            try:
                frames.append(decode_frame(candidate))
            except PathProtocolError:
                # 坏帧已经完整消费；继续寻找后面的帧头。
                continue


def _scaled_action_value(action: Action) -> int:
    if not math.isfinite(action.value):
        raise PathProtocolError("动作值必须是有限数")
    scaled = int(round(action.value * ACTION_VALUE_SCALE))
    if not math.isclose(
        scaled / ACTION_VALUE_SCALE,
        action.value,
        abs_tol=0.000_001,
    ):
        raise PathProtocolError("动作值最多保留两位小数")
    return scaled


def _validate_action(action: Action, index: int, action_count: int) -> tuple[int, int]:
    code = int(ActionType[action.action])
    scaled = _scaled_action_value(action)
    if code in (ActionType.SEGMENT_END, ActionType.CAR_STOP):
        if scaled != 0:
            raise PathProtocolError("停靠和全任务停止动作的值必须为 0")
    elif scaled <= 0:
        raise PathProtocolError("移动和转向动作必须大于 0")
    elif code in (ActionType.MOVE_FORWARD, ActionType.MOVE_BACKWARD) and scaled > 500_000:
        raise PathProtocolError("单次直行动作不能超过 5000 mm")
    elif code in (ActionType.TURN_LEFT, ActionType.TURN_RIGHT) and scaled > 36_000:
        raise PathProtocolError("单次转向动作不能超过 360 度")
    if (code == ActionType.CAR_STOP) != (index == action_count - 1):
        raise PathProtocolError("CAR_STOP 必须且只能位于动作批末尾")
    return code, scaled


def encode_action_batch(actions: Iterable[Action], sequence: int) -> bytes:
    action_list = list(actions)
    if not 1 <= len(action_list) <= MAX_ACTIONS:
        raise PathProtocolError(f"动作数量必须在 1 到 {MAX_ACTIONS} 之间")
    payload = bytearray(struct.pack("<H", len(action_list)))
    for index, action in enumerate(action_list):
        code, scaled_value = _validate_action(action, index, len(action_list))
        payload.extend(struct.pack("<Bi", code, scaled_value))
    return encode_frame(PathMessageType.ACTION_BATCH, sequence, bytes(payload))


def encode_chassis_velocity(command: ChassisVelocity, sequence: int) -> bytes:
    """编码 Jetson 闭环控制输出；STM32 超过 watchdog_ms 未刷新即停车。"""

    if not all(-100 <= value <= 100 for value in (command.x, command.y, command.z)):
        raise PathProtocolError("车体速度 x/y/z 必须在 -100..100")
    if not 50 <= command.watchdog_ms <= 1000:
        raise PathProtocolError("速度看门狗必须在 50..1000 ms")
    payload = struct.pack("<hhhH", command.x, command.y, command.z, command.watchdog_ms)
    return encode_frame(PathMessageType.CHASSIS_VELOCITY, sequence, payload)


def decode_chassis_velocity(frame: Frame) -> ChassisVelocity:
    if frame.version != WIRE_PROTOCOL_VERSION:
        raise PathProtocolError("不支持的协议版本")
    if frame.message_type != PathMessageType.CHASSIS_VELOCITY or len(frame.payload) != 8:
        raise PathProtocolError("不是有效的车体速度帧")
    command = ChassisVelocity(*struct.unpack("<hhhH", frame.payload))
    if not all(-100 <= value <= 100 for value in (command.x, command.y, command.z)):
        raise PathProtocolError("车体速度字段越界")
    if not 50 <= command.watchdog_ms <= 1000:
        raise PathProtocolError("速度看门狗字段越界")
    return command


def encode_chassis_stop(sequence: int) -> bytes:
    return encode_frame(PathMessageType.CHASSIS_STOP, sequence)


def decode_action_batch(frame: Frame) -> list[Action]:
    if frame.version != WIRE_PROTOCOL_VERSION:
        raise PathProtocolError("不支持的协议版本")
    if frame.message_type != PathMessageType.ACTION_BATCH or len(frame.payload) < 2:
        raise PathProtocolError("不是有效的动作批帧")
    action_count = struct.unpack_from("<H", frame.payload)[0]
    if not 1 <= action_count <= MAX_ACTIONS or len(frame.payload) != 2 + action_count * 5:
        raise PathProtocolError("动作批长度无效")
    actions: list[Action] = []
    for index in range(action_count):
        code, scaled_value = struct.unpack_from("<Bi", frame.payload, 2 + index * 5)
        try:
            action_type = ActionType(code)
        except ValueError as exc:
            raise PathProtocolError(f"未知动作编码：{code}") from exc
        action = Action(action_type.name, scaled_value / ACTION_VALUE_SCALE)
        _validate_action(action, index, action_count)
        actions.append(action)
    return actions


def decode_response(frame: Frame) -> Ack | Nack | ExecutionStatus:
    if frame.version != WIRE_PROTOCOL_VERSION:
        raise PathProtocolError("回执协议版本不匹配")
    try:
        message_type = PathMessageType(frame.message_type)
    except ValueError as exc:
        raise PathProtocolError(f"未知回执类型：{frame.message_type}") from exc
    try:
        if message_type == PathMessageType.ACK and len(frame.payload) == 2:
            return Ack(frame.sequence, PathAckCode(frame.payload[0]), PathExecutionState(frame.payload[1]))
        if message_type == PathMessageType.NACK and len(frame.payload) == 2:
            return Nack(frame.sequence, PathNackCode(frame.payload[0]), PathExecutionState(frame.payload[1]))
        if message_type == PathMessageType.STATUS and len(frame.payload) == 5:
            current, total = struct.unpack_from("<HH", frame.payload, 1)
            return ExecutionStatus(frame.sequence, PathExecutionState(frame.payload[0]), current, total)
    except ValueError as exc:
        raise PathProtocolError("回执字段无效") from exc
    raise PathProtocolError("回执载荷长度无效")


class SerialPathClient:
    """带 ACK 超时、重传和重复序号幂等性的上位机发送器。"""

    def __init__(self, stream: ByteStream, *, ack_timeout_s: float = 0.5, retries: int = 3) -> None:
        if ack_timeout_s <= 0 or retries < 0:
            raise ValueError("ack_timeout_s 必须大于 0，retries 不能为负数")
        self.stream = stream
        self.ack_timeout_s = ack_timeout_s
        self.retries = retries
        # 新的上位机会话从随机序号开始，避免设备重启前的最后一个序号被误判为重传。
        self._next_sequence = secrets.randbelow(0x10000)
        self._decoder = FrameDecoder()

    def _read_responses(self, sequence: int) -> Ack | Nack | None:
        deadline = time.monotonic() + self.ack_timeout_s
        while time.monotonic() < deadline:
            chunk = self.stream.read(64)
            if not chunk:
                continue
            for frame in self._decoder.feed(chunk):
                response = decode_response(frame)
                if isinstance(response, (Ack, Nack)) and response.sequence == sequence:
                    return response
        return None

    def upload_actions(self, actions: Iterable[Action], sequence: int | None = None) -> UploadReceipt:
        if sequence is None:
            sequence = self._next_sequence
            self._next_sequence = (self._next_sequence + 1) & 0xFFFF
        frame = encode_action_batch(actions, sequence)
        retryable_nacks = {
            PathNackCode.BAD_CRC,
            PathNackCode.RX_OVERFLOW,
            PathNackCode.FRAME_TIMEOUT,
        }
        for attempt in range(1, self.retries + 2):
            self.stream.write(frame)
            flush = getattr(self.stream, "flush", None)
            if callable(flush):
                flush()
            response = self._read_responses(sequence)
            if isinstance(response, Ack):
                return UploadReceipt(sequence, response, attempt)
            if isinstance(response, Nack) and response.code not in retryable_nacks:
                raise PathProtocolError(f"底盘拒绝动作批：{response.code.name}")
        raise PathProtocolError(f"动作批 seq={sequence} 在 {self.retries + 1} 次发送后未获确认")


def upload_actions_to_serial(
    actions: Iterable[Action],
    port: str,
    *,
    baudrate: int = 115200,
    ack_timeout_s: float = 0.5,
    retries: int = 3,
) -> UploadReceipt:
    """通过 pyserial 打开串口并可靠下发一次动作批。"""

    try:
        import serial
    except ImportError as exc:
        raise PathProtocolError("串口下发需要安装 pyserial：python -m pip install pyserial") from exc

    with serial.Serial(port, baudrate=baudrate, timeout=min(ack_timeout_s, 0.1)) as stream:
        return SerialPathClient(
            stream,
            ack_timeout_s=ack_timeout_s,
            retries=retries,
        ).upload_actions(actions)
