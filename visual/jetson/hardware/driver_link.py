from __future__ import annotations

from dataclasses import dataclass
import secrets
import time
from typing import Callable, Protocol

from path_planning.path_protocol import (
    Ack, ChassisVelocity, FrameDecoder, Nack, PathExecutionState,
    decode_response, encode_chassis_stop, encode_chassis_velocity,
)


class DriverLinkError(RuntimeError):
    """A latched driver-link fault; create a new session after operator review."""


@dataclass
class LinkStatistics:
    sent: int = 0
    acknowledged: int = 0
    timeouts: int = 0
    rejected: int = 0
    ignored_responses: int = 0
    io_errors: int = 0


class Writable(Protocol):
    def write(self, data: bytes) -> int: ...


class DriverLink:
    """Jetson-to-F407 stream on H27-3/H27-4 (USART6, 115200 8N1)."""

    def __init__(self, stream: Writable, *, watchdog_ms: int = 300,
                 require_ack: bool = False, ack_timeout_s: float = 0.12,
                 clock: Callable[[], float] = time.monotonic) -> None:
        if not 50 <= watchdog_ms <= 1000:
            raise ValueError("watchdog_ms must be in 50..1000")
        if ack_timeout_s <= 0 or (require_ack and ack_timeout_s >= watchdog_ms / 1000):
            raise ValueError("ACK timeout must be positive and shorter than watchdog")
        if require_ack and not callable(getattr(stream, "read", None)):
            raise TypeError("ACK mode requires a bounded-time read() stream")
        self.stream = stream
        self.watchdog_ms = watchdog_ms
        self.require_ack = require_ack  # False is only for offline/test sinks.
        self.ack_timeout_s = ack_timeout_s
        self._clock = clock
        self._sequence = secrets.randbelow(65536) if require_ack else 0
        self._decoder = FrameDecoder()
        self.statistics = LinkStatistics()
        self.fault_reason: str | None = None
        self._session_ready = not require_ack

    def _next(self) -> int:
        value = self._sequence
        self._sequence = (self._sequence + 1) & 0xFFFF
        return value

    def send_velocity(self, x: int, y: int, z: int) -> None:
        if self.fault_reason is not None:
            raise DriverLinkError(self.fault_reason)
        if not self._session_ready:
            raise DriverLinkError("STOP acknowledgement required before velocity")
        command = ChassisVelocity(x, y, z, self.watchdog_ms)
        sequence = self._next()
        self._exchange(encode_chassis_velocity(command, sequence), sequence, PathExecutionState.RUNNING)

    def stop(self) -> None:
        sequence = self._next()
        packet = encode_chassis_stop(sequence)
        if self.fault_reason is not None:
            # STOP is always permitted, but cannot clear the host fault latch.
            self._write(packet)
            return
        self._exchange(packet, sequence, PathExecutionState.WAITING)
        self._session_ready = True

    def _write(self, packet: bytes) -> None:
        try:
            written = self.stream.write(packet)
            if written != len(packet):
                raise OSError(f"short serial write: {written}/{len(packet)} bytes")
        except Exception:
            self.statistics.io_errors += 1
            raise
        self.statistics.sent += 1

    def _exchange(self, packet: bytes, sequence: int, expected_state: PathExecutionState) -> None:
        try:
            self._write(packet)
            if not self.require_ack:
                return
            deadline = self._clock() + self.ack_timeout_s
            while self._clock() < deadline:
                try:
                    data = self.stream.read(64)  # type: ignore[attr-defined]
                except Exception:
                    self.statistics.io_errors += 1
                    raise
                for frame in self._decoder.feed(data):
                    if frame.sequence != sequence:
                        self.statistics.ignored_responses += 1
                        continue
                    response = decode_response(frame)
                    if isinstance(response, Nack):
                        self.statistics.rejected += 1
                        raise DriverLinkError(f"F407 NACK seq={sequence}: {response.code.name}")
                    if not isinstance(response, Ack) or response.execution_state != expected_state:
                        raise DriverLinkError(f"unexpected F407 state for seq={sequence}: {response}")
                    self.statistics.acknowledged += 1
                    return
            self.statistics.timeouts += 1
            raise DriverLinkError(f"F407 ACK timeout seq={sequence}")
        except Exception as exc:
            self.fault_reason = str(exc)
            self._session_ready = False
            # No automatic velocity retry: stale motion must not be resurrected.
            try:
                self._write(encode_chassis_stop(self._next()))
            except Exception:
                pass  # Physical communication loss is covered by the MCU watchdog.
            raise DriverLinkError(self.fault_reason) from exc

    def close(self) -> None:
        close = getattr(self.stream, "close", None)
        if callable(close):
            close()

    @classmethod
    def open_serial(cls, port: str, *, baudrate: int = 115200, watchdog_ms: int = 300) -> "DriverLink":
        try:
            import serial
        except ImportError as exc:
            raise RuntimeError("driver serial link requires pyserial") from exc
        stream = serial.Serial(port, baudrate=baudrate, timeout=0.01, write_timeout=0.1)
        try:
            stream.reset_input_buffer()
            return cls(stream, watchdog_ms=watchdog_ms, require_ack=True,
                       ack_timeout_s=min(0.12, watchdog_ms / 2000))
        except Exception:
            stream.close()
            raise
