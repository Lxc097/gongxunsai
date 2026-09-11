"""Receive-only STP-23L driver (development manual V0.1, PDF pp. 5-6).

No commands are sent: the datasheet and 2025 user manual reserve RX for
production. Device timestamps are raw uint32 values of unspecified unit/epoch;
freshness uses the host monotonic clock.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from statistics import median
import struct
import time
from typing import Callable, Mapping, Protocol


HEADER = b"\xaa\xaa\xaa\xaa"
POINT = struct.Struct("<hHIBIh")  # byte aligned; signed distance/reftof; 15 bytes
POINT_COUNT = 12
PAYLOAD_SIZE = POINT_COUNT * POINT.size + 4
MAX_PAYLOAD_SIZE = 320
BAUDRATE = 230400


class Readable(Protocol):
    def read(self, size: int = 1) -> bytes: ...


@dataclass(frozen=True)
class Stp23lPoint:
    distance_mm: int
    noise: int
    peak: int
    confidence: int
    integration: int
    reftof: int  # temperature-change indicator, NOT degrees Celsius


@dataclass(frozen=True)
class Stp23lPacket:
    points: tuple[Stp23lPoint, ...]
    device_timestamp: int


@dataclass
class Stp23lStatistics:
    received_bytes: int = 0
    measurement_packets: int = 0
    checksum_errors: int = 0
    length_errors: int = 0
    ignored_packets: int = 0
    discarded_bytes: int = 0
    backlog_events: int = 0


class Stp23lDecoder:
    """Consume a bytearray in place; retain at most 330 partial-frame bytes.

    Decode only default 12-point, address=0, offset=0 measurements. Other
    checksum-valid packets are consumed/countable, never treated as distance.
    """

    def __init__(self) -> None:
        self.statistics = Stp23lStatistics()

    def __call__(self, buffer: bytearray) -> list[Stp23lPacket]:
        packets: list[Stp23lPacket] = []
        cursor = 0
        while True:
            start = buffer.find(HEADER, cursor)
            if start < 0:
                keep = 0
                while keep < min(3, len(buffer) - cursor) and buffer[-keep - 1] == 0xAA:
                    keep += 1
                self.statistics.discarded_bytes += len(buffer) - cursor - keep
                cursor = len(buffer) - keep
                break
            self.statistics.discarded_bytes += start - cursor
            cursor = start
            if len(buffer) - cursor < 10:
                break
            address, command, offset, length = struct.unpack_from("<BBHH", buffer, cursor + 4)
            if length > MAX_PAYLOAD_SIZE or (command == 0x02 and length != PAYLOAD_SIZE):
                self.statistics.length_errors += 1
                self.statistics.discarded_bytes += 1
                cursor += 1
                continue
            end = cursor + 11 + length
            if len(buffer) < end:
                break
            if sum(buffer[cursor + 4:end - 1]) & 0xFF != buffer[end - 1]:
                self.statistics.checksum_errors += 1
                self.statistics.discarded_bytes += 1
                cursor += 1
                continue
            if command == 0x02 and address == 0 and offset == 0:
                points = tuple(
                    Stp23lPoint(*POINT.unpack_from(buffer, cursor + 10 + i * POINT.size))
                    for i in range(POINT_COUNT)
                )
                timestamp = struct.unpack_from("<I", buffer, end - 5)[0]
                packets.append(Stp23lPacket(points, timestamp))
                self.statistics.measurement_packets += 1
            else:
                self.statistics.ignored_packets += 1
            cursor = end
        del buffer[:cursor]
        return packets


@dataclass(frozen=True)
class Stp23lConfig:
    # Conservative floor: range table says 0.07 m; accuracy text also says 0.03 m.
    minimum_mm: int = 70
    maximum_mm: int = 7500
    window: int = 7
    max_jump_mm: int = 120
    stable_samples: int = 3  # independent packets, NOT points in one packet
    stable_tolerance_mm: int = 20
    minimum_valid_points: int = 6
    minimum_confidence: int | None = None  # no vendor threshold is documented
    stale_timeout_s: float = 0.5
    read_timeout_s: float = 0.02

    def __post_init__(self) -> None:
        for name in ("minimum_mm", "maximum_mm", "window", "max_jump_mm",
                     "stable_samples", "stable_tolerance_mm", "minimum_valid_points"):
            if type(getattr(self, name)) is not int:
                raise ValueError(f"stp23l.{name} must be an integer")
        if not 30 <= self.minimum_mm < self.maximum_mm <= 7500:
            raise ValueError("stp23l range must satisfy 30 <= minimum < maximum <= 7500 mm")
        if self.window < 3 or self.window % 2 == 0:
            raise ValueError("median window must be an odd number >= 3")
        if not 2 <= self.stable_samples <= self.window:
            raise ValueError("stable_samples must be in 2..window")
        if not 0 <= self.stable_tolerance_mm <= self.max_jump_mm or self.max_jump_mm <= 0:
            raise ValueError("invalid stp23l jump/stability tolerances")
        if not 1 <= self.minimum_valid_points <= POINT_COUNT:
            raise ValueError("minimum_valid_points must be in 1..12")
        if self.minimum_confidence is not None and (
            type(self.minimum_confidence) is not int or not 0 <= self.minimum_confidence <= 255
        ):
            raise ValueError("minimum_confidence must be null or an integer in 0..255")
        for name in ("stale_timeout_s", "read_timeout_s"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"stp23l.{name} must be finite and positive")
        if self.read_timeout_s >= self.stale_timeout_s:
            raise ValueError("read_timeout_s must be shorter than stale_timeout_s")

    @classmethod
    def from_mapping(cls, section: Mapping[str, object]) -> Stp23lConfig:
        if section.get("baudrate", BAUDRATE) != BAUDRATE:
            raise ValueError("STP-23L requires 230400 baud, 8-N-1, no flow control")
        return cls(**{name: section[name] for name in cls.__dataclass_fields__ if name in section})


class DistanceFilter:
    """Median, stationary-window gate, and recoverable step-change rejection."""

    def __init__(self, window: int = 7, *, minimum_mm: int = 70, maximum_mm: int = 7500,
                 max_jump_mm: int = 120, stable_samples: int = 3,
                 stable_tolerance_mm: int = 20) -> None:
        Stp23lConfig(window=window, minimum_mm=minimum_mm, maximum_mm=maximum_mm,
                     max_jump_mm=max_jump_mm, stable_samples=stable_samples,
                     stable_tolerance_mm=stable_tolerance_mm)
        self.values: deque[int] = deque(maxlen=window)
        self._candidates: deque[int] = deque(maxlen=stable_samples)
        self.minimum_mm = minimum_mm
        self.maximum_mm = maximum_mm
        self.max_jump_mm = max_jump_mm
        self.stable_samples = stable_samples
        self.stable_tolerance_mm = stable_tolerance_mm
        self._accepted = 0

    def reset(self) -> None:
        self.values.clear()
        self._candidates.clear()
        self._accepted = 0

    def update(self, distance_mm: int | None) -> tuple[int | None, bool]:
        if type(distance_mm) is not int or not self.minimum_mm <= distance_mm <= self.maximum_mm:
            self.reset()
            return None, False
        if self.values and abs(distance_mm - median(self.values)) > self.max_jump_mm:
            self._accepted = 0
            self._candidates.append(distance_mm)
            if max(self._candidates) - min(self._candidates) > self.stable_tolerance_mm:
                self._candidates.clear()
                self._candidates.append(distance_mm)
            if len(self._candidates) < self.stable_samples:
                return None, False
            # Reacquire a persistent surface, then require new confirmations.
            self.values.clear()
            self.values.extend(self._candidates)
            self._candidates.clear()
            return int(round(median(self.values))), False
        self._candidates.clear()
        self.values.append(distance_mm)
        self._accepted += 1
        stable = (self._accepted >= self.stable_samples
                  and max(self.values) - min(self.values) <= self.stable_tolerance_mm)
        return int(round(median(self.values))), stable


@dataclass(frozen=True)
class Stp23lReading:
    distance_mm: int | None
    stable: bool
    timestamp_s: float  # host receipt time, NOT the sensor's acquisition time
    reason: str
    packet: Stp23lPacket | None = None


class Stp23lReader:
    """Read-only adapter. Twelve points form one temporal observation.

    Only the newest queued packet is filtered; backlog cannot manufacture
    confirmations. ``poll_reading`` returns cached readings with their ORIGINAL
    receipt time, or an invalid reading on errors/staleness. Optional legacy
    distance decoders remain supported, but should migrate to the built-in one.
    """

    def __init__(self, stream: Readable,
                 frame_decoder: Callable[[bytearray], list[int]] | None = None,
                 distance_filter: DistanceFilter | None = None, *,
                 config: Stp23lConfig | None = None,
                 clock: Callable[[], float] = time.monotonic) -> None:
        self.stream = stream
        self.config = config or Stp23lConfig()
        self.frame_decoder = frame_decoder
        self.decoder = Stp23lDecoder()
        self.filter = distance_filter or DistanceFilter(
            window=self.config.window, minimum_mm=self.config.minimum_mm,
            maximum_mm=self.config.maximum_mm, max_jump_mm=self.config.max_jump_mm,
            stable_samples=self.config.stable_samples,
            stable_tolerance_mm=self.config.stable_tolerance_mm,
        )
        self.buffer = bytearray()
        self.clock = clock
        self._last_poll_s: float | None = None
        self._last_chunk_s: float | None = None
        self._last_packet_s: float | None = None
        self._reading: Stp23lReading | None = None

    @classmethod
    def open_serial(cls, port: str, *, config: Stp23lConfig | None = None) -> Stp23lReader:
        import serial  # tests and binary replay use only the standard library
        settings = config or Stp23lConfig()
        stream = serial.Serial(port=port, baudrate=BAUDRATE, bytesize=8, parity="N",
                               stopbits=1, timeout=settings.read_timeout_s,
                               xonxoff=False, rtscts=False, dsrdtr=False)
        try:
            stream.reset_input_buffer()
            return cls(stream, config=settings)
        except BaseException:
            stream.close()
            raise

    def _invalid(self, now: float, reason: str, packet: Stp23lPacket | None = None) -> Stp23lReading:
        self.filter.reset()
        self._reading = Stp23lReading(None, False, now, reason, packet)
        return self._reading

    def poll_reading(self) -> Stp23lReading:
        now = self.clock()
        if self._last_poll_s is not None and now - self._last_poll_s > self.config.stale_timeout_s:
            self.filter.reset()
            self.buffer.clear()
            self._reading = None
        self._last_poll_s = now
        try:
            waiting = getattr(self.stream, "in_waiting", None)
            chunk = self.stream.read(min(max(waiting, 1), 4096) if waiting is not None else 256)
        except Exception:
            self.buffer.clear()
            self._invalid(self.clock(), "io_error")
            raise
        now = self.clock()
        self.decoder.statistics.received_bytes += len(chunk)
        if chunk:
            if self._last_chunk_s is not None and now - self._last_chunk_s > self.config.stale_timeout_s:
                self.buffer.clear()
            self._last_chunk_s = now
        # A long serial FIFO queue has no trustworthy acquisition time. Discard
        # it in bounded reads; subsequent independent packets must settle again.
        if waiting is not None and waiting > 2 * (PAYLOAD_SIZE + 11):
            self.decoder.statistics.backlog_events += 1
            self.decoder.statistics.discarded_bytes += len(chunk) + len(self.buffer)
            self.buffer.clear()
            return self._invalid(now, "backlog")
        self.buffer.extend(chunk)
        stats = self.decoder.statistics
        errors_before = (stats.checksum_errors, stats.length_errors, stats.discarded_bytes, stats.ignored_packets)
        if self.frame_decoder is not None:
            values = self.frame_decoder(self.buffer)
            if len(self.buffer) > MAX_PAYLOAD_SIZE + 10:
                self.buffer.clear()
                return self._invalid(now, "decoder_overflow")
            packet = None
            distance = values[-1] if values else None
            has_measurement = bool(values)
        else:
            packets = self.decoder(self.buffer)
            packet = packets[-1] if packets else None
            has_measurement = packet is not None
            distance = None
        protocol_error = errors_before != (
            stats.checksum_errors, stats.length_errors, stats.discarded_bytes, stats.ignored_packets
        )
        if protocol_error:
            self.filter.reset()
        if has_measurement:
            if self._last_packet_s is None or now - self._last_packet_s > self.config.stale_timeout_s:
                self.filter.reset()
            self._last_packet_s = now
            if packet is not None:
                distances = [p.distance_mm for p in packet.points
                             if self.config.minimum_mm <= p.distance_mm <= self.config.maximum_mm
                             and (self.config.minimum_confidence is None
                                  or p.confidence >= self.config.minimum_confidence)]
                if len(distances) < self.config.minimum_valid_points:
                    return self._invalid(now, "insufficient_valid_points", packet)
                if max(distances) - min(distances) > self.config.stable_tolerance_mm:
                    return self._invalid(now, "moving_or_mixed_surface", packet)
                distance = int(round(median(distances)))
            value, stable = self.filter.update(distance)
            self._reading = Stp23lReading(value, stable, now, "stable" if stable else "settling", packet)
        elif protocol_error:
            return self._invalid(now, "protocol_error")
        elif self._last_packet_s is None or now - self._last_packet_s > self.config.stale_timeout_s:
            return self._invalid(now, "waiting" if self._last_packet_s is None else "stale")
        return self._reading or self._invalid(now, "waiting")

    def poll(self) -> list[tuple[int | None, bool]]:
        previous = self._reading
        reading = self.poll_reading()
        return [] if reading is previous else [(reading.distance_mm, reading.stable)]

    def close(self) -> None:
        self.buffer.clear()
        self._invalid(self.clock(), "closed")
        close = getattr(self.stream, "close", None)
        if close is not None:
            close()
