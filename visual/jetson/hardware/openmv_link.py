from __future__ import annotations

from dataclasses import dataclass
import json
import math


@dataclass(frozen=True)
class OpenMvObservation:
    material_color: str | None
    material_dx_px: float | None
    material_dy_px: float | None
    ring_dx_px: float | None
    ring_dy_px: float | None
    stable: bool
    material_stable: bool = False
    ring_stable: bool = False


def _optional_number(value: object) -> float | None:
    if value is None:
        return None
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("OpenMV coordinate is not finite")
    return result


def parse_openmv_line(data: bytes | str) -> OpenMvObservation:
    """Parse one UTF-8 JSON line produced by openmv/terminal_vision.py."""

    if isinstance(data, bytes):
        data = data.decode("utf-8")
    item = json.loads(data)
    if item.get("type") != "terminal":
        raise ValueError("not an OpenMV terminal observation")
    color = item.get("material_color")
    if color is not None and color not in {"red", "yellow", "blue", "green", "black", "cyan"}:
        raise ValueError("unknown material color")
    stable = bool(item.get("stable", False))
    return OpenMvObservation(
        color,
        _optional_number(item.get("material_dx_px")),
        _optional_number(item.get("material_dy_px")),
        _optional_number(item.get("ring_dx_px")),
        _optional_number(item.get("ring_dy_px")),
        stable,
        bool(item.get("material_stable", stable)),
        bool(item.get("ring_stable", stable)),
    )
