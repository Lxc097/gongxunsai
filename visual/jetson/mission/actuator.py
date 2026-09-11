from __future__ import annotations

from typing import Protocol

from jetson.hardware import OpenMvObservation

from .models import ActionResult, MaterialColor


class MissionManipulator(Protocol):
    """Non-blocking adapter for the real arm/gripper controller.

    Implementations must include gripper/limit feedback in ``verified=True``;
    a ToF reading alone is not proof that a pick or placement succeeded.
    A successful pick must also confirm the item is stowed in ``vehicle_slot``
    and the arm is safe for travel before returning success. A successful stack
    must confirm placement on the same-colour base at the requested site.
    """

    def align_xy(
        self,
        dx_px: float,
        dy_px: float,
        *,
        target: str,
        ring_number: int | None = None,
    ) -> None:
        """Apply one bounded X/Y visual-servo correction without descending."""
        ...

    def start_pick(
        self,
        *,
        color: MaterialColor,
        ring_number: int | None,
        vehicle_slot: int,
        from_turntable: bool,
        observation: OpenMvObservation,
        distance_mm: int,
    ) -> None: ...

    def start_place(
        self,
        *,
        color: MaterialColor,
        ring_number: int,
        vehicle_slot: int,
        stack_level: int,
        observation: OpenMvObservation,
        distance_mm: int,
    ) -> None: ...

    def poll(self, timestamp_s: float) -> ActionResult: ...

    def safe_retract(self) -> None: ...

    def abort(self) -> None: ...
