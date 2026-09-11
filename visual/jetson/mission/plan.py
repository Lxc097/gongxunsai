from __future__ import annotations

from .models import MissionCode, MissionStep, MissionStepKind


def build_preliminary_steps(code: MissionCode, *, home_station: str = "start_1") -> list[MissionStep]:
    """Expand a task code into the complete preliminary-round visit/action order."""

    steps: list[MissionStep] = []
    first_batch = code.batches[0]
    temporary_sites = dict(zip(first_batch.pickup_order, first_batch.target_rings))
    for batch_index, batch in enumerate(code.batches):
        steps.append(MissionStep(MissionStepKind.NAVIGATE, batch_index, station="raw_material"))
        for slot, (color, ring) in enumerate(zip(batch.pickup_order, batch.target_rings), start=1):
            steps.append(MissionStep(
                MissionStepKind.PICK_RAW, batch_index, color=color,
                ring_number=ring, vehicle_slot=slot,
            ))

        steps.append(MissionStep(MissionStepKind.NAVIGATE, batch_index, station="rough_processing"))
        for slot, (color, ring) in enumerate(zip(batch.pickup_order, batch.target_rings), start=1):
            steps.append(MissionStep(
                MissionStepKind.PLACE_ROUGH, batch_index, color=color,
                ring_number=ring, vehicle_slot=slot,
            ))
        for slot, (color, ring) in enumerate(zip(batch.pickup_order, batch.target_rings), start=1):
            steps.append(MissionStep(
                MissionStepKind.PICK_ROUGH, batch_index, color=color,
                ring_number=ring, vehicle_slot=slot,
            ))

        steps.append(MissionStep(MissionStepKind.NAVIGATE, batch_index, station="temporary_storage"))
        for slot, color in enumerate(batch.pickup_order, start=1):
            steps.append(MissionStep(
                MissionStepKind.PLACE_TEMPORARY, batch_index, color=color,
                ring_number=temporary_sites[color], vehicle_slot=slot,
                stack_level=batch_index,
            ))

    steps.append(MissionStep(MissionStepKind.NAVIGATE, station=home_station))
    return steps
