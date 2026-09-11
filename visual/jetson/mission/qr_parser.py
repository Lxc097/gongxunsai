from __future__ import annotations

import re

from .models import MaterialColor, MissionCode, TaskBatch


class MissionCodeError(ValueError):
    pass


def _groups(payload: str) -> tuple[str, str, str, str]:
    text = payload.strip()
    separated = re.findall(r"\d+", text)
    if len(separated) == 1 and len(separated[0]) == 12:
        compact = separated[0]
        separated = [compact[index:index + 3] for index in range(0, 12, 3)]
    if len(separated) != 4 or any(len(group) != 3 for group in separated):
        raise MissionCodeError("task code must contain four three-digit groups")
    # Reject letters and punctuation other than the separators used on task cards.
    remainder = re.sub(r"[0-9\s+,:;/|._-]", "", text)
    if remainder:
        raise MissionCodeError("task code contains unsupported characters")
    return tuple(separated)  # type: ignore[return-value]


def parse_mission_code(payload: str) -> MissionCode:
    """Parse ``colours+sites+colours+rough_sites`` (e.g. 156+123+516+231).

    Colours use 1..6, but each batch has the same three distinct colours.
    Placement sites are 1..3; second-batch temporary sites are derived by colour
    from the first batch, not from the fourth QR group.
    """

    groups = _groups(payload)
    batches: list[TaskBatch] = []
    for batch_index in range(2):
        pickup_group = groups[batch_index * 2]
        ring_group = groups[batch_index * 2 + 1]
        try:
            colors = tuple(MaterialColor(int(value)) for value in pickup_group)
        except ValueError as exc:
            raise MissionCodeError("pickup colour digits must be in 1..6") from exc
        rings = tuple(int(value) for value in ring_group)
        if len(set(colors)) != 3:
            raise MissionCodeError("each batch must contain three distinct colours")
        if set(rings) != {1, 2, 3}:
            raise MissionCodeError("target sites must be a permutation of 1, 2, 3")
        batches.append(TaskBatch(colors, rings))  # type: ignore[arg-type]
    if set(batches[0].pickup_order) != set(batches[1].pickup_order):
        raise MissionCodeError("both batches must contain the same three colours")
    return MissionCode(payload, groups, tuple(batches))  # type: ignore[arg-type]
