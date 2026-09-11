from __future__ import annotations

from dataclasses import asdict, dataclass, field

from .models import MissionStep


@dataclass(frozen=True)
class MissionEvent:
    timestamp_s: float
    name: str
    detail: str = ""


@dataclass
class MissionStatistics:
    started_at_s: float | None = None
    finished_at_s: float | None = None
    successful_picks: int = 0
    successful_placements: int = 0
    retries: int = 0
    failures: int = 0
    events: list[MissionEvent] = field(default_factory=list)

    def event(self, timestamp_s: float, name: str, detail: str = "") -> None:
        self.events.append(MissionEvent(timestamp_s, name, detail))

    def record_success(self, timestamp_s: float, step: MissionStep) -> None:
        if step.kind.name.startswith("PICK"):
            self.successful_picks += 1
        elif step.kind.name.startswith("PLACE"):
            self.successful_placements += 1
        self.event(timestamp_s, "step_succeeded", step.kind.name)

    def to_dict(self) -> dict[str, object]:
        result = asdict(self)
        if self.started_at_s is not None and self.finished_at_s is not None:
            result["elapsed_s"] = self.finished_at_s - self.started_at_s
        return result

