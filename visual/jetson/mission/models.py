from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, IntEnum, auto


class MaterialColor(IntEnum):
    RED = 1
    YELLOW = 2
    BLUE = 3
    GREEN = 4
    BLACK = 5
    CYAN = 6

    @classmethod
    def from_name(cls, name: str) -> "MaterialColor":
        aliases = {
            "red": cls.RED,
            "yellow": cls.YELLOW,
            "blue": cls.BLUE,
            "green": cls.GREEN,
            "black": cls.BLACK,
            "cyan": cls.CYAN,
        }
        try:
            return aliases[name.strip().lower()]
        except KeyError as exc:
            raise ValueError(f"unknown material color: {name}") from exc


@dataclass(frozen=True)
class TaskBatch:
    """Three distinct colours and rough-processing sites (a permutation of 1..3).

    ``target_rings`` is retained for API compatibility; it means placement sites,
    not the six concentric scoring circles within each site.
    """

    pickup_order: tuple[MaterialColor, MaterialColor, MaterialColor]
    target_rings: tuple[int, int, int]


@dataclass(frozen=True)
class MissionCode:
    raw_payload: str
    groups: tuple[str, str, str, str]
    batches: tuple[TaskBatch, TaskBatch]

    @property
    def canonical(self) -> str:
        return "+".join(self.groups)


class MissionStepKind(Enum):
    NAVIGATE = auto()
    PICK_RAW = auto()
    PLACE_ROUGH = auto()
    PICK_ROUGH = auto()
    PLACE_TEMPORARY = auto()


@dataclass(frozen=True)
class MissionStep:
    kind: MissionStepKind
    batch_index: int | None = None
    station: str | None = None
    color: MaterialColor | None = None
    ring_number: int | None = None
    vehicle_slot: int | None = None
    stack_level: int = 0


class MissionState(Enum):
    BOOT_SELF_TEST = auto()
    WAIT_START = auto()
    NAVIGATE_QR = auto()
    SEARCH_QR = auto()
    READ_AND_VALIDATE_QR = auto()
    BUILD_OBSTACLE_MAP = auto()
    NAVIGATE = auto()
    FINE_ALIGN_MATERIAL = auto()
    PICK_MATERIAL = auto()
    VERIFY_PICK = auto()
    FINE_ALIGN_RING = auto()
    PLACE_OR_STACK = auto()
    VERIFY_PLACE = auto()
    RETURN_HOME = auto()
    MISSION_COMPLETE = auto()
    SAFE_STOP = auto()


class ActionState(Enum):
    IDLE = auto()
    RUNNING = auto()
    SUCCEEDED = auto()
    FAILED = auto()


@dataclass(frozen=True)
class ActionResult:
    state: ActionState
    verified: bool = False
    error: str | None = None
