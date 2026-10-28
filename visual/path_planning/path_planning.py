#!/usr/bin/env python3
"""
河北省大学生工程实践与创新能力大赛“智能搬运”赛项路径规划器。

算法流程：
1. 按 2400 mm × 2400 mm 场地建立栅格地图；
2. 将四块非灰色区域和现场随机圆柱障碍物按机器人安全半径膨胀；
3. 使用带转向代价的八邻域 A* 搜索无碰撞路径；
4. 用视线检测压缩冗余航点；
5. 将航点转换为 STM32 可执行的转向/直行动作。

本文件只依赖 Python 标准库，可直接在上位机上运行。
"""

from __future__ import annotations

import argparse
import heapq
import json
import math
from dataclasses import asdict, dataclass
from enum import IntEnum
import re
from typing import Iterable, Sequence


Point = tuple[float, float]
GridPoint = tuple[int, int]

FIELD_WIDTH_MM = 2400.0
FIELD_HEIGHT_MM = 2400.0

# 图 1/图 6 中四块 450 mm × 450 mm 非灰色区域。
# 中央十字灰色车道宽 400 mm，场地中心为 (1200, 1200)。
NON_ROAD_ZONES = (
    (550.0, 550.0, 450.0, 450.0),
    (1400.0, 550.0, 450.0, 450.0),
    (550.0, 1400.0, 450.0, 450.0),
    (1400.0, 1400.0, 450.0, 450.0),
)

# 各功能区的车道侧作业点。机器人停在灰色车道中，不驶入功能区。
STATIONS: dict[str, Point] = {
    "start_1": (2250.0, 2250.0),
    "start_2": (2250.0, 150.0),
    "qr": (2150.0, 1200.0),
    "raw_material": (1200.0, 2150.0),
    "rough_processing": (1200.0, 250.0),
    "temporary_storage": (250.0, 1200.0),
}

COLORS = {
    "1": "红色",
    "2": "黄色",
    "3": "蓝色",
    "4": "绿色",
    "5": "黑色",
    "6": "浅蓝色",
}


# 动作整数编码定义在本文件，不能依赖枚举声明顺序。当前 driver 工程没有
# ActionType 头文件，F407 拒绝 ACTION_BATCH，见 PROTOCOL.md。
PATH_ACTION_PROTOCOL_VERSION = 1


class ActionType(IntEnum):
    """Python/STM32 共用的底盘动作编码。"""

    MOVE_FORWARD = 0
    MOVE_BACKWARD = 1
    TURN_LEFT = 2
    TURN_RIGHT = 3
    SEGMENT_END = 4
    CAR_STOP = 5


class PathPlanningError(RuntimeError):
    """路径规划失败。"""


@dataclass(frozen=True)
class CircleObstacle:
    """黑色圆柱障碍物的二维圆形占地，单位为 mm。"""

    x: float
    y: float
    radius: float = 25.0


@dataclass(frozen=True)
class Action:
    """底盘动作。value 对直行为 mm，对转向为度。"""

    action: str
    value: float

    def __post_init__(self) -> None:
        if self.action not in ActionType.__members__:
            raise ValueError(f"不支持的路径动作：{self.action}")

    @property
    def code(self) -> int:
        """STM32 ``ActionType`` 对应的稳定整数编码。"""

        return int(ActionType[self.action])

    def to_dict(self) -> dict[str, float | int | str]:
        return {"action": self.action, "code": self.code, "value": self.value}


@dataclass
class PlanResult:
    waypoints: list[Point]
    length_mm: float
    expanded_nodes: int
    actions: list[Action]

    def to_dict(self) -> dict:
        return {
            "waypoints": [[round(x, 2), round(y, 2)] for x, y in self.waypoints],
            "length_mm": round(self.length_mm, 2),
            "expanded_nodes": self.expanded_nodes,
            "actions": [action.to_dict() for action in self.actions],
        }


@dataclass(frozen=True)
class PlannerConfig:
    """路径规划参数，长度单位均为 mm。"""

    resolution: float = 25.0
    robot_radius: float = 130.0
    safety_margin: float = 20.0
    turn_penalty: float = 35.0
    max_expanded_nodes: int = 500_000

    def __post_init__(self) -> None:
        if self.resolution <= 0:
            raise ValueError("resolution 必须大于 0")
        if self.robot_radius < 0 or self.safety_margin < 0:
            raise ValueError("robot_radius 和 safety_margin 不能为负数")
        if self.clearance >= min(FIELD_WIDTH_MM, FIELD_HEIGHT_MM) / 2:
            raise ValueError("机器人安全半径过大，场地内不存在可行区域")

    @property
    def clearance(self) -> float:
        """规划使用的总安全半径。"""

        return self.robot_radius + self.safety_margin


@dataclass
class MissionResult:
    """完整初赛搬运流程的规划结果。"""

    visits: list[str]
    legs: list[PlanResult]

    @property
    def length_mm(self) -> float:
        return sum(leg.length_mm for leg in self.legs)

    @property
    def actions(self) -> list[Action]:
        return [action for leg in self.legs for action in leg.actions]

    def to_dict(self) -> dict:
        return {
            "visits": self.visits,
            "total_length_mm": round(self.length_mm, 2),
            "legs": [leg.to_dict() for leg in self.legs],
            "actions": [action.to_dict() for action in self.actions],
        }


# 八邻域方向按逆时针排列，索引同时代表 45° 的整数倍。
MOVES: tuple[tuple[int, int], ...] = (
    (1, 0),
    (1, 1),
    (0, 1),
    (-1, 1),
    (-1, 0),
    (-1, -1),
    (0, -1),
    (1, -1),
)

# 初赛必须完成的区域访问顺序。每一区域内部的抓取、放置动作由机械臂执行。
MISSION_STATIONS: tuple[str, ...] = (
    "qr",
    "raw_material",
    "rough_processing",
    "temporary_storage",
    "raw_material",
    "rough_processing",
    "temporary_storage",
)


def normalize_angle(angle_deg: float) -> float:
    """把角度归一化到 [-180, 180)。"""

    return (angle_deg + 180.0) % 360.0 - 180.0


def path_length(path: Sequence[Point]) -> float:
    return sum(
        math.hypot(x2 - x1, y2 - y1)
        for (x1, y1), (x2, y2) in zip(path, path[1:])
    )


def _direction_index(heading_deg: float) -> int:
    return int(round((heading_deg % 360.0) / 45.0)) % 8


class CompetitionArena:
    """连续坐标场地模型。

    矩形非车道区和圆柱障碍物均按 ``clearance`` 膨胀；场地边界则向内
    收缩相同距离。因此，只要机器人实际外接圆不大于配置值，规划出的
    机器人中心轨迹就不会越过车道或碰撞障碍物。
    """

    def __init__(
        self,
        config: PlannerConfig | None = None,
        obstacles: Iterable[CircleObstacle] = (),
    ) -> None:
        self.config = config or PlannerConfig()
        self.obstacles = tuple(obstacles)

    @property
    def clearance(self) -> float:
        return self.config.clearance

    def is_free_point(self, point: Point) -> bool:
        x, y = point
        c = self.clearance

        if not (c <= x <= FIELD_WIDTH_MM - c and c <= y <= FIELD_HEIGHT_MM - c):
            return False

        for rx, ry, width, height in NON_ROAD_ZONES:
            if rx - c <= x <= rx + width + c and ry - c <= y <= ry + height + c:
                return False

        for obstacle in self.obstacles:
            radius = obstacle.radius + c
            if (x - obstacle.x) ** 2 + (y - obstacle.y) ** 2 <= radius**2:
                return False

        return True

    def is_free_segment(
        self,
        start: Point,
        goal: Point,
        sample_step: float | None = None,
    ) -> bool:
        """按不大于半个栅格的间距检查连续线段，避免航点间穿障碍。"""

        distance = math.dist(start, goal)
        step = sample_step or min(10.0, self.config.resolution / 2.0)
        samples = max(1, math.ceil(distance / step))
        return all(
            self.is_free_point(
                (
                    start[0] + (goal[0] - start[0]) * i / samples,
                    start[1] + (goal[1] - start[1]) * i / samples,
                )
            )
            for i in range(samples + 1)
        )

    def validate_path(self, path: Sequence[Point]) -> bool:
        return bool(path) and all(
            self.is_free_segment(start, goal)
            for start, goal in zip(path, path[1:])
        )


class AStarPlanner:
    """带转向代价的八邻域 A* 路径规划器。"""

    def __init__(self, arena: CompetitionArena) -> None:
        self.arena = arena
        self.config = arena.config
        self.nx = int(round(FIELD_WIDTH_MM / self.config.resolution)) + 1
        self.ny = int(round(FIELD_HEIGHT_MM / self.config.resolution)) + 1
        self._free_cache: dict[GridPoint, bool] = {}

    def _to_world(self, grid: GridPoint) -> Point:
        return (
            grid[0] * self.config.resolution,
            grid[1] * self.config.resolution,
        )

    def _in_grid(self, grid: GridPoint) -> bool:
        return 0 <= grid[0] < self.nx and 0 <= grid[1] < self.ny

    def _is_free_grid(self, grid: GridPoint) -> bool:
        if grid not in self._free_cache:
            self._free_cache[grid] = self._in_grid(grid) and self.arena.is_free_point(
                self._to_world(grid)
            )
        return self._free_cache[grid]

    def _nearest_visible_grid(self, point: Point) -> GridPoint:
        """把任意连续坐标映射到附近可见的自由栅格点。"""

        if not self.arena.is_free_point(point):
            raise PathPlanningError(
                f"点 {point} 不在安全车道内；请检查坐标、车体半径或障碍物"
            )

        center = (
            int(round(point[0] / self.config.resolution)),
            int(round(point[1] / self.config.resolution)),
        )
        limit = max(self.nx, self.ny)
        for radius in range(limit):
            candidates: list[GridPoint] = []
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if max(abs(dx), abs(dy)) == radius:
                        candidates.append((center[0] + dx, center[1] + dy))
            candidates.sort(key=lambda cell: math.dist(point, self._to_world(cell)))
            for cell in candidates:
                if self._is_free_grid(cell) and self.arena.is_free_segment(
                    point, self._to_world(cell)
                ):
                    return cell
        raise PathPlanningError(f"点 {point} 附近找不到自由栅格")

    def _reconstruct(
        self,
        came_from: dict[tuple[int, int, int], tuple[int, int, int]],
        state: tuple[int, int, int],
    ) -> list[Point]:
        states = [state]
        while state in came_from:
            state = came_from[state]
            states.append(state)
        states.reverse()
        return [self._to_world((x, y)) for x, y, _ in states]

    def _simplify(self, path: Sequence[Point]) -> list[Point]:
        """使用最远可见点法消除栅格锯齿和冗余航点。"""

        if len(path) <= 2:
            return list(path)
        result = [path[0]]
        anchor = 0
        while anchor < len(path) - 1:
            candidate = len(path) - 1
            while candidate > anchor + 1:
                if self.arena.is_free_segment(path[anchor], path[candidate]):
                    break
                candidate -= 1
            result.append(path[candidate])
            anchor = candidate
        return result

    def plan(
        self,
        start: Point,
        goal: Point,
        start_heading_deg: float | None = None,
        simplify: bool = True,
    ) -> PlanResult:
        start_cell = self._nearest_visible_grid(start)
        goal_cell = self._nearest_visible_grid(goal)

        if start_cell == goal_cell:
            waypoints = [start, goal] if start != goal else [start]
            actions, _ = waypoints_to_actions(
                waypoints, start_heading_deg or 0.0, stop_at_end=True
            )
            return PlanResult(waypoints, path_length(waypoints), 0, actions)

        initial_direction = (
            _direction_index(start_heading_deg) if start_heading_deg is not None else -1
        )
        initial = (start_cell[0], start_cell[1], initial_direction)
        queue: list[tuple[float, float, int, tuple[int, int, int]]] = []
        counter = 0
        heapq.heappush(
            queue,
            (math.dist(start, goal), 0.0, counter, initial),
        )
        came_from: dict[tuple[int, int, int], tuple[int, int, int]] = {}
        best_cost = {initial: 0.0}
        expanded = 0
        final_state: tuple[int, int, int] | None = None

        while queue:
            _, current_cost, _, current = heapq.heappop(queue)
            if current_cost > best_cost.get(current, math.inf) + 1e-9:
                continue
            x, y, previous_direction = current
            if (x, y) == goal_cell:
                final_state = current
                break

            expanded += 1
            if expanded > self.config.max_expanded_nodes:
                raise PathPlanningError("A* 超过最大扩展节点数，未找到路径")

            current_world = self._to_world((x, y))
            for direction, (dx, dy) in enumerate(MOVES):
                neighbor = (x + dx, y + dy)
                if not self._is_free_grid(neighbor):
                    continue
                neighbor_world = self._to_world(neighbor)
                if not self.arena.is_free_segment(current_world, neighbor_world):
                    continue

                move_cost = math.hypot(dx, dy) * self.config.resolution
                turn_steps = (
                    0
                    if previous_direction < 0
                    else min(
                        (direction - previous_direction) % 8,
                        (previous_direction - direction) % 8,
                    )
                )
                new_cost = (
                    current_cost
                    + move_cost
                    + turn_steps * self.config.turn_penalty
                )
                state = (neighbor[0], neighbor[1], direction)
                if new_cost + 1e-9 >= best_cost.get(state, math.inf):
                    continue

                best_cost[state] = new_cost
                came_from[state] = current
                heuristic = math.dist(neighbor_world, self._to_world(goal_cell))
                counter += 1
                heapq.heappush(
                    queue,
                    (new_cost + heuristic, new_cost, counter, state),
                )

        if final_state is None:
            raise PathPlanningError(f"从 {start} 到 {goal} 不存在安全路径")

        grid_path = self._reconstruct(came_from, final_state)
        # 保留传入的精确坐标及其映射栅格，确保非栅格输入点的接入线段也安全。
        continuous_path: list[Point] = [start]
        for point in grid_path:
            if math.dist(continuous_path[-1], point) > 1e-6:
                continuous_path.append(point)
        if math.dist(continuous_path[-1], goal) > 1e-6:
            continuous_path.append(goal)
        if simplify:
            continuous_path = self._simplify(continuous_path)
        if not self.arena.validate_path(continuous_path):
            raise PathPlanningError("内部错误：路径连续碰撞检测未通过")

        heading = start_heading_deg if start_heading_deg is not None else 0.0
        actions, _ = waypoints_to_actions(
            continuous_path, heading, stop_at_end=True
        )
        return PlanResult(
            waypoints=continuous_path,
            length_mm=path_length(continuous_path),
            expanded_nodes=expanded,
            actions=actions,
        )


def waypoints_to_actions(
    path: Sequence[Point],
    start_heading_deg: float = 90.0,
    stop_at_end: bool = True,
    end_action: ActionType = ActionType.CAR_STOP,
) -> tuple[list[Action], float]:
    """把连续航点转换为底盘动作，并返回最终航向。

    ``SEGMENT_END`` 用于停靠作业点后继续下一段；``CAR_STOP`` 只用于
    单段规划或完整任务的最终停车。
    """

    actions: list[Action] = []
    heading = start_heading_deg
    for start, goal in zip(path, path[1:]):
        dx, dy = goal[0] - start[0], goal[1] - start[1]
        distance = math.hypot(dx, dy)
        if distance < 1e-6:
            continue
        target_heading = math.degrees(math.atan2(dy, dx))
        turn = normalize_angle(target_heading - heading)
        if abs(turn) >= 0.5:
            actions.append(
                Action("TURN_LEFT" if turn > 0 else "TURN_RIGHT", round(abs(turn), 2))
            )
        actions.append(Action("MOVE_FORWARD", round(distance, 2)))
        heading = target_heading
    if stop_at_end:
        actions.append(Action(end_action.name, 0.0))
    return actions, heading


def path_to_actions(
    path: Sequence[Point],
    start_heading_deg: float = 90.0,
) -> list[Action]:
    """兼容简单调用的动作转换接口。"""

    return waypoints_to_actions(path, start_heading_deg, stop_at_end=True)[0]


def plan_preliminary_mission(
    planner: AStarPlanner,
    start: Point,
    start_name: str,
    stations: dict[str, Point] | None = None,
    start_heading_deg: float = 90.0,
    simplify: bool = True,
) -> MissionResult:
    """规划“扫码→第一批→第二批→返回启停区”的完整初赛路线。

    每个中间作业点以 ``SEGMENT_END`` 停车，最终才发出 ``CAR_STOP``。
    """

    points = dict(STATIONS)
    if stations:
        points.update(stations)

    visits = [start_name, *MISSION_STATIONS, start_name]
    coordinates = [start, *(points[name] for name in MISSION_STATIONS), start]
    legs: list[PlanResult] = []
    heading = start_heading_deg
    leg_pairs = list(zip(coordinates, coordinates[1:]))
    for index, (leg_start, leg_goal) in enumerate(leg_pairs):
        end_action = (
            ActionType.CAR_STOP
            if index == len(leg_pairs) - 1
            else ActionType.SEGMENT_END
        )
        leg = planner.plan(
            leg_start,
            leg_goal,
            start_heading_deg=heading,
            simplify=simplify,
        )
        actions, _ = waypoints_to_actions(
            leg.waypoints,
            heading,
            stop_at_end=True,
            end_action=end_action,
        )
        leg.actions = actions
        legs.append(leg)
        _, heading = waypoints_to_actions(
            leg.waypoints, heading, stop_at_end=False
        )
    return MissionResult(visits=visits, legs=legs)


def actions_to_c(actions: Sequence[Action], variable_name: str = "route") -> str:
    """生成可纳入 STM32 工程的 C 动作数组。

    生成文件只引用 ``map.h`` 的类型，避免重复定义 ``ActionType`` 或
    ``PathAction``；请将其放入使用相同头文件搜索路径的 STM32 工程。
    """

    if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", variable_name):
        raise ValueError("C 数组变量名必须是合法的标识符")

    lines = [
        "/* Generated by path_planning.py; do not edit manually. */",
        f"/* PATH_ACTION_PROTOCOL_VERSION: {PATH_ACTION_PROTOCOL_VERSION} */",
        '#include "map.h"',
        "",
        f"static const PathAction {variable_name}[] = {{",
    ]
    for action in actions:
        lines.append(
            f"    {{{action.action}, {action.value:.2f}f}},"
        )
    lines.append("};")
    lines.append(
        f"static const unsigned int {variable_name}_count = "
        f"sizeof({variable_name}) / sizeof({variable_name}[0]);"
    )
    return "\n".join(lines)


def _parse_point(value: str) -> Point:
    try:
        x, y = (float(item.strip()) for item in value.split(","))
    except (ValueError, TypeError) as exc:
        raise argparse.ArgumentTypeError("坐标格式应为 X,Y") from exc
    return x, y


def _parse_obstacle(value: str) -> CircleObstacle:
    try:
        values = [float(item.strip()) for item in value.split(",")]
    except ValueError as exc:
        raise argparse.ArgumentTypeError("障碍物格式应为 X,Y 或 X,Y,R") from exc
    if len(values) == 2:
        return CircleObstacle(values[0], values[1])
    if len(values) == 3 and values[2] > 0:
        return CircleObstacle(*values)
    raise argparse.ArgumentTypeError("障碍物格式应为 X,Y 或 X,Y,R，且 R>0")


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="智能搬运赛项：带转向代价和动态避障的 A* 路径规划"
    )
    parser.add_argument("--start", choices=("start_1", "start_2"), default="start_1")
    parser.add_argument("--start-point", type=_parse_point, help="覆盖启停点，格式 X,Y")
    parser.add_argument("--goal", choices=tuple(STATIONS), default="raw_material")
    parser.add_argument("--goal-point", type=_parse_point, help="覆盖目标点，格式 X,Y")
    parser.add_argument("--mission", action="store_true", help="规划完整初赛搬运流程")
    parser.add_argument(
        "--obstacle",
        action="append",
        type=_parse_obstacle,
        default=[],
        metavar="X,Y[,R]",
        help="添加视觉检测到的圆柱障碍物，可重复使用；默认半径 25mm",
    )
    parser.add_argument("--qr-y", type=float, help="二维码板中心的现场 y 坐标")
    parser.add_argument("--raw-x", type=float, help="原料转盘中心的现场 x 坐标")
    parser.add_argument("--resolution", type=float, default=25.0)
    parser.add_argument("--robot-radius", type=float, default=130.0)
    parser.add_argument("--safety-margin", type=float, default=20.0)
    parser.add_argument("--turn-penalty", type=float, default=35.0)
    parser.add_argument("--start-heading", type=float, default=90.0)
    parser.add_argument("--no-simplify", action="store_true")
    parser.add_argument("--pretty", action="store_true")
    parser.add_argument("--export-c", metavar="FILE", help="同时导出 STM32 C 动作数组")
    parser.add_argument("--serial-port", metavar="PORT", help="已禁用：当前固件不接受 ACTION_BATCH；使用 jetson.main")
    parser.add_argument("--baud-rate", type=int, default=115200, help="串口波特率，默认 115200")
    parser.add_argument("--ack-timeout", type=float, default=0.5, help="ACK 超时秒数，默认 0.5")
    parser.add_argument("--retries", type=int, default=3, help="超时或可重试 NACK 的重传次数，默认 3")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    if args.serial_port:
        raise PathPlanningError("当前 F407/F103 固件拒绝 ACTION_BATCH，禁止串口上传；"
                                "请使用 python -m jetson.main 的动态速度闭环。离线导出仍可用。")
    config = PlannerConfig(
        resolution=args.resolution,
        robot_radius=args.robot_radius,
        safety_margin=args.safety_margin,
        turn_penalty=args.turn_penalty,
    )
    arena = CompetitionArena(config, args.obstacle)
    planner = AStarPlanner(arena)

    start = args.start_point or STATIONS[args.start]
    stations = dict(STATIONS)
    if args.qr_y is not None:
        stations["qr"] = (stations["qr"][0], args.qr_y)
    if args.raw_x is not None:
        stations["raw_material"] = (args.raw_x, stations["raw_material"][1])

    if args.mission:
        result: PlanResult | MissionResult = plan_preliminary_mission(
            planner,
            start=start,
            start_name=args.start,
            stations=stations,
            start_heading_deg=args.start_heading,
            simplify=not args.no_simplify,
        )
        actions = result.actions
    else:
        goal = args.goal_point or stations[args.goal]
        result = planner.plan(
            start,
            goal,
            start_heading_deg=args.start_heading,
            simplify=not args.no_simplify,
        )
        actions = result.actions

    payload = {
        "config": asdict(config),
        "obstacles": [asdict(obstacle) for obstacle in args.obstacle],
        "result": result.to_dict(),
    }

    print(json.dumps(payload, ensure_ascii=False, indent=2 if args.pretty else None))

    if args.export_c:
        with open(args.export_c, "w", encoding="utf-8", newline="\n") as file:
            file.write(actions_to_c(actions))
            file.write("\n")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (PathPlanningError, ValueError) as error:
        raise SystemExit(f"路径规划失败：{error}") from error
