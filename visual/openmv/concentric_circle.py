"""物料放置区最小同心圆查找。

核心代码只使用 OpenMV MicroPython 具备的基础语法和 ``math`` 模块。
桌面测试既可传入 ``(x, y, r, magnitude)``，也可直接传入 OpenMV
``img.find_circles()`` 返回的圆对象。
"""

import math


# 赛规表 3：各环外径相对物料最大直径 phi 的增量，单位 mm。
# phi1=phi+3，之后的直径增量依次为 5、7、10、10、10 mm。
RULE_RING_DIAMETER_OFFSETS_MM = (3.0, 8.0, 15.0, 25.0, 35.0, 45.0)


def competition_ring_radii_px(material_diameter_mm, pixels_per_mm):
    """根据赛规和标定比例计算 1～6 环的理论外半径（像素）。"""

    if material_diameter_mm <= 0:
        raise ValueError("material_diameter_mm must be positive")
    if pixels_per_mm <= 0:
        raise ValueError("pixels_per_mm must be positive")
    return tuple(
        (material_diameter_mm + offset) * pixels_per_mm / 2.0
        for offset in RULE_RING_DIAMETER_OFFSETS_MM
    )


def _median(values):
    ordered = sorted(values)
    count = len(ordered)
    middle = count // 2
    if count % 2:
        return float(ordered[middle])
    return (ordered[middle - 1] + ordered[middle]) / 2.0


def _object_value(value, name, default=None):
    if isinstance(value, dict):
        return value.get(name, default)
    attribute = getattr(value, name, default)
    return attribute() if callable(attribute) else attribute


def _normalise_circle(circle):
    if isinstance(circle, (tuple, list)):
        if len(circle) < 3:
            raise ValueError("circle tuple must contain x, y and radius")
        magnitude = circle[3] if len(circle) > 3 else 1.0
        values = circle[0], circle[1], circle[2], magnitude
    else:
        radius = _object_value(circle, "r", None)
        if radius is None:
            radius = _object_value(circle, "radius", None)
        values = (
            _object_value(circle, "x", None),
            _object_value(circle, "y", None),
            radius,
            _object_value(circle, "magnitude", 1.0),
        )

    if values[0] is None or values[1] is None or values[2] is None:
        raise ValueError("circle must provide x, y and radius")
    magnitude = 1.0 if values[3] is None else float(values[3])
    return {
        "x": float(values[0]),
        "y": float(values[1]),
        "r": float(values[2]),
        "magnitude": magnitude,
    }


def _distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


class ConcentricCircleFinder:
    """从 Hough 圆候选中找出同心圆组及其中的最小圆。

    ``center_tolerance_px`` 用于吸收不同半径圆的圆心抖动；
    ``radius_merge_tolerance_px`` 用于合并同一条 1.5 mm 圆环的双边缘和
    Hough 重复结果。只有具有足够多不同半径的圆组才会被接受。
    """

    def __init__(
        self,
        center_tolerance_px=6.0,
        radius_merge_tolerance_px=2.0,
        min_concentric_circles=3,
        min_radius_px=2.0,
        max_radius_px=32767.0,
        min_magnitude=0.0,
        rule_radius_tolerance_px=3.0,
    ):
        if center_tolerance_px <= 0:
            raise ValueError("center_tolerance_px must be positive")
        if radius_merge_tolerance_px < 0:
            raise ValueError("radius_merge_tolerance_px cannot be negative")
        if min_concentric_circles < 2:
            raise ValueError("min_concentric_circles must be at least 2")
        if min_radius_px <= 0 or max_radius_px < min_radius_px:
            raise ValueError("invalid radius range")
        self.center_tolerance_px = float(center_tolerance_px)
        self.radius_merge_tolerance_px = float(radius_merge_tolerance_px)
        self.min_concentric_circles = int(min_concentric_circles)
        self.min_radius_px = float(min_radius_px)
        self.max_radius_px = float(max_radius_px)
        self.min_magnitude = float(min_magnitude)
        self.rule_radius_tolerance_px = float(rule_radius_tolerance_px)

    def _filter(self, circles):
        accepted = []
        for circle in circles:
            item = _normalise_circle(circle)
            if not self.min_radius_px <= item["r"] <= self.max_radius_px:
                continue
            if item["magnitude"] < self.min_magnitude:
                continue
            accepted.append(item)
        return accepted

    def _center_clusters(self, circles):
        # 强圆先成为种子，使偶发弱干扰不容易把两个目标串成一个圆组。
        ordered = sorted(circles, key=lambda item: item["magnitude"], reverse=True)
        clusters = []
        for circle in ordered:
            best = None
            best_distance = None
            for cluster in clusters:
                cx = _median([item["x"] for item in cluster])
                cy = _median([item["y"] for item in cluster])
                distance = _distance(circle["x"], circle["y"], cx, cy)
                if distance <= self.center_tolerance_px:
                    if best_distance is None or distance < best_distance:
                        best = cluster
                        best_distance = distance
            if best is None:
                clusters.append([circle])
            else:
                best.append(circle)
        return clusters

    def _merge_duplicate_radii(self, cluster):
        groups = []
        for circle in sorted(cluster, key=lambda item: item["r"]):
            if not groups:
                groups.append([circle])
                continue
            radius = _median([item["r"] for item in groups[-1]])
            if abs(circle["r"] - radius) <= self.radius_merge_tolerance_px:
                groups[-1].append(circle)
            else:
                groups.append([circle])
        return groups

    def _rule_match_count(self, radii, expected_radii_px):
        if not expected_radii_px:
            return 0
        matched = 0
        used = set()
        for expected in expected_radii_px:
            best_index = None
            best_error = None
            for index, radius in enumerate(radii):
                if index in used:
                    continue
                error = abs(radius - expected)
                if error <= self.rule_radius_tolerance_px:
                    if best_error is None or error < best_error:
                        best_index = index
                        best_error = error
            if best_index is not None:
                used.add(best_index)
                matched += 1
        return matched

    def _describe_cluster(self, cluster, expected_radii_px):
        radius_groups = self._merge_duplicate_radii(cluster)
        if len(radius_groups) < self.min_concentric_circles:
            return None

        # 每个半径只取最强候选参与共同圆心计算，避免重复边缘产生偏置。
        representatives = [
            max(group, key=lambda item: item["magnitude"])
            for group in radius_groups
        ]
        center_x = _median([item["x"] for item in representatives])
        center_y = _median([item["y"] for item in representatives])
        radii = [_median([item["r"] for item in group]) for group in radius_groups]
        spread = max(
            _distance(item["x"], item["y"], center_x, center_y)
            for item in representatives
        )
        smallest_group = radius_groups[0]
        smallest = max(smallest_group, key=lambda item: item["magnitude"])
        magnitude_sum = sum(item["magnitude"] for item in representatives)
        rule_matches = self._rule_match_count(radii, expected_radii_px)
        return {
            "x": center_x,
            "y": center_y,
            "r": radii[0],
            "detected_x": smallest["x"],
            "detected_y": smallest["y"],
            "concentric_count": len(radius_groups),
            "radii": tuple(radii),
            "center_spread_px": spread,
            "rule_matches": rule_matches,
            "magnitude_sum": magnitude_sum,
        }

    def find_all(self, circles, expected_radii_px=None):
        """返回所有有效同心圆目标，按可信程度从高到低排序。"""

        targets = []
        for cluster in self._center_clusters(self._filter(circles)):
            target = self._describe_cluster(cluster, expected_radii_px)
            if target is not None:
                targets.append(target)
        targets.sort(
            key=lambda item: (
                item["rule_matches"],
                item["concentric_count"],
                -item["center_spread_px"],
                item["magnitude_sum"],
            ),
            reverse=True,
        )
        return targets

    def find(
        self,
        circles,
        expected_center=None,
        expected_radii_px=None,
        max_expected_center_distance_px=None,
    ):
        """返回最佳目标；没有可靠同心圆组时返回 ``None``。

        已知目标大致像素位置时应传 ``expected_center=(x, y)``，这样同一
        画面出现多个放置位时不会选错环号。
        """

        targets = self.find_all(circles, expected_radii_px)
        if not targets:
            return None
        if expected_center is None:
            return targets[0]
        expected_x, expected_y = expected_center
        selected = min(
            targets,
            key=lambda item: _distance(
                item["x"], item["y"], expected_x, expected_y
            ),
        )
        if max_expected_center_distance_px is not None:
            if max_expected_center_distance_px <= 0:
                raise ValueError("max_expected_center_distance_px must be positive")
            distance = _distance(
                selected["x"], selected["y"], expected_x, expected_y
            )
            if distance > max_expected_center_distance_px:
                return None
        return selected

    def find_in_image(
        self,
        image,
        roi=None,
        expected_center=None,
        expected_radii_px=None,
        max_expected_center_distance_px=None,
        threshold=2500,
        x_margin=10,
        y_margin=10,
        r_margin=10,
        x_stride=2,
        y_stride=1,
        r_step=1,
    ):
        """调用 OpenMV ``image.find_circles`` 后选出最小同心圆。"""

        arguments = {
            "x_stride": x_stride,
            "y_stride": y_stride,
            "threshold": threshold,
            "x_margin": x_margin,
            "y_margin": y_margin,
            "r_margin": r_margin,
            "r_min": int(self.min_radius_px),
            "r_max": int(self.max_radius_px),
            "r_step": r_step,
        }
        if roi is not None:
            arguments["roi"] = roi
        circles = image.find_circles(**arguments)
        return self.find(
            circles,
            expected_center,
            expected_radii_px,
            max_expected_center_distance_px,
        )


class ConcentricCircleTracker:
    """多帧稳定判定；只有连续稳定后才允许机械臂释放物料。"""

    def __init__(self, stable_frames=4, max_center_jump_px=3.0, max_radius_jump_px=3.0):
        if stable_frames < 1:
            raise ValueError("stable_frames must be positive")
        self.stable_frames = int(stable_frames)
        self.max_center_jump_px = float(max_center_jump_px)
        self.max_radius_jump_px = float(max_radius_jump_px)
        self.reset()

    def reset(self):
        self._history = []

    def update(self, target):
        """更新一帧结果，返回含 ``stable`` 与平滑坐标的状态字典。"""

        if target is None:
            self.reset()
            return {"stable": False, "stable_frames": 0, "target": None}

        if self._history:
            previous = self._history[-1]
            center_jump = _distance(
                target["x"], target["y"], previous["x"], previous["y"]
            )
            radius_jump = abs(target["r"] - previous["r"])
            if (
                center_jump > self.max_center_jump_px
                or radius_jump > self.max_radius_jump_px
            ):
                self.reset()

        self._history.append(target)
        if len(self._history) > self.stable_frames:
            self._history.pop(0)

        smoothed = dict(target)
        smoothed["x"] = _median([item["x"] for item in self._history])
        smoothed["y"] = _median([item["y"] for item in self._history])
        smoothed["r"] = _median([item["r"] for item in self._history])
        count = len(self._history)
        return {
            "stable": count >= self.stable_frames,
            "stable_frames": count,
            "target": smoothed,
        }
