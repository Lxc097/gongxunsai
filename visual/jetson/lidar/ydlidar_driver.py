from __future__ import annotations

from typing import Any


class YdlidarX3Pro:
    """Thin wrapper around the official ``ydlidar`` Python binding."""

    def __init__(self, port: str, *, baudrate: int = 115200) -> None:
        try:
            import ydlidar
        except ImportError as exc:
            raise RuntimeError("install the official YDLIDAR SDK Python binding") from exc
        self.ydlidar = ydlidar
        self.laser = ydlidar.CYdLidar()
        self.laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
        self.laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, baudrate)
        self.laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
        self.laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
        self.laser.setlidaropt(ydlidar.LidarPropSingleChannel, True)
        self.scan = ydlidar.LaserScan()

    def start(self) -> None:
        if not self.laser.initialize() or not self.laser.turnOn():
            raise RuntimeError("YDLIDAR X3 Pro initialization failed")

    def read_scan(self) -> tuple[list[float], list[float]]:
        if not self.laser.doProcessSimple(self.scan):
            return [], []
        angles = [float(point.angle) * 180.0 / 3.141592653589793 for point in self.scan.points]
        ranges = [float(point.range) * 1000.0 for point in self.scan.points]
        return angles, ranges

    def close(self) -> None:
        self.laser.turnOff()
        self.laser.disconnecting()
