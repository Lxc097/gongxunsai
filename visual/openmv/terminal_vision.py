"""OpenMV terminal camera program: material colour/centre and ring centre.

Copy this file and ``concentric_circle.py`` to the OpenMV filesystem.  LAB
thresholds and the working ROI must be calibrated under the final illumination.
The output is one UTF-8 JSON line per stable observation for the Jetson USB-UART
receiver; OpenMV is not used for global vehicle localisation.
"""

import sensor
import time
import ujson
from pyb import UART

from concentric_circle import ConcentricCircleFinder, ConcentricCircleTracker


# Example LAB starts only. Tune with OpenMV IDE Threshold Editor on the vehicle.
COLOR_THRESHOLDS = {
    "red": (20, 75, 20, 80, 5, 65),
    "yellow": (45, 95, -15, 35, 35, 85),
    "blue": (10, 70, -20, 30, -70, -15),
    "green": (15, 75, -70, -10, -10, 60),
    "black": (0, 30, -20, 20, -20, 20),
    "cyan": (30, 85, -55, -5, -35, 10),
}
MIN_BLOB_PIXELS = 120
MIN_BLOB_AREA = 180


def best_material(image, center_x, center_y):
    best = None
    for color, threshold in COLOR_THRESHOLDS.items():
        blobs = image.find_blobs([threshold], pixels_threshold=MIN_BLOB_PIXELS,
                                 area_threshold=MIN_BLOB_AREA, merge=True)
        for blob in blobs:
            score = blob.pixels()
            if best is None or score > best[0]:
                best = (score, color, blob.cx() - center_x, blob.cy() - center_y)
    return best


def main():
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.skip_frames(time=2000)
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)
    uart = UART(3, 115200, timeout_char=20)
    finder = ConcentricCircleFinder(min_concentric_circles=3)
    tracker = ConcentricCircleTracker(stable_frames=4)
    clock = time.clock()
    last_material = None
    material_stable_count = 0

    while True:
        clock.tick()
        image = sensor.snapshot()
        cx, cy = image.width() / 2, image.height() / 2
        material = best_material(image, cx, cy)
        material_key = None
        if material:
            # Quantisation prevents a one-pixel centroid wobble from resetting
            # the consecutive-frame gate.
            material_key = (material[1], int(material[2] / 4), int(material[3] / 4))
        if material_key is not None and material_key == last_material:
            material_stable_count += 1
        else:
            last_material = material_key
            material_stable_count = 1 if material_key is not None else 0
        material_stable = material_stable_count >= 4
        ring = finder.find_in_image(image, expected_center=(cx, cy))
        ring_state = tracker.update(ring)
        ring_target = ring_state["target"]
        item = {
            "type": "terminal",
            "material_color": material[1] if material else None,
            "material_dx_px": material[2] if material else None,
            "material_dy_px": material[3] if material else None,
            "ring_dx_px": ring_target["x"] - cx if ring_target else None,
            "ring_dy_px": ring_target["y"] - cy if ring_target else None,
            "material_stable": material_stable,
            "ring_stable": bool(ring_state["stable"]),
            "stable": bool(material_stable or ring_state["stable"]),
        }
        uart.write(ujson.dumps(item) + "\n")


main()
