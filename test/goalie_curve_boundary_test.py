#!/usr/bin/env python3

import math

LEFT_X = -390.0
RIGHT_X = 390.0
BOTTOM_Y = -1075.0
ARC_CENTER_Y = -975.0
TOP_Y = -835.0
LEFT_ARC_CENTER_X = -250.0
RIGHT_ARC_CENTER_X = 250.0
ARC_RADIUS = 140.0
MIN_OFFSET = 60.0
MAX_OFFSET = 120.0


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def candidate(px, py, cx, cy, nx, ny):
    return {
        "closest": (cx, cy),
        "normal": (nx, ny),
        "distance_sq": (px - cx) ** 2 + (py - cy) ** 2,
    }


def vertical(px, py, x, min_y, max_y, nx, ny):
    return candidate(px, py, x, clamp(py, min_y, max_y), nx, ny)


def horizontal(px, py, min_x, max_x, y, nx, ny):
    return candidate(px, py, clamp(px, min_x, max_x), y, nx, ny)


def arc(px, py, center_x, center_y, min_theta, max_theta):
    dx = px - center_x
    dy = py - center_y
    theta = (min_theta + max_theta) * 0.5 if abs(dx) < 1e-6 and abs(dy) < 1e-6 else math.atan2(dy, dx)
    while theta < 0:
        theta += 2.0 * math.pi
    while theta >= 2.0 * math.pi:
        theta -= 2.0 * math.pi
    theta = clamp(theta, min_theta, max_theta)
    nx = math.cos(theta)
    ny = math.sin(theta)
    return candidate(px, py, center_x + ARC_RADIUS * nx, center_y + ARC_RADIUS * ny, nx, ny)


def evaluate(px, py):
    candidates = [
        vertical(px, py, LEFT_X, BOTTOM_Y, ARC_CENTER_Y, -1.0, 0.0),
        arc(px, py, LEFT_ARC_CENTER_X, ARC_CENTER_Y, math.pi * 0.5, math.pi),
        horizontal(px, py, LEFT_ARC_CENTER_X, RIGHT_ARC_CENTER_X, TOP_Y, 0.0, 1.0),
        arc(px, py, RIGHT_ARC_CENTER_X, ARC_CENTER_Y, 0.0, math.pi * 0.5),
        vertical(px, py, RIGHT_X, BOTTOM_Y, ARC_CENTER_Y, 1.0, 0.0),
    ]
    best = min(candidates, key=lambda c: c["distance_sq"])
    cx, cy = best["closest"]
    nx, ny = best["normal"]
    signed_distance = (px - cx) * nx + (py - cy) * ny
    return signed_distance, best


def assert_close(actual, expected, label):
    if abs(actual - expected) > 1e-6:
        raise AssertionError(f"{label}: expected {expected}, got {actual}")


def main():
    cases = [
        ((-250.0, -775.0), 60.0),
        ((-250.0, -715.0), 120.0),
        ((-250.0, -745.0), 90.0),
        ((-250.0, -800.0), 35.0),
        ((-250.0, -830.0), 5.0),
        ((-450.0, -975.0), 60.0),
        ((450.0, -975.0), 60.0),
    ]

    for (px, py), expected in cases:
        signed_distance, _ = evaluate(px, py)
        assert_close(signed_distance, expected, f"pose ({px}, {py})")

    inside_distance, _ = evaluate(-250.0, -745.0)
    assert MIN_OFFSET <= inside_distance <= MAX_OFFSET

    print("goalie_curve_boundary_test: PASS")


if __name__ == "__main__":
    main()
