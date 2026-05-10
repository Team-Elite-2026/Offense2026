#!/usr/bin/env python3
"""
Curved goalie-line defense simulation that mirrors current runtime math.

What this version adds:
1) Ball is drawn as an actual FIELD position (not only angle text).
2) Ball angle is computed per robot sample from world geometry:
      ballAngle(robot frame) = bearing_world(robot->ball) - robot_heading_field
3) No gradient/colorbar; vector SIZE encodes magnitudes:
   - Blue: tangent component magnitude = 1.0
   - Purple: normal correction component magnitude = normal_gain
   - Red: resultant pre-normalization magnitude = sqrt(x^2 + y^2)
4) Green vector is desired heading, using the same runDefense logic.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def normalize360(angle: float) -> float:
    angle = math.fmod(angle, 360.0)
    if angle < 0:
        angle += 360.0
    return angle


def normalize180(angle: float) -> float:
    angle = normalize360(angle)
    if angle > 180.0:
        angle -= 360.0
    return angle


def wrap_angle(angle: float) -> float:
    # Mirrors Trig::wrapAngle in src/trig.cpp.
    if angle < -180.0:
        angle += 360.0
    if angle > 180.0:
        angle -= 360.0
    return angle


def angular_distance(a: float, b: float) -> float:
    return abs(normalize180(a - b))


def sin_deg(angle: float) -> float:
    return math.sin(math.radians(angle))


def cos_deg(angle: float) -> float:
    return math.cos(math.radians(angle))


def angle_to_vec(angle_deg: float) -> tuple[float, float]:
    # Same angle convention as codebase: atan2(x, y) => 0 at +Y, 90 at +X.
    return sin_deg(angle_deg), cos_deg(angle_deg)


def bearing_world_from_to(x0: float, y0: float, x1: float, y1: float) -> float:
    dx = x1 - x0
    dy = y1 - y0
    return normalize360(math.degrees(math.atan2(dx, dy)))


def project_tangent(line_normal_angle: float, movement_angle: float) -> float:
    t_plus = normalize360(line_normal_angle + 90.0)
    t_minus = normalize360(line_normal_angle - 90.0)
    if angular_distance(movement_angle, t_plus) <= angular_distance(movement_angle, t_minus):
        return t_plus
    return t_minus


def blend_components(
    tangent_angle: float,
    line_normal_angle: float,
    chord_norm: float,
    cross_line: bool,
) -> tuple[float, float, float, float]:
    chord = min(1.0, max(0.0, chord_norm))
    normal_gain = 0.0 if chord >= 0.92 else (0.65 * (1.0 - chord))

    correction_angle = normalize360(line_normal_angle + 180.0) if cross_line else normalize360(line_normal_angle)

    tx = sin_deg(tangent_angle)
    ty = cos_deg(tangent_angle)
    cx = normal_gain * sin_deg(correction_angle)
    cy = normal_gain * cos_deg(correction_angle)
    rx = tx + cx
    ry = ty + cy
    result_mag = math.sqrt(rx * rx + ry * ry)
    result_angle = normalize360(math.degrees(math.atan2(rx, ry)))
    return result_angle, normal_gain, correction_angle, result_mag


def defense_calc_robot_frame(
    ball_angle: float,
    home_goal_angle: float,
    heading_correction: float,
    line_normal_angle: float,
    chord_norm: float,
    cross_line: bool,
) -> float:
    # Matches Defense::defenseCalc math path for non-blocked movement.
    ball = normalize360(ball_angle)
    goal = normalize360(home_goal_angle)
    rotated_ball = normalize360(ball + heading_correction)
    rotated_goal = normalize360(goal + heading_correction)

    angle_diff = angular_distance(ball, goal)
    blocked = (
        (angle_diff > 170.0)
        or (rotated_goal < 115.0 and rotated_ball > 115.0 and rotated_ball < 260.0)
        or (rotated_goal > 245.0 and rotated_ball < 245.0 and rotated_ball > 100.0)
    )
    if blocked:
        return -1.0

    raw_x = sin_deg(ball) + sin_deg(goal)
    raw_y = cos_deg(ball) + cos_deg(goal)
    raw_defense = normalize360(math.degrees(math.atan2(raw_x, raw_y)))

    tangent = project_tangent(line_normal_angle, raw_defense)
    final, _, _, _ = blend_components(tangent, line_normal_angle, chord_norm, cross_line)
    return final


def desired_heading_field_from_line(
    line_angle_robot: float,
    current_offset_field: float,
) -> tuple[float, float]:
    # Mirrors runDefense heading logic.
    rel_a = wrap_angle(line_angle_robot)
    rel_b = wrap_angle(line_angle_robot + 180.0)
    chosen_rel_normal = rel_a if abs(rel_a) <= abs(rel_b) else rel_b
    desired_field = wrap_angle(current_offset_field + wrap_angle(chosen_rel_normal))
    return desired_field, chosen_rel_normal


def chord_from_signed_offset_norm(offset_norm: float) -> float:
    d = abs(offset_norm)
    if d >= 1.0:
        return 0.0
    return math.sqrt(1.0 - d * d)


@dataclass
class Sample:
    theta_deg: float
    robot_x: float
    robot_y: float
    offset_norm: float
    chord_norm: float
    line_angle_robot: float
    ball_angle_robot: float
    goal_angle_robot: float
    move_angle_robot: float
    move_angle_field: float
    desired_heading_field: float
    chosen_rel_normal: float
    tangent_angle_robot: float
    correction_angle_robot: float
    normal_gain: float
    result_mag: float


def simulate_panel(
    ball_world: tuple[float, float],
    home_goal_world: tuple[float, float],
    current_offset_field: float,
    cross_line: bool,
    theta_samples_deg: np.ndarray,
    line_radius: float,
    sensor_radius_visual: float,
) -> list[Sample]:
    # Signed distance profile from line (normalized by sensor radius).
    # Positive = robot pushed outward from arc, negative = inward.
    offset_norm_profile = np.array(
        [max(-0.95, min(0.95, 0.78 * math.sin(math.radians(1.2 * t)))) for t in theta_samples_deg]
    )

    samples: list[Sample] = []
    bx, by = ball_world
    gx, gy = home_goal_world

    for theta_deg, offset_norm in zip(theta_samples_deg, offset_norm_profile):
        x_line = line_radius * sin_deg(theta_deg)
        y_line = line_radius * cos_deg(theta_deg)

        outward_world = normalize360(theta_deg)
        inward_world = normalize360(theta_deg + 180.0)
        ux, uy = angle_to_vec(outward_world)
        offset_world = offset_norm * sensor_radius_visual

        robot_x = x_line + (offset_world * ux)
        robot_y = y_line + (offset_world * uy)

        # Line normal toward nearest point on line (world), then robot frame.
        line_normal_world = inward_world if offset_norm >= 0 else outward_world
        line_angle_robot = normalize360(line_normal_world - current_offset_field)

        # Ball/goal angles as camera-like robot-relative bearings.
        ball_bearing_world = bearing_world_from_to(robot_x, robot_y, bx, by)
        goal_bearing_world = bearing_world_from_to(robot_x, robot_y, gx, gy)
        ball_angle_robot = normalize180(ball_bearing_world - current_offset_field)
        goal_angle_robot = normalize180(goal_bearing_world - current_offset_field)

        chord_norm = chord_from_signed_offset_norm(offset_norm)

        raw_x = sin_deg(ball_angle_robot) + sin_deg(goal_angle_robot)
        raw_y = cos_deg(ball_angle_robot) + cos_deg(goal_angle_robot)
        raw_def = normalize360(math.degrees(math.atan2(raw_x, raw_y)))
        tangent_angle = project_tangent(line_angle_robot, raw_def)
        _, normal_gain, correction_angle, result_mag = blend_components(
            tangent_angle, line_angle_robot, chord_norm, cross_line
        )

        move_angle_robot = defense_calc_robot_frame(
            ball_angle=ball_angle_robot,
            home_goal_angle=goal_angle_robot,
            heading_correction=current_offset_field,
            line_normal_angle=line_angle_robot,
            chord_norm=chord_norm,
            cross_line=cross_line,
        )
        move_angle_field = normalize360(current_offset_field + move_angle_robot) if move_angle_robot >= 0 else -1.0

        desired_heading_field, chosen_rel_normal = desired_heading_field_from_line(
            line_angle_robot=line_angle_robot,
            current_offset_field=current_offset_field,
        )

        samples.append(
            Sample(
                theta_deg=float(theta_deg),
                robot_x=float(robot_x),
                robot_y=float(robot_y),
                offset_norm=float(offset_norm),
                chord_norm=float(chord_norm),
                line_angle_robot=float(line_angle_robot),
                ball_angle_robot=float(ball_angle_robot),
                goal_angle_robot=float(goal_angle_robot),
                move_angle_robot=float(move_angle_robot),
                move_angle_field=float(move_angle_field),
                desired_heading_field=float(desired_heading_field),
                chosen_rel_normal=float(chosen_rel_normal),
                tangent_angle_robot=float(tangent_angle),
                correction_angle_robot=float(correction_angle),
                normal_gain=float(normal_gain),
                result_mag=float(result_mag),
            )
        )

    return samples


def print_numeric_report(samples: list[Sample], label: str) -> None:
    key_thetas = {-60.0, 0.0, 60.0}
    print(f"\n{label}")
    print("theta  ballR  goalR lineR chordN gain resMag moveR desHeadF")
    for s in samples:
        if round(s.theta_deg, 1) in key_thetas:
            print(
                f"{s.theta_deg:>5.1f} {s.ball_angle_robot:>6.1f} {s.goal_angle_robot:>6.1f} "
                f"{s.line_angle_robot:>5.1f} {s.chord_norm:>6.3f} {s.normal_gain:>4.2f} "
                f"{s.result_mag:>6.3f} {s.move_angle_robot:>5.1f} {s.desired_heading_field:>8.1f}"
            )


def main() -> None:
    line_radius = 1.0
    sensor_radius_visual = 0.11
    theta_samples = np.linspace(-60.0, 60.0, 11)

    # Robot heading field offset equivalent to compassSensor.currentOffset().
    current_offset_field = 20.0

    # Drawn fixed home goal point in world coordinates.
    home_goal_world = (0.0, 0.05)

    # Ball FIELD positions for columns (left to right).
    ball_positions = [
        (-0.90, 1.75),
        (-0.35, 1.95),
        (0.35, 1.95),
        (0.90, 1.75),
    ]

    cross_states = [False, True]

    fig, axes = plt.subplots(
        len(cross_states),
        len(ball_positions),
        figsize=(18, 9),
        sharex=True,
        sharey=True,
        constrained_layout=True,
    )

    # Visual scale for vectors (all lengths multiplied by this).
    vec_scale = 0.10

    for row, cross_line in enumerate(cross_states):
        for col, ball_world in enumerate(ball_positions):
            ax = axes[row, col]
            bx, by = ball_world
            gx, gy = home_goal_world

            samples = simulate_panel(
                ball_world=ball_world,
                home_goal_world=home_goal_world,
                current_offset_field=current_offset_field,
                cross_line=cross_line,
                theta_samples_deg=theta_samples,
                line_radius=line_radius,
                sensor_radius_visual=sensor_radius_visual,
            )

            print_numeric_report(
                samples,
                label=(
                    f"crossLine={cross_line}, ballWorld=({bx:.2f},{by:.2f}), "
                    f"homeGoal=({gx:.2f},{gy:.2f}), headingOffset={current_offset_field:.1f}"
                ),
            )

            # Curved goalie line.
            arc_theta = np.linspace(-65, 65, 220)
            x_arc = line_radius * np.sin(np.radians(arc_theta))
            y_arc = line_radius * np.cos(np.radians(arc_theta))
            ax.plot(x_arc, y_arc, "k--", linewidth=1.4)

            # Plot ball and home goal.
            ax.scatter([bx], [by], s=55, color="#ff7f0e", marker="o", zorder=6)
            ax.scatter([gx], [gy], s=55, color="#111111", marker="s", zorder=6)

            # Robot sample points and vectors.
            for s in samples:
                ax.scatter([s.robot_x], [s.robot_y], s=22, color="#666666", zorder=5)

                # Tangent component (|v| = 1.0) -> blue
                tx, ty = angle_to_vec(normalize360(current_offset_field + s.tangent_angle_robot))
                ax.arrow(
                    s.robot_x,
                    s.robot_y,
                    vec_scale * tx,
                    vec_scale * ty,
                    head_width=0.010,
                    head_length=0.014,
                    fc="#1f77b4",
                    ec="#1f77b4",
                    alpha=0.75,
                    zorder=4,
                )

                # Normal correction component (|v| = normal_gain) -> purple
                cx, cy = angle_to_vec(normalize360(current_offset_field + s.correction_angle_robot))
                ax.arrow(
                    s.robot_x,
                    s.robot_y,
                    vec_scale * s.normal_gain * cx,
                    vec_scale * s.normal_gain * cy,
                    head_width=0.009,
                    head_length=0.012,
                    fc="#9467bd",
                    ec="#9467bd",
                    alpha=0.8,
                    zorder=4,
                )

                # Resultant move vector (|v| = pre-normalization resultant magnitude) -> red
                if s.move_angle_field >= 0:
                    mvx, mvy = angle_to_vec(s.move_angle_field)
                    ax.arrow(
                        s.robot_x,
                        s.robot_y,
                        vec_scale * s.result_mag * mvx,
                        vec_scale * s.result_mag * mvy,
                        head_width=0.012,
                        head_length=0.016,
                        fc="#d62728",
                        ec="#d62728",
                        alpha=0.95,
                        zorder=6,
                    )

                # Desired heading vector from runDefense logic -> green (unit length)
                hvx, hvy = angle_to_vec(s.desired_heading_field)
                ax.arrow(
                    s.robot_x,
                    s.robot_y,
                    0.085 * hvx,
                    0.085 * hvy,
                    head_width=0.010,
                    head_length=0.014,
                    fc="#2ca02c",
                    ec="#2ca02c",
                    alpha=0.92,
                    zorder=6,
                )

            # Text for panel: center-sample ball angle in robot frame.
            center = samples[len(samples) // 2]
            ax.set_title(
                f"cross={cross_line} | ball=({bx:.2f},{by:.2f})\n"
                f"center ballAngle(robot)={center.ball_angle_robot:.1f} deg"
            )
            ax.set_aspect("equal", adjustable="box")
            ax.grid(alpha=0.2)
            ax.set_xlim(-1.15, 1.15)
            ax.set_ylim(0.00, 2.15)
            if row == len(cross_states) - 1:
                ax.set_xlabel("X")
            if col == 0:
                ax.set_ylabel("Y")

    legend_lines = [
        plt.Line2D([0], [0], color="#1f77b4", lw=2, label="Tangent component (mag=1.0)"),
        plt.Line2D([0], [0], color="#9467bd", lw=2, label="Normal correction component (mag=normal_gain)"),
        plt.Line2D([0], [0], color="#d62728", lw=2, label="Resultant move component (mag=result_mag)"),
        plt.Line2D([0], [0], color="#2ca02c", lw=2, label="Desired heading from runDefense"),
        plt.Line2D([0], [0], marker="o", color="w", markerfacecolor="#ff7f0e", markersize=8, label="Ball position"),
        plt.Line2D([0], [0], marker="s", color="w", markerfacecolor="#111111", markersize=8, label="Home goal"),
    ]
    fig.legend(handles=legend_lines, loc="lower center", ncol=3, frameon=False)
    fig.suptitle(
        "Defense Sim: Curved Goalie Line with Field Ball Positions and Runtime-Equivalent Heading",
        fontsize=14,
    )

    out_png = Path(__file__).with_name("defense_curve_sim.png")
    fig.savefig(out_png, dpi=180)
    print(f"\nwrote: {out_png.resolve()}")


if __name__ == "__main__":
    main()
