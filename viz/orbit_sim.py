"""
Orbit path simulator for RoboCup Jr Soccer (Offense2026).

Faithfully ports the embedded angle conventions so the simulated paths reflect
the real robot, then compares the CURRENT goal-blind orbit against a proposed
GOAL-AWARE orbit.

Conventions (verified against src/trig.cpp::getAngle and src/Movement.cpp):
  * Field frame: +y points toward the attacked goal, +x is right.
  * Bearings are measured CLOCKWISE from +y:  unit(theta) = (sin theta, cos theta),
    so a field bearing to point P from R is atan2(dx, dy).
  * movement(a) translates the chassis toward body bearing (heading + a); 0 = front.
  * ballAngle: 0..360 robot-relative (0 = ball dead ahead).
  * goalAngle: -180..180 robot-relative (0 = goal dead ahead).
  * ballDist: cm.
"""

import math
import numpy as np


def norm360(a):
    a = math.fmod(a, 360.0)
    if a < 0:
        a += 360.0
    return a


def norm180(a):
    a = norm360(a)
    if a > 180.0:
        a -= 360.0
    return a


# --------------------------------------------------------------------------
# CURRENT orbit: exact port of Orbit::CalculateRobotAngle (src/orbit.cpp)
# --------------------------------------------------------------------------
def orbit_current(ball_angle, dist, derivative=-5, sample_time=0, kd=0.3):
    dTerm = 0.0
    if derivative != -5 and sample_time > 0:
        dTerm = kd * (derivative / (sample_time / 1000.0))

    d = dist / 150.0
    if d > 1:
        d = 1.0
    d = 1.0 - d
    dampen = min(1.0, 0.02 * math.exp(4.5 * d))

    newball = (360.0 - ball_angle) if ball_angle > 180 else ball_angle
    orbit_value = min(90.0, 4.0 * math.exp(0.1 * (newball - 30.0)))

    out = orbit_value * dampen
    if dTerm > 3:
        out -= dTerm

    ra = ball_angle + (-1 if ball_angle > 180 else 1) * out
    return norm360(ra)


# --------------------------------------------------------------------------
# PROPOSED goal-aware orbit.
#
# Core idea ("target point" from the user's sketch): steer toward the point that
# sits a fixed contact distance db BEHIND the ball, on the ball->goal line, so at
# contact robot->ball->goal are colinear.  In robot-relative terms (far-goal
# approximation: ball->goal direction ~ goalAngle):
#
#     T = ball_vec - db * unit(goalAngle)
#     m_core = atan2(Tx, Ty)
#
# A tangential "go-around" term is added so we commit to arcing onto the
# anti-goal side instead of shoving the ball when we are on the goal side
# (|Delta|>90), reusing the team's trusted exp distance-dampening.
# --------------------------------------------------------------------------
def orbit_goal_aware(ball_angle, dist, goal_angle,
                     db=11.0, k_tan=55.0, reach=150.0):
    Delta = norm180(goal_angle - ball_angle)   # where the goal is, relative to ball

    # ---- target-point core ----
    bx = dist * math.sin(math.radians(ball_angle))
    by = dist * math.cos(math.radians(ball_angle))
    tx = bx - db * math.sin(math.radians(goal_angle))
    ty = by - db * math.cos(math.radians(goal_angle))
    m_core = math.degrees(math.atan2(tx, ty))
    delta_core = norm180(m_core - ball_angle)

    # ---- tangential go-around boost ----
    # decays with distance the same way the legacy orbit does (commit hard far out,
    # ease in as we close); pushes toward the anti-goal side of the ball.
    d = dist / reach
    if d > 1:
        d = 1.0
    d = 1.0 - d
    dampen = min(1.0, 0.02 * math.exp(4.5 * d))
    # sin(Delta) is 0 when aligned/anti-aligned and max at 90 deg; sign turns us
    # toward the side that gets us behind the ball.  The (1-cos) factor keeps the
    # boost alive through the Delta~180 wrong-side case so we don't stall.
    boost = -k_tan * math.sin(math.radians(Delta)) * dampen
    # break the unstable Delta==180 tie deterministically
    if abs(Delta) > 179.0:
        boost = -k_tan * dampen   # arc to a fixed side

    delta = delta_core + boost
    return norm360(ball_angle + delta), delta_core, boost


# --------------------------------------------------------------------------
# Trajectory simulation (holonomic translation + heading tracking the goal)
# --------------------------------------------------------------------------
def simulate(robot_xy, ball_xy, goal_xy, controller,
             speed=40.0, dt=0.02, omega_max=240.0, max_steps=3000,
             contact_cm=11.0, heading0=None):
    """Returns dict with path, headings, captured flag, contact-alignment error.

    speed cm/s, omega_max deg/s. 'controller' is callable(ballAngle,dist,goalAngle)
    -> movement_angle (robot-relative deg).
    """
    R = np.array(robot_xy, float)
    B = np.array(ball_xy, float)
    G = np.array(goal_xy, float)

    # initial heading: face the goal
    gdir = math.degrees(math.atan2(G[0] - R[0], G[1] - R[1]))
    h = gdir if heading0 is None else heading0

    path = [R.copy()]
    headings = [h]
    captured = False
    align_err = None

    for _ in range(max_steps):
        dxb, dyb = B - R
        ball_field = math.degrees(math.atan2(dxb, dyb))
        ball_dist = math.hypot(dxb, dyb)
        ball_angle = norm360(ball_field - h)

        dxg, dyg = G - R
        goal_field = math.degrees(math.atan2(dxg, dyg))
        goal_angle = norm180(goal_field - h)

        # capture test: close to ball AND ball roughly in front (in dribbler)
        if ball_dist <= contact_cm and abs(norm180(ball_angle)) < 35:
            captured = True
            # alignment error = angle between (robot->ball) and (ball->goal)
            ball_to_goal = math.degrees(math.atan2(G[0] - B[0], G[1] - B[1]))
            robot_to_ball = ball_field
            align_err = abs(norm180(ball_to_goal - robot_to_ball))
            break

        m = controller(ball_angle, ball_dist, goal_angle)

        # translate toward body bearing (h + m)
        world = math.radians(h + m)
        R = R + speed * dt * np.array([math.sin(world), math.cos(world)])
        path.append(R.copy())

        # rotate heading toward goal
        dh = norm180(goal_field - h)
        dh = max(-omega_max * dt, min(omega_max * dt, dh))
        h += dh
        headings.append(h)

    return {
        "path": np.array(path),
        "headings": np.array(headings),
        "captured": captured,
        "align_err": align_err,
        "ball": B, "goal": G, "robot0": np.array(robot_xy, float),
    }


if __name__ == "__main__":
    # quick smoke test
    cur = lambda ba, d, ga: orbit_current(ba, d)
    new = lambda ba, d, ga: orbit_goal_aware(ba, d, ga)[0]
    for name, ctl in [("current", cur), ("goal_aware", new)]:
        s = simulate((-40, -60), (0, 0), (0, 120), ctl)
        print(f"{name:11s} captured={s['captured']} steps={len(s['path'])} "
              f"align_err={s['align_err']}")
