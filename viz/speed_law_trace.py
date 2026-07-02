import math, numpy as np
from orbit_sim import orbit_goal_aware, norm180, norm360

FLOOR, AMP = 0.17, 0.11
law_cos_robot  = lambda ra, ba: FLOOR + AMP*abs(math.cos(math.radians(ra)))
law_cos_offset = lambda ra, ba: FLOOR + AMP*max(0.0, math.cos(math.radians(norm180(ra-ba))))

ball = np.array([0.0, 0.0]); goal = np.array([0.0, 140.0])
R = np.array([15.0, 55.0])                      # on the GOAL side -> full swing
h = math.degrees(math.atan2(goal[0]-R[0], goal[1]-R[1]))

print(f"{'dist':>5} {'ballAng':>8} {'robotAng':>9} {'offset':>7} "
      f"{'|cos(robot)|':>12} {'cos(offset)':>11}  note")
for i in range(600):
    d = ball - R; dist = math.hypot(*d)
    if dist < 15: break
    bf = math.degrees(math.atan2(d[0], d[1])); ba = norm360(bf - h)
    ga = norm180(math.degrees(math.atan2(goal[0]-R[0], goal[1]-R[1])) - h)
    ra = orbit_goal_aware(ba, dist, ga, db=12, k_tan=50)[0]
    off = norm180(ra - ba)
    if i % 12 == 0:
        note = "LATERAL swing" if abs(off) > 55 else ""
        print(f"{dist:5.1f} {ba:8.1f} {ra:9.1f} {off:7.1f} "
              f"{law_cos_robot(ra,ba):12.3f} {law_cos_offset(ra,ba):11.3f}  {note}")
    world = math.radians(h + ra); R = R + 2.0*np.array([math.sin(world), math.cos(world)])
    gf = math.degrees(math.atan2(goal[0]-R[0], goal[1]-R[1])); h += max(-4, min(4, norm180(gf-h)))
