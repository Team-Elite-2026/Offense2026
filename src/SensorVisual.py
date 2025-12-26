import serial
import threading
import re
import time

import matplotlib.pyplot as plt
import numpy as np

# =========================
# USER CONFIG
# =========================
SERIAL_PORT = "/dev/cu.usbmodem172973501"     # change as needed
BAUD_RATE = 115200
UPDATE_RATE = 0.1

# =========================
# SENSOR COORDINATES
# =========================
coords = np.array([
    (86.5, 0), (85.63, -12.219), (80.169, -18.904), (71.206, -24.997),
    (64.174, -35.019), (57.142, -45.062), (50.110, -55.104), (43.602, -65.388),
    (44.506, -74.275), (33.562, -79.777), (21.954, -83.689), (9.912, -85.394),
    (-2.326, -86.469), (-14.518, -85.282), (-26.423, -82.397), (-37.805, -77.871),
    (-47.732, -71.282), (-45.747, -61.336), (-52.779, -51.293), (-59.811, -41.251),
    (-66.843, -31.208), (-73.874, -21.166), (-84.136, -19.660), (-86.166, -7.597),
    (-86.375, 4.650), (-84.852, 16.804), (-75.774, 18.931), (-68.537, 28.788),
    (-61.506, 38.830), (-54.474, 48.873), (-47.442, 58.915), (-45.728, 69.253),
    (-40.440, 76.541), (-31.672, 72.424), (-21.946, 65.169), (-9.933, 63.291),
    (2.326, 63.291), (14.583, 63.379), (26.052, 67.343), (34.246, 76.291),
    (44.465, 74.207), (43.602, 65.388), (50.110, 55.104), (57.142, 45.062),
    (64.174, 35.019), (71.206, 24.977), (80.169, 18.904), (85.633, 12.219)
])

NUM_SENSORS = len(coords)

# =========================
# SHARED STATE
# =========================
centroid = np.array([0.0, 0.0])
angle_to_line_deg = None

lock = threading.Lock()

# =========================
# REGEX PATTERNS
# =========================
angle_pattern = re.compile(r"Line Angle:\s*([-+]?\d*\.?\d+)")
centroid_pattern = re.compile(
    r"Centroid:\s*\(\s*([-+]?\d*\.?\d+)\s*,\s*([-+]?\d*\.?\d+)\s*\)"
)

# =========================
# SERIAL THREAD
# =========================
def serial_reader():
    global angle_to_line_deg, centroid

    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print("Serial connected")

    while True:
        line = ser.readline().decode(errors="ignore").strip()

        if m := angle_pattern.match(line):
            angle_val = float(m.group(1))
            with lock:
                # Use angle directly from C++ (already calculated there)
                if angle_val == -5:
                    angle_to_line_deg = None
                else:
                    angle_to_line_deg = angle_val

        elif m := centroid_pattern.match(line):
            with lock:
                centroid = np.array([float(m.group(1)), float(m.group(2))])
                print(centroid)

# =========================
# PLOTTING SETUP
# =========================
plt.ion()
fig, ax = plt.subplots(figsize=(7, 7))

# Show sensor positions as gray dots (static)
ax.scatter(coords[:, 0], coords[:, 1], c='gray', s=50, alpha=0.3, label="Sensor Positions")

centroid_plot, = ax.plot(0, 0, "ro", markersize=8, label="Centroid")
line_plot, = ax.plot([], [], "g-", linewidth=2, label="Line Angle")

# Text for displaying angle to line intersection
angle_text = ax.text(-95, -95, "", fontsize=12, color="white", 
                     bbox=dict(boxstyle="round,pad=0.5", facecolor="black", alpha=0.7))

ax.set_aspect("equal")
ax.set_title("Live Line Sensor + Line Angle Visualization")
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.grid(True)
ax.legend()

# Plot limits
limit = 100
ax.set_xlim(-limit, limit)
ax.set_ylim(-limit, limit)

# =========================
# START SERIAL THREAD
# =========================
threading.Thread(target=serial_reader, daemon=True).start()

# =========================
# MAIN LOOP
# =========================
while True:
    with lock:
        # Update centroid
        centroid_plot.set_data([centroid[0]], [centroid[1]])

        # Use angle directly from C++ (already calculated there)
        if angle_to_line_deg is not None:
            # Draw line based on the angle from C++
            # The angle represents direction from robot to line intersection
            # Convert angle to radians and draw line through centroid
            angle_rad = np.deg2rad(angle_to_line_deg)
            
            # Calculate line direction (perpendicular to the angle direction)
            # If angle is direction TO line, line is perpendicular to that
            line_angle_rad = angle_rad + np.pi/2  # Perpendicular
            
            dx = np.cos(line_angle_rad)
            dy = np.sin(line_angle_rad)
            
            # Extend line across plot limits
            t = np.array([-limit * 2, limit * 2])
            x_line = centroid[0] + dx * t
            y_line = centroid[1] + dy * t
            
            line_plot.set_data(x_line, y_line)
            angle_text.set_text(f"Angle to line: {angle_to_line_deg:.2f}°")
        else:
            # Hide line if no angle detected
            line_plot.set_data([], [])
            angle_text.set_text("Angle to line: N/A")

    plt.pause(UPDATE_RATE)
