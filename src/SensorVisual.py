import serial
import threading
import re
import time

import matplotlib.pyplot as plt
import matplotlib.colors as colors
import numpy as np

# =========================
# USER CONFIG
# =========================
SERIAL_PORT = "COM4"     # change as needed
BAUD_RATE = 115200
MAX_SENSOR_VALUE = 1023
UPDATE_RATE = 0.01

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
sensor_values = np.zeros(NUM_SENSORS)
centroid = np.array([0.0, 0.0])
line_angle_deg = 0.0

lock = threading.Lock()

# =========================
# REGEX PATTERNS
# =========================
sensor_pattern = re.compile(r"Sensor\s+(\d+):\s+(\d+)")
angle_pattern = re.compile(r"Line Angle:\s*([-+]?\d*\.?\d+)")
centroid_pattern = re.compile(
    r"Centroid:\s*\(\s*([-+]?\d*\.?\d+)\s*,\s*([-+]?\d*\.?\d+)\s*\)"
)

# =========================
# SERIAL THREAD
# =========================
def serial_reader():
    global line_angle_deg, centroid

    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print("Serial connected")

    while True:
        line = ser.readline().decode(errors="ignore").strip()

        if m := sensor_pattern.match(line):
            idx = int(m.group(1)) - 1
            val = int(m.group(2))
            if 0 <= idx < NUM_SENSORS:
                with lock:
                    sensor_values[idx] = val

        elif m := angle_pattern.match(line):
            with lock:
                line_angle_deg = float(m.group(1))
                # print(line_angle_deg)

        elif m := centroid_pattern.match(line):
            with lock:
                centroid = np.array([float(m.group(1)), float(m.group(2))])
                print(centroid)

# =========================
# PLOTTING SETUP
# =========================
plt.ion()
fig, ax = plt.subplots(figsize=(7, 7))

sc = ax.scatter(
    coords[:, 0],
    coords[:, 1],
    c=sensor_values,
    cmap="inferno",
    s=120,
    norm=colors.Normalize(vmin=0, vmax=MAX_SENSOR_VALUE)
)

centroid_plot, = ax.plot(0, 0, "ro", markersize=8, label="Centroid")
line_plot, = ax.plot([], [], "g-", linewidth=2, label="Line Angle")

plt.colorbar(sc, label="Sensor Value")

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
        sc.set_array(sensor_values)
        sc.set_sizes(50 + (sensor_values / MAX_SENSOR_VALUE) * 300)

        # Update centroid
        centroid_plot.set_data([centroid[0]], [centroid[1]])

        # Draw line only if valid
        if line_angle_deg != -5:
            angle_rad = np.deg2rad(line_angle_deg)
            dx = np.cos(angle_rad)
            dy = np.sin(angle_rad)

            t = np.array([-limit, limit])

            x_line = centroid[0] + dx * t
            y_line = centroid[1] + dy * t

            line_plot.set_data(x_line, y_line)
        else:
            # Hide line
            line_plot.set_data([], [])

    plt.pause(UPDATE_RATE)
