import argparse
import re
import threading
import time
from collections import deque

import matplotlib.pyplot as plt
import serial


offset_values = deque()
time_values = deque()
data_lock = threading.Lock()
last_match_time = None
new_data_event = threading.Event()


def parse_args():
    parser = argparse.ArgumentParser(
        description="Live plot of CompassSensor currentOffset() over time."
    )
    parser.add_argument(
        "--port",
        default="/dev/cu.usbmodem172973501",
        help="Serial port (example: /dev/cu.usbmodem172973501 or COM3)",
    )
    parser.add_argument("--baud", type=int, default=9600, help="Serial baud rate")
    parser.add_argument(
        "--serial-timeout",
        type=float,
        default=0.02,
        help="Serial read timeout in seconds (lower = more responsive)",
    )
    parser.add_argument(
        "--window",
        type=float,
        default=20.0,
        help="Rolling time window in seconds",
    )
    parser.add_argument(
        "--refresh",
        type=float,
        default=0.05,
        help="Plot refresh period in seconds",
    )
    return parser.parse_args()


def serial_reader(port, baud, window_seconds, serial_timeout):
    global last_match_time
    # Supports lines like:
    #  - "12" (integer-only line)
    #  - "Offset: 12"
    #  - "Current Offset: -37"
    labeled_pattern = re.compile(r"offset[^-\d]*(-?\d+)", re.IGNORECASE)
    integer_only_pattern = re.compile(r"^\s*(-?\d+)\s*$")

    ser = serial.Serial(port, baud, timeout=serial_timeout)
    ser.reset_input_buffer()
    start_time = time.monotonic()
    print(f"Connected to {port} @ {baud}")

    while True:
        raw = ser.readline().decode(errors="ignore").strip()
        if not raw:
            continue

        # If plot/render can't keep up, discard stale backlog and keep only newest line.
        while ser.in_waiting > 0:
            newer = ser.readline().decode(errors="ignore").strip()
            if newer:
                raw = newer

        labeled_match = labeled_pattern.search(raw)
        integer_match = integer_only_pattern.match(raw)

        if labeled_match:
            offset = int(labeled_match.group(1))
        elif integer_match:
            offset = int(integer_match.group(1))
        else:
            continue
        t = time.monotonic() - start_time

        with data_lock:
            time_values.append(t)
            offset_values.append(offset)
            last_match_time = t
            new_data_event.set()

            cutoff = t - window_seconds
            while time_values and time_values[0] < cutoff:
                time_values.popleft()
                offset_values.popleft()


def main():
    args = parse_args()

    plt.ion()
    fig, ax = plt.subplots(figsize=(10, 5))
    line, = ax.plot([], [], linewidth=2, color="tab:blue", label="Current Offset")
    status_text = ax.text(
        0.02,
        0.95,
        "",
        transform=ax.transAxes,
        va="top",
        fontsize=10,
        color="tab:red",
    )
    ax.axhline(0, linestyle="--", linewidth=1, color="tab:gray", label="Target (0)")
    ax.set_title("Compass Offset vs Time")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Offset (deg)")
    ax.set_ylim(-190, 190)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")

    threading.Thread(
        target=serial_reader,
        args=(args.port, args.baud, args.window, args.serial_timeout),
        daemon=True,
    ).start()

    session_start_time = time.monotonic()
    while True:
        # Wake on new data; timeout keeps UI responsive if stream pauses.
        new_data_event.wait(timeout=args.refresh)
        new_data_event.clear()

        with data_lock:
            x = list(time_values)
            y = list(offset_values)
            local_last_match_time = last_match_time

        if x:
            line.set_data(x, y)
            ax.set_xlim(max(0.0, x[-1] - args.window), max(args.window, x[-1]))
            if y:
                y_min = max(-190, min(y) - 10)
                y_max = min(190, max(y) + 10)
                if y_min < y_max:
                    ax.set_ylim(y_min, y_max)
            status_text.set_text("")
        else:
            status_text.set_text("No offset data received yet.\nCheck COM port, baud, and firmware Serial output.")

        now_relative = time.monotonic() - session_start_time
        if local_last_match_time is not None and (now_relative - local_last_match_time) > 1.0:
            status_text.set_text("Offset data stream paused.")

        fig.canvas.draw_idle()
        fig.canvas.flush_events()
        plt.pause(0.001)


if __name__ == "__main__":
    main()
