import re
import threading
import time
from collections import deque

import matplotlib.pyplot as plt

try:
    import serial
except ImportError as exc:
    raise SystemExit(
        "pyserial is required for VariablePlotter.py. Install it with `python3 -m pip install pyserial`."
    ) from exc


SCALAR_PATTERN = re.compile(
    r"^\s*([^:]+?)\s*:\s*([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)\s*$"
)

# =========================
# USER CONFIG
# =========================
SERIAL_PORT = "/dev/cu.Bluetooth-Incoming-Port"
BAUD_RATE = 9600
WINDOW_SECONDS = 10.0
REFRESH_SECONDS = 0.02
SERIAL_TIMEOUT = 0.01
MAX_SAMPLES = 4000
LIST_ONLY = False

# Leave empty to plot every scalar label found on Serial.
VARIABLES_TO_PLOT = [
    "Desired Heading"
]


class SharedState:
    def __init__(self, max_samples, selected_variables):
        self.max_samples = max_samples
        self.selected_variables = set(selected_variables)
        self.lock = threading.Lock()
        self.start_time = time.monotonic()
        self.last_data_time = None
        self.new_data_event = threading.Event()
        self.discovered_labels = set()
        self.series = {}

    def should_track(self, label):
        return not self.selected_variables or label in self.selected_variables

    def ensure_series(self, label):
        if label not in self.series:
            self.series[label] = {
                "time": deque(maxlen=self.max_samples),
                "value": deque(maxlen=self.max_samples),
            }
        return self.series[label]

    def add_point(self, label, value):
        now = time.monotonic() - self.start_time
        with self.lock:
            self.discovered_labels.add(label)
            if not self.should_track(label):
                return

            series = self.ensure_series(label)
            series["time"].append(now)
            series["value"].append(value)
            self.last_data_time = now
            self.new_data_event.set()

    def snapshot(self):
        with self.lock:
            copied = {
                label: (list(points["time"]), list(points["value"]))
                for label, points in self.series.items()
            }
            return copied, self.last_data_time, set(self.discovered_labels)


def serial_reader(port, baud, serial_timeout, state):
    ser = serial.Serial(port, baud, timeout=serial_timeout)
    ser.reset_input_buffer()
    print(f"Connected to {port} @ {baud}")

    while True:
        raw = ser.readline().decode(errors="ignore").strip()
        if not raw:
            continue

        match = SCALAR_PATTERN.match(raw)
        if not match:
            continue

        label = match.group(1).strip()
        value = float(match.group(2))
        state.add_point(label, value)


def run_list_only(state):
    seen = set()
    print("Watching serial stream. Press Ctrl-C to stop.")
    while True:
        _, _, discovered = state.snapshot()
        new_labels = sorted(discovered - seen)
        for label in new_labels:
            print(label)
        seen |= discovered
        time.sleep(0.05)


def run_plot(state, window_seconds, refresh_seconds):
    plt.ion()
    fig, ax = plt.subplots(figsize=(11, 6))
    status_text = ax.text(
        0.02,
        0.98,
        "",
        transform=ax.transAxes,
        va="top",
        fontsize=10,
        color="tab:red",
    )
    ax.set_title("Live Variable Plot")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Value")
    ax.grid(True, alpha=0.3)

    lines = {}

    while True:
        state.new_data_event.wait(timeout=refresh_seconds)
        state.new_data_event.clear()

        series_snapshot, last_data_time, discovered = state.snapshot()
        plotted_any = False
        y_min = None
        y_max = None

        for label, (x_vals, y_vals) in sorted(series_snapshot.items()):
            if label not in lines:
                (line,) = ax.plot([], [], linewidth=2, label=label)
                lines[label] = line
                ax.legend(loc="upper right")

            line = lines[label]
            if not x_vals:
                continue

            cutoff = max(0.0, x_vals[-1] - window_seconds)
            first_visible_index = 0
            while first_visible_index < len(x_vals) and x_vals[first_visible_index] < cutoff:
                first_visible_index += 1

            visible_x = x_vals[first_visible_index:]
            visible_y = y_vals[first_visible_index:]
            if not visible_x:
                continue

            line.set_data(visible_x, visible_y)
            plotted_any = True

            local_min = min(visible_y)
            local_max = max(visible_y)
            y_min = local_min if y_min is None else min(y_min, local_min)
            y_max = local_max if y_max is None else max(y_max, local_max)

        for label, line in lines.items():
            if label not in series_snapshot:
                line.set_data([], [])

        if plotted_any:
            latest_time = max(times[-1] for times, _ in series_snapshot.values() if times)
            ax.set_xlim(max(0.0, latest_time - window_seconds), max(window_seconds, latest_time))
            if y_min == y_max:
                padding = 1.0
            else:
                padding = max(0.5, (y_max - y_min) * 0.1)
            ax.set_ylim(y_min - padding, y_max + padding)
            status_text.set_text("")
        else:
            status_text.set_text(
                "No matching scalar data yet.\n"
                "Print lines like `Line Angle: 123.4` from the firmware."
            )

        now_relative = time.monotonic() - state.start_time
        if last_data_time is not None and (now_relative - last_data_time) > 1.0:
            status_text.set_text("Data stream paused.")

        if not series_snapshot and discovered:
            status_text.set_text(
                "Saw scalar labels, but none match --var.\n"
                f"Available: {', '.join(sorted(discovered))}"
            )

        fig.canvas.draw_idle()
        fig.canvas.flush_events()
        plt.pause(0.001)


def main():
    state = SharedState(MAX_SAMPLES, VARIABLES_TO_PLOT)

    threading.Thread(
        target=serial_reader,
        args=(SERIAL_PORT, BAUD_RATE, SERIAL_TIMEOUT, state),
        daemon=True,
    ).start()

    if LIST_ONLY:
        run_list_only(state)
        return

    run_plot(state, WINDOW_SECONDS, REFRESH_SECONDS)


if __name__ == "__main__":
    main()
