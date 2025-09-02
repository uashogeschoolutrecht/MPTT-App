import asyncio
import sys
import os
import time
from collections import deque
from typing import List, Optional

import numpy as np

# Qt and Matplotlib Qt backend
from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QPushButton,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QCheckBox,
    QSlider,
    QSizePolicy,
    QLineEdit,
    QTextEdit,
)
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

from qasync import QEventLoop

# Add the parent directory to the Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.sensor import MovellaDOTSensor
from models.data_structures import SensorConfiguration
from models.enums import OutputRate, FilterProfile, PayloadMode
from bleak import BleakScanner, BleakClient


# ---------------------- Math helpers (unchanged) ----------------------
def _quat_normalize(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=np.float64)
    n = np.linalg.norm(q, axis=-1, keepdims=True)
    n[n == 0] = 1.0
    return q / n


def _quat_conj(q: np.ndarray) -> np.ndarray:
    # q = [w, x, y, z]
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float64)


def _quat_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    # Hamilton product, quats in [w, x, y, z]
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ], dtype=np.float64)


def _quat_to_euler_deg(q: np.ndarray) -> np.ndarray:
    """Convert unit quaternion [w,x,y,z] to ZYX euler angles [roll, pitch, yaw] in degrees.
    roll: x (left/right), pitch: y (forward/backward), yaw: z (twist)
    """
    w, x, y, z = q
    # roll (x-axis rotation) - negated to match sensor convention
    sinr_cosp = -2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = np.degrees(np.arctan2(sinr_cosp, cosr_cosp))

    # pitch (y-axis rotation) - negated to match sensor convention
    sinp = -2.0 * (w * y - z * x)
    sinp = np.clip(sinp, -1.0, 1.0)
    pitch = np.degrees(np.arcsin(sinp))

    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.degrees(np.arctan2(siny_cosp, cosy_cosp))

    return np.array([roll, pitch, yaw], dtype=np.float64)


# ---------------------- Qt GUI ----------------------
class TiltBallWindow(QMainWindow):
    def __init__(self, sensors: List[MovellaDOTSensor], parent=None):
        super().__init__(parent)
        self.setWindowTitle("Spiral Tracking - PySide6")
        self.sensors = sensors
        self.sensor: Optional[MovellaDOTSensor] = sensors[0] if sensors else None

        # State
        self.xy_range = 30.0
        self.step = 5.0
        self.min_range = 5.0
        self.max_range = 180.0
        # Start with measurement stopped and target not moving
        self.is_running = False
        self.calib_q: Optional[np.ndarray] = None
        self.pending_calibration = False
        # Measurement timing
        self.measure_start_t: Optional[float] = None
        # Measurement path storage (recorded while running)
        self.meas_x: List[float] = []
        self.meas_y: List[float] = []
        # Full sample logs (time, values)
        self.samples_t: List[float] = []  # seconds since start
        self.samples_roll: List[float] = []
        self.samples_pitch: List[float] = []
        self.samples_x: List[float] = []
        self.samples_y: List[float] = []
        self.samples_tx: List[float] = []
        self.samples_ty: List[float] = []
        self.samples_err: List[float] = []
        self.samples_quat: List[List[float]] = []  # [w,x,y,z]

        # Moving average buffers
        self.buf_roll: deque = deque(maxlen=5)
        self.buf_pitch: deque = deque(maxlen=5)

        # Error tracking
        # Default window equals default target duration (60s)
        self.err_window_sec = 60.0
        self.err_times: deque = deque()
        self.err_values: deque = deque()
        self.t0 = time.monotonic()
        self.theta_current = 0.0

        # Build UI
        central = QWidget(self)
        self.setCentralWidget(central)
        root = QHBoxLayout(central)

        # Matplotlib Figure and Canvas
        self.fig = Figure(figsize=(8, 7), dpi=100)
        gs = self.fig.add_gridspec(3, 1, height_ratios=[4.0, 0.2, 1.0])
        self.ax = self.fig.add_subplot(gs[0])
        self.ax_err = self.fig.add_subplot(gs[2])
        self.canvas = FigureCanvas(self.fig)
        self.canvas.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # Configure axes
        self._setup_main_axes()
        self._setup_error_axes()

        root.addWidget(self.canvas, 2)

        # Controls panel
        controls = QVBoxLayout()

        self.lbl_deg = QLabel("L/R: +0.0°, F/B: +0.0°", self)
        self.lbl_deg.setAlignment(Qt.AlignCenter)
        controls.addWidget(self.lbl_deg)

        self.lbl_settings = QLabel(self._settings_text(False, 5, 60), self)
        self.lbl_settings.setAlignment(Qt.AlignCenter)
        controls.addWidget(self.lbl_settings)

        # Buttons row 1: Calibrate, Start/Stop
        row1 = QHBoxLayout()
        self.btn_cal = QPushButton("Calibrate", self)
        # Default to Start when app opens
        self.btn_toggle = QPushButton("Start", self)
        row1.addWidget(self.btn_cal)
        row1.addWidget(self.btn_toggle)
        controls.addLayout(row1)

        # Buttons row 2: Range -, Range +, Exit
        row2 = QHBoxLayout()
        self.btn_rminus = QPushButton("Range -", self)
        self.btn_rplus = QPushButton("Range +", self)
        self.btn_exit = QPushButton("Exit", self)
        row2.addWidget(self.btn_rminus)
        row2.addWidget(self.btn_rplus)
        row2.addWidget(self.btn_exit)
        controls.addLayout(row2)

        # Filter toggle
        self.chk_filter = QCheckBox("Filter")
        controls.addWidget(self.chk_filter)

        # Filter window slider
        self.lbl_win = QLabel("N = 5", self)
        self.s_win = QSlider(Qt.Horizontal, self)
        self.s_win.setRange(3, 150)
        self.s_win.setValue(5)
        self.s_win.setSingleStep(1)
        controls.addWidget(self.lbl_win)
        controls.addWidget(self.s_win)

        # Target duration slider
        self.lbl_target = QLabel("Target T = 60 s", self)
        self.s_target = QSlider(Qt.Horizontal, self)
        self.s_target.setRange(30, 120)
        self.s_target.setValue(60)
        self.s_target.setSingleStep(1)
        controls.addWidget(self.lbl_target)
        controls.addWidget(self.s_target)

        # Session info & save section
        controls.addSpacing(8)
        controls.addWidget(QLabel("Session info", self))
        row_subj = QHBoxLayout()
        row_subj.addWidget(QLabel("Subject ID:"))
        self.in_subject = QLineEdit(self)
        self.in_subject.setPlaceholderText("e.g. 001")
        self.in_subject.setMaxLength(32)
        row_subj.addWidget(self.in_subject)
        controls.addLayout(row_subj)

        row_expr = QHBoxLayout()
        row_expr.addWidget(QLabel("Experimenter:"))
        self.in_experimenter = QLineEdit(self)
        self.in_experimenter.setPlaceholderText("Name/initials")
        row_expr.addWidget(self.in_experimenter)
        controls.addLayout(row_expr)

        controls.addWidget(QLabel("Remarks:"))
        self.in_remarks = QTextEdit(self)
        self.in_remarks.setPlaceholderText("Notes, conditions, etc.")
        self.in_remarks.setFixedHeight(70)
        controls.addWidget(self.in_remarks)

        self.btn_save = QPushButton("Save", self)
        self.btn_save.setEnabled(False)
        controls.addWidget(self.btn_save)

        controls.addStretch(1)
        container = QWidget(self)
        container.setLayout(controls)
        container.setMinimumWidth(320)
        root.addWidget(container, 1)

        # Signals
        self.btn_cal.clicked.connect(self._on_calibrate)
        self.btn_toggle.clicked.connect(self._on_toggle_run)
        self.btn_rminus.clicked.connect(lambda: self._set_range(self.xy_range - self.step))
        self.btn_rplus.clicked.connect(lambda: self._set_range(self.xy_range + self.step))
        self.btn_exit.clicked.connect(self.close)
        self.chk_filter.toggled.connect(self._on_filter_toggled)
        self.s_win.valueChanged.connect(self._on_filter_size_changed)
        self.s_target.valueChanged.connect(self._on_target_changed)
        self.btn_save.clicked.connect(self._on_save)

        # Update timer
        self.timer = QTimer(self)
        self.timer.setInterval(10)  # ~100 Hz
        self.timer.timeout.connect(self._on_tick)
        self.timer.start()

    # ------------- Axes setup -------------
    def _setup_main_axes(self):
        self.ax.clear()
        self.ax.set_xlim(-self.xy_range, self.xy_range)
        self.ax.set_ylim(-self.xy_range, self.xy_range)
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.grid(True, linestyle=':', alpha=0.5)
        self.ax.axhline(0, color='k', lw=0.5)
        self.ax.axvline(0, color='k', lw=0.5)

        # Reference path
        self.num_path_pts = 1000
        self.theta_path, self.x_path, self.y_path = self._compute_path_arrays(self.xy_range)
        (self.path_line,) = self.ax.plot(self.x_path, self.y_path, color='green', lw=1.2, alpha=0.35)
        (self.target_dot,) = self.ax.plot([self.x_path[0]], [self.y_path[0]], 'o', color='green', markersize=10, alpha=0.9)

        # Balls
        (self.ball,) = self.ax.plot([0], [0], 'o', color='red', markersize=12, label='Manual Calc')
        # Measurement path line (initially empty/hidden)
        (self.meas_line,) = self.ax.plot([], [], '-', color='orange', lw=1.5, alpha=0.85, label='Measurement Path')
        self.meas_line.set_visible(False)
        self.ax.legend(loc='upper right', fontsize=9)

        self.canvas.draw_idle()

    def _setup_error_axes(self):
        self.ax_err.clear()
        self.ax_err.set_title('Abs Error (deg)', fontsize=9)
        self.ax_err.set_xlim(0.0, self.err_window_sec)
        self.ax_err.set_ylim(0.0, 3.0 * self.xy_range)
        self.ax_err.grid(True, linestyle=':', alpha=0.3)
        (self.err_line,) = self.ax_err.plot([], [], color='tab:red', lw=1.0)
        self.canvas.draw_idle()

    # ------------- Helpers -------------
    def _compute_path_arrays(self, range_val: float):
        theta = np.linspace(0.0, 2.0 * np.pi, self.num_path_pts)
        r = range_val * np.cos(2.0 * theta)
        x = r * np.cos(theta)
        y = r * np.sin(theta)
        return theta, x, y

    def _set_range(self, new_range: float):
        new_range = float(np.clip(new_range, self.min_range, self.max_range))
        if abs(new_range - self.xy_range) < 1e-6:
            return
        self.xy_range = new_range
        self.ax.set_xlim(-self.xy_range, self.xy_range)
        self.ax.set_ylim(-self.xy_range, self.xy_range)
        self.theta_path, self.x_path, self.y_path = self._compute_path_arrays(self.xy_range)
        self.path_line.set_data(self.x_path, self.y_path)
        self.ax_err.set_ylim(0.0, 3.0 * self.xy_range)
        self._update_settings_label()
        self.canvas.draw_idle()

    def _settings_text(self, filter_on: bool, n: int, tgt: int) -> str:
        return f"Range: ±{self.xy_range:.0f}°, Filter: {'ON' if filter_on else 'OFF'}, N={n}, Target: {tgt}s"

    def _update_settings_label(self):
        self.lbl_settings.setText(self._settings_text(self.chk_filter.isChecked(), int(self.s_win.value()), int(self.s_target.value())))

    # ------------- Slots -------------
    def _on_calibrate(self):
        self.pending_calibration = True
        print("\nWill calibrate to center on next sample...")

    def _on_toggle_run(self):
        # Start or stop the moving target and error recording
        if not self.is_running:
            # Starting a new measurement
            self.is_running = True
            period = max(1.0, float(self.s_target.value()))
            self.err_window_sec = period
            self.measure_start_t = time.monotonic()
            # Start target motion from beginning of the path
            self.t0 = self.measure_start_t
            self.theta_current = 0.0
            # Reset traces and logs
            self.err_times.clear(); self.err_values.clear()
            self.err_line.set_data([], [])
            self.meas_x.clear(); self.meas_y.clear()
            self.meas_line.set_data([], [])
            self.meas_line.set_visible(False)
            self.samples_t.clear(); self.samples_roll.clear(); self.samples_pitch.clear()
            self.samples_x.clear(); self.samples_y.clear(); self.samples_tx.clear(); self.samples_ty.clear()
            self.samples_err.clear(); self.samples_quat.clear()
            self.ax_err.set_xlim(0.0, self.err_window_sec)
            self.btn_toggle.setText("Stop")
            self.btn_save.setEnabled(False)
        else:
            # Stopping the measurement
            self.is_running = False
            self._finalize_measurement_path()
            self.btn_toggle.setText("Start")
            self.btn_save.setEnabled(True)

    def _on_filter_toggled(self, _checked: bool):
        self.buf_roll.clear()
        self.buf_pitch.clear()
        self._update_settings_label()

    def _on_filter_size_changed(self, val: int):
        self.buf_roll.clear(); self.buf_pitch.clear()
        self.buf_roll.maxlen = int(val)
        self.buf_pitch.maxlen = int(val)
        self.lbl_win.setText(f"N = {int(val)}")
        self._update_settings_label()

    def _on_target_changed(self, val: int):
        self.lbl_target.setText(f"Target T = {int(val)} s")
        self._update_settings_label()
        # When not running, let the error-axis reflect the selected duration
        if not self.is_running:
            self.err_window_sec = float(val)
            self.ax_err.set_xlim(0.0, self.err_window_sec)
            self.canvas.draw_idle()

    # ------------- Update loop -------------
    def _on_tick(self):
        # Update moving green target only when running
        period = max(1.0, float(self.s_target.value()))
        if self.is_running:
            now = time.monotonic()
            elapsed = (now - self.t0) % period
            self.theta_current = (elapsed / period) * (2.0 * np.pi)
        r_t = self.xy_range * np.cos(2.0 * self.theta_current)
        x_t = r_t * np.cos(self.theta_current)
        y_t = r_t * np.sin(self.theta_current)
        self.target_dot.set_data([x_t], [y_t])

        # Always read latest sensor data to allow practicing with the red dot
        data = self.sensor.get_collected_data() if self.sensor else None
        if data and len(data.get('quaternions', [])) > 0:
            q_cur = np.asarray(data['quaternions'][-1], dtype=np.float64)
            q_cur = _quat_normalize(q_cur)

            if self.calib_q is None or self.pending_calibration:
                self.calib_q = q_cur
                self.pending_calibration = False

            q_rel = _quat_mul(q_cur, _quat_conj(self.calib_q))
            q_rel = _quat_normalize(q_rel)

            roll_raw, pitch_raw, _yaw = _quat_to_euler_deg(q_rel)

            roll, pitch = roll_raw, pitch_raw

            if self.chk_filter.isChecked():
                self.buf_roll.append(roll)
                self.buf_pitch.append(pitch)
                roll_f = float(np.mean(self.buf_roll)) if len(self.buf_roll) > 0 else float(roll)
                pitch_f = float(np.mean(self.buf_pitch)) if len(self.buf_pitch) > 0 else float(pitch)
            else:
                roll_f, pitch_f = float(roll), float(pitch)

            x = float(np.clip(roll_f, -self.xy_range, self.xy_range))
            y = float(np.clip(pitch_f, -self.xy_range, self.xy_range))

            self.ball.set_data([x], [y])

            self.lbl_deg.setText(f"Raw Roll: {roll_f:+.1f}°, Raw Pitch: {pitch_f:+.1f}°")
            print(f"Roll:{roll_f:+.1f}°, Pitch:{pitch_f:+.1f}°", end='\r')

            # Error plot and logging: record only while running
            if self.is_running and self.measure_start_t is not None:
                now_t = time.monotonic()
                elapsed_run = now_t - self.measure_start_t
                # Append current red-dot position to measurement path and logs
                self.meas_x.append(x); self.meas_y.append(y)
                err = float(np.hypot(x - x_t, y - y_t))
                self.err_times.append(elapsed_run); self.err_values.append(err)
                self.err_line.set_data(list(self.err_times), list(self.err_values))
                self.ax_err.set_xlim(0.0, self.err_window_sec)
                # Logs
                self.samples_t.append(elapsed_run)
                self.samples_roll.append(roll_f)
                self.samples_pitch.append(pitch_f)
                self.samples_x.append(x)
                self.samples_y.append(y)
                self.samples_tx.append(float(x_t))
                self.samples_ty.append(float(y_t))
                self.samples_err.append(err)
                self.samples_quat.append([float(v) for v in q_rel])

                # Auto-stop when elapsed reaches the selected duration
                if elapsed_run >= self.err_window_sec:
                    self.is_running = False
                    self._finalize_measurement_path()
                    self.btn_toggle.setText("Start")
                    self.btn_save.setEnabled(True)

        self.canvas.draw_idle()

    def _finalize_measurement_path(self):
        """Show the recorded measurement path on the main plot."""
        if len(self.meas_x) > 1:
            self.meas_line.set_data(self.meas_x, self.meas_y)
            self.meas_line.set_visible(True)
            # Refresh legend in case it wasn't visible before
            self.ax.legend(loc='upper right', fontsize=9)
        self.canvas.draw_idle()

    def _on_save(self):
        """Save metadata and raw samples to a results folder."""
        if self.is_running:
            print("Stop measurement before saving.")
            return
        if not self.samples_t:
            print("No data to save.")
            return

        # Prepare output folder
        ts = time.strftime("%Y%m%d_%H%M%S")
        subj = (self.in_subject.text().strip() or "unknown").replace(" ", "_")
        base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "results"))
        out_dir = os.path.join(base_dir, f"{ts}_S{subj}")
        os.makedirs(out_dir, exist_ok=True)

        # Metadata
        device = getattr(self.sensor, "_device_name", None)
        address = getattr(self.sensor, "_device_address", None)
        tag = getattr(self.sensor, "_device_tag", None)
        cfg = {}
        if getattr(self, "sensor", None) is not None and getattr(self.sensor, "config", None) is not None:
            cfg = {
                "output_rate_hz": int(self.sensor.config.output_rate),
                "filter_profile": self.sensor.config.filter_profile.name,
                "payload_mode": self.sensor.config.payload_mode.name,
            }
        meta = {
            "timestamp": ts,
            "subject_id": subj,
            "experimenter": self.in_experimenter.text().strip(),
            "remarks": self.in_remarks.toPlainText().strip(),
            "duration_s": float(self.err_window_sec),
            "range_deg": float(self.xy_range),
            "filter_on": bool(self.chk_filter.isChecked()),
            "filter_N": int(self.s_win.value()),
            "device_name": device,
            "device_address": address,
            "device_tag": tag,
            "sensor_settings": cfg,
            "columns": [
                "t_s","roll_deg","pitch_deg","x_deg","y_deg","target_x_deg","target_y_deg","error_deg","q_w","q_x","q_y","q_z"
            ],
        }

        # Write metadata as JSON-like text header in CSV and separate meta.json
        samples_path = os.path.join(out_dir, "samples.csv")
        try:
            with open(samples_path, "w", encoding="utf-8") as f:
                f.write("# Spiral Tracking raw samples\n")
                for k, v in meta.items():
                    if k == "columns":
                        continue
                    f.write(f"# {k}: {v}\n")
                f.write(",".join(meta["columns"]) + "\n")
                for i in range(len(self.samples_t)):
                    row = [
                        self.samples_t[i], self.samples_roll[i], self.samples_pitch[i],
                        self.samples_x[i], self.samples_y[i], self.samples_tx[i], self.samples_ty[i],
                        self.samples_err[i],
                    ] + self.samples_quat[i]
                    f.write(",".join(f"{v:.6f}" for v in row) + "\n")
        except Exception as e:
            print(f"Failed to write samples.csv: {e}")

        # Separate metadata JSON
        try:
            import json
            with open(os.path.join(out_dir, "meta.json"), "w", encoding="utf-8") as jf:
                json.dump(meta, jf, indent=2)
        except Exception as e:
            print(f"Failed to write meta.json: {e}")

        print(f"Saved results to: {out_dir}")
        self.btn_save.setText("Saved")
        self.btn_save.setEnabled(False)

    # ------------- Cleanup -------------
    async def shutdown(self):
        try:
            if self.sensors:
                await asyncio.gather(*(s.stop_measurement() for s in self.sensors), return_exceptions=True)
                await asyncio.gather(*(s.disconnect() for s in self.sensors), return_exceptions=True)
        except Exception:
            pass

    def closeEvent(self, event):  # noqa: N802
        # Schedule async shutdown and then stop the event loop
        asyncio.create_task(self.shutdown())
        loop = asyncio.get_event_loop()
        try:
            loop.call_soon(loop.stop)
        except Exception:
            pass
        super().closeEvent(event)


# ---------------------- App bootstrap ----------------------
async def init_sensors() -> List[MovellaDOTSensor]:
    print("Scanning for Movella DOT sensors (5 seconds)...")
    devices = await BleakScanner.discover(timeout=5.0)
    print(f"Found {len(devices)} devices:")
    for device in devices:
        print(f"  {device.name} ({device.address})")
    dot_devices = [d for d in devices if d.name and ("Xsens DOT" in d.name or "Movella DOT" in d.name)]

    if not dot_devices:
        print("No Movella DOT sensors found")
        return []

    max_sensors = 1
    dot_devices = dot_devices[:max_sensors]
    print(f"Using {len(dot_devices)} Movella DOT sensor(s)")

    sensors: List[MovellaDOTSensor] = []

        


    config = SensorConfiguration(
        output_rate=OutputRate.RATE_30,
        filter_profile=FilterProfile.GENERAL,
        payload_mode=PayloadMode.ORIENTATION_QUATERNION,
    )

    for device in dot_devices:
        try:
            sensor = MovellaDOTSensor(config)
            sensor.client = BleakClient(device.address)
            print(f"\nConnecting to {device.name} ({device.address})...")
            await sensor.client.connect()
            sensor.is_connected = True
            sensor._device_address = device.address
            sensor._device_name = device.name

            print("\nReading device information...")
            device_info = await sensor.get_device_info()
            sensor._device_tag = device_info.device_tag
            for key, value in device_info.__dict__.items():
                print(f"{key}: {value}")
            print(f"Current Output Rate: {device_info.output_rate} Hz")
            print(f"Current Filter Profile: {device_info.filter_profile.name}")

            print("\nIdentifying sensor...")
            await sensor.identify_sensor()
            await asyncio.sleep(2)

            await sensor.configure_sensor()
            sensors.append(sensor)
            print(f"Successfully connected and configured {device.name}")
        except Exception as e:
            print(f"Failed to connect to {device.name}: {str(e)}")

    if sensors:
        print("\nStarting measurements on all sensors...")
        await asyncio.gather(*(s.start_measurement() for s in sensors))
    return sensors


async def amain():
    sensors = await init_sensors()
    if not sensors:
        print("No sensors available. Exiting.")
        return
    win = TiltBallWindow(sensors)
    win.show()


def main():
    app = QApplication(sys.argv)
    loop = QEventLoop(app)
    asyncio.set_event_loop(loop)
    with loop:
        loop.run_until_complete(amain())
        loop.run_forever()


if __name__ == "__main__":
    main()
