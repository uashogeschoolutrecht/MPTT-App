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
        self.setWindowTitle("MPTT-App")
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
        # Blind flag per-sample
        self.samples_blind: List[int] = []  # 1 if within blind window else 0

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

        # Blind segment configuration (UI-controlled)
        self.blind_enabled = False
        self.blind_duration_sec = 5.0
        self.blind_start_sec = 5.0
        # Repeating blind configuration
        self.blind_repeat_enabled = False
        self.blind_repeat_every_sec = 10.0
        # Snapshot used during an active run
        self.active_blind_start: Optional[float] = None
        self.active_blind_end: Optional[float] = None
        self.active_blind_windows: List[tuple] = []  # list of (start,end)

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

        # Blind segment controls
        controls.addSpacing(6)
        self.chk_blind = QCheckBox("Enable blind segment", self)
        self.chk_blind.setChecked(False)
        controls.addWidget(self.chk_blind)
        self.lbl_blind_dur = QLabel("Blind duration = 5 s", self)
        self.s_blind_dur = QSlider(Qt.Horizontal, self)
        self.s_blind_dur.setRange(1, 60)
        self.s_blind_dur.setValue(5)
        self.s_blind_dur.setSingleStep(1)
        controls.addWidget(self.lbl_blind_dur)
        controls.addWidget(self.s_blind_dur)
        self.lbl_blind_start = QLabel("Blind start @ 5 s", self)
        self.s_blind_start = QSlider(Qt.Horizontal, self)
        self.s_blind_start.setRange(0, self.s_target.value())
        self.s_blind_start.setValue(5)
        self.s_blind_start.setSingleStep(1)
        controls.addWidget(self.lbl_blind_start)
        controls.addWidget(self.s_blind_start)
        # Repeat checkbox + interval slider
        self.chk_blind_repeat = QCheckBox("Repeat blind every", self)
        self.chk_blind_repeat.setChecked(False)
        controls.addWidget(self.chk_blind_repeat)
        self.lbl_blind_every = QLabel("Repeat every = 10 s", self)
        self.s_blind_every = QSlider(Qt.Horizontal, self)
        self.s_blind_every.setRange(2, 120)
        self.s_blind_every.setValue(10)
        self.s_blind_every.setSingleStep(1)
        controls.addWidget(self.lbl_blind_every)
        controls.addWidget(self.s_blind_every)
        # Disable sliders until enabled
        self.s_blind_dur.setEnabled(False)
        self.s_blind_start.setEnabled(False)
        self.chk_blind_repeat.setEnabled(False)
        self.s_blind_every.setEnabled(False)

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
        # Blind signals
        self.chk_blind.toggled.connect(self._on_blind_toggled)
        self.s_blind_dur.valueChanged.connect(self._on_blind_dur_changed)
        self.s_blind_start.valueChanged.connect(self._on_blind_start_changed)
        # Repeat signals
        self.chk_blind_repeat.toggled.connect(self._on_blind_repeat_toggled)
        self.s_blind_every.valueChanged.connect(self._on_blind_every_changed)

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
        # Blind segment path overlay
        (self.meas_blind_line,) = self.ax.plot([], [], '-', color='royalblue', lw=2.0, alpha=0.9, label='Blind segment')
        self.meas_blind_line.set_visible(False)
        self.ax.legend(loc='upper right', fontsize=9)

        self.canvas.draw_idle()

    def _setup_error_axes(self):
        self.ax_err.clear()
        self.ax_err.set_title('Abs Error (deg)', fontsize=9)
        self.ax_err.set_xlim(0.0, self.err_window_sec)
        self.ax_err.set_ylim(0.0, 3.0 * self.xy_range)
        self.ax_err.grid(True, linestyle=':', alpha=0.3)
        (self.err_line,) = self.ax_err.plot([], [], color='tab:red', lw=1.0)
        # Placeholder for blind shading
        self.err_blind_span = None
        self.canvas.draw_idle()

    # ------------- Helpers -------------
    def _compute_path_arrays(self, range_val: float):
        # 8-lobed rose curve with offset: r(θ) = R0 + A*cos(8θ)
        # Choose R0 and A so that r_min > 0 (avoid center) and r_max < range (stay on screen)
        theta = np.linspace(0.0, 2.0 * np.pi, self.num_path_pts)
        R0 = 0.55 * float(range_val)
        A = 0.30 * float(range_val)
        r = R0 + A * np.cos(8.0 * theta)
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
        # Be robust during __init__: blind controls may not be created yet
        blind_txt = "OFF"
        try:
            chk = getattr(self, "chk_blind", None)
            if chk is not None and chk.isChecked():
                # Prefer slider values if available, otherwise fall back to defaults from state
                s_dur = getattr(self, "s_blind_dur", None)
                s_start = getattr(self, "s_blind_start", None)
                s_every = getattr(self, "s_blind_every", None)
                chk_rep = getattr(self, "chk_blind_repeat", None)
                try:
                    bdur = int(s_dur.value()) if s_dur is not None else int(self.blind_duration_sec)
                except Exception:
                    bdur = int(self.blind_duration_sec)
                try:
                    bstart = int(s_start.value()) if s_start is not None else int(self.blind_start_sec)
                except Exception:
                    bstart = int(self.blind_start_sec)
                if chk_rep is not None and chk_rep.isChecked():
                    try:
                        bevery = int(s_every.value()) if s_every is not None else int(self.blind_repeat_every_sec)
                    except Exception:
                        bevery = int(self.blind_repeat_every_sec)
                    blind_txt = f"{bdur}s every {bevery}s starting {bstart}s"
                else:
                    blind_txt = f"{bdur}s @ {bstart}s"
        except Exception:
            # If anything goes wrong during early init, keep Blind: OFF
            pass
        return f"Range: ±{self.xy_range:.0f}°, Filter: {'ON' if filter_on else 'OFF'}, N={n}, Target: {tgt}s, Blind: {blind_txt}"

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
            # Remove any old blind shading
            if self.err_blind_span is not None:
                try:
                    self.err_blind_span.remove()
                except Exception:
                    pass
                self.err_blind_span = None
            # Remove any prior multiple spans
            if hasattr(self, 'err_blind_spans'):
                try:
                    for sp in self.err_blind_spans:
                        sp.remove()
                except Exception:
                    pass
                self.err_blind_spans = []
            self.meas_x.clear(); self.meas_y.clear()
            self.meas_line.set_data([], [])
            self.meas_line.set_visible(False)
            self.meas_blind_line.set_data([], [])
            self.meas_blind_line.set_visible(False)
            self.samples_t.clear(); self.samples_roll.clear(); self.samples_pitch.clear()
            self.samples_x.clear(); self.samples_y.clear(); self.samples_tx.clear(); self.samples_ty.clear()
            self.samples_err.clear(); self.samples_quat.clear(); self.samples_blind.clear()
            self.ax_err.set_xlim(0.0, self.err_window_sec)
            # Snapshot blind windows for this run
            self.active_blind_start = None
            self.active_blind_end = None
            self.active_blind_windows = []
            if self.chk_blind.isChecked():
                bstart = float(self.s_blind_start.value())
                bdur = float(self.s_blind_dur.value())
                if self.chk_blind_repeat.isChecked():
                    bevery = max(1.0, float(self.s_blind_every.value()))
                    t0 = max(0.0, bstart)
                    while t0 < period:
                        s = t0
                        e = min(t0 + bdur, period)
                        if e > s:
                            self.active_blind_windows.append((s, e))
                        t0 += bevery
                    # For compatibility fields
                    if self.active_blind_windows:
                        self.active_blind_start = self.active_blind_windows[0][0]
                        self.active_blind_end = self.active_blind_windows[0][1]
                else:
                    bstart = max(0.0, min(bstart, period))
                    bend = max(bstart, min(bstart + bdur, period))
                    self.active_blind_start = bstart
                    self.active_blind_end = bend
                    self.active_blind_windows = [(bstart, bend)]
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
        # Update blind start slider max to new duration
        self.s_blind_start.setRange(0, int(val))
        # Clamp current start and duration to fit in new window
        bstart = min(self.s_blind_start.value(), int(val))
        self.s_blind_start.setValue(bstart)
        bdur = min(self.s_blind_dur.value(), int(val))
        self.s_blind_dur.setValue(bdur)
        self._on_blind_start_changed(self.s_blind_start.value())
        self._on_blind_dur_changed(self.s_blind_dur.value())
        self._update_settings_label()
        # When not running, let the error-axis reflect the selected duration
        if not self.is_running:
            self.err_window_sec = float(val)
            self.ax_err.set_xlim(0.0, self.err_window_sec)
            self.canvas.draw_idle()

    def _on_blind_toggled(self, checked: bool):
        self.blind_enabled = bool(checked)
        self.s_blind_dur.setEnabled(checked)
        self.s_blind_start.setEnabled(checked)
        self.chk_blind_repeat.setEnabled(checked)
        rep_enabled = checked and self.chk_blind_repeat.isChecked()
        self.s_blind_every.setEnabled(rep_enabled)
        self._update_settings_label()

    def _on_blind_dur_changed(self, val: int):
        self.blind_duration_sec = float(val)
        self.lbl_blind_dur.setText(f"Blind duration = {int(val)} s")
        # Ensure start + duration <= target unless repeating (we'll clip per-window later)
        max_t = int(self.s_target.value())
        if (not self.chk_blind_repeat.isChecked()) and (self.s_blind_start.value() + val > max_t):
            self.s_blind_start.setValue(max(0, max_t - val))
        self._update_settings_label()

    def _on_blind_start_changed(self, val: int):
        self.blind_start_sec = float(val)
        self.lbl_blind_start.setText(f"Blind start @ {int(val)} s")
        # For single-window mode keep it in range
        max_t = int(self.s_target.value())
        if (not self.chk_blind_repeat.isChecked()) and (val + self.s_blind_dur.value() > max_t):
            self.s_blind_dur.setValue(max(1, max_t - val))
        self._update_settings_label()

    def _on_blind_repeat_toggled(self, checked: bool):
        self.blind_repeat_enabled = bool(checked)
        self.s_blind_every.setEnabled(self.blind_enabled and checked)
        self._update_settings_label()

    def _on_blind_every_changed(self, val: int):
        self.blind_repeat_every_sec = float(val)
        self.lbl_blind_every.setText(f"Repeat every = {int(val)} s")
        self._update_settings_label()

    # ------------- Update loop -------------
    def _on_tick(self):
        # Update moving green target only when running
        period = max(1.0, float(self.s_target.value()))
        if self.is_running:
            now = time.monotonic()
            elapsed = (now - self.t0) % period
            self.theta_current = (elapsed / period) * (2.0 * np.pi)
        # 8-lobed rose curve with offset: r(θ) = R0 + A*cos(8θ)
        # Choose R0 and A so that r_min > 0 (avoid center) and r_max < range (stay on screen)
        R0 = 0.55 * float(self.xy_range)
        A = 0.30 * float(self.xy_range)
        r_t = R0 + A * np.cos(8.0 * self.theta_current)
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

            # Determine blind visibility for the red ball
            blind_active_now = False
            if self.is_running and self.measure_start_t is not None:
                now_t = time.monotonic()
                elapsed_run_tmp = now_t - self.measure_start_t
                if self.active_blind_windows:
                    for s, e in self.active_blind_windows:
                        if s <= elapsed_run_tmp < e:
                            blind_active_now = True
                            break
                elif (self.active_blind_start is not None and self.active_blind_end is not None):
                    if self.active_blind_start <= elapsed_run_tmp < self.active_blind_end:
                        blind_active_now = True

            # Hide or show the red ball
            self.ball.set_visible(not blind_active_now)

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
                self.samples_blind.append(1 if blind_active_now else 0)

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
            # Blind overlay (subset of measured path during blind)
            if any(self.samples_blind):
                xb = []
                yb = []
                for i in range(len(self.meas_x)):
                    if self.samples_blind[i]:
                        xb.append(self.meas_x[i])
                        yb.append(self.meas_y[i])
                        # Insert a break (NaN) at the end of each blind segment
                        if i + 1 >= len(self.meas_x) or not self.samples_blind[i + 1]:
                            xb.append(np.nan)
                            yb.append(np.nan)
                if len(xb) > 1:
                    self.meas_blind_line.set_data(xb, yb)
                    self.meas_blind_line.set_visible(True)
            # Refresh legend in case it wasn't visible before
            self.ax.legend(loc='upper right', fontsize=9)
        # Add shaded region(s) on error plot for blind windows
        # Remove previous spans
        if hasattr(self, 'err_blind_spans'):
            try:
                for sp in self.err_blind_spans:
                    sp.remove()
            except Exception:
                pass
        self.err_blind_spans = []
        spans = self.active_blind_windows if self.active_blind_windows else (
            [(self.active_blind_start, self.active_blind_end)] if (self.active_blind_start is not None and self.active_blind_end is not None) else []
        )
        for s, e in spans:
            try:
                sp = self.ax_err.axvspan(s, e, color='royalblue', alpha=0.15, label='Blind')
                self.err_blind_spans.append(sp)
            except Exception:
                pass
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

        # Compute stats from collected samples
        try:
            t = np.asarray(self.samples_t, dtype=np.float64)
            err = np.asarray(self.samples_err, dtype=np.float64)
            x = np.asarray(self.samples_x, dtype=np.float64)
            y = np.asarray(self.samples_y, dtype=np.float64)
            tx = np.asarray(self.samples_tx, dtype=np.float64)
            ty = np.asarray(self.samples_ty, dtype=np.float64)
            blind = np.asarray(self.samples_blind, dtype=np.int32) if len(self.samples_blind) == len(t) else np.zeros_like(t, dtype=np.int32)
            duration = float(t[-1]) if len(t) > 0 else 0.0
            auc_err = float(np.trapz(err, t)) if len(t) > 1 else 0.0  # deg*s
            mean_err = float(np.mean(err)) if err.size else 0.0
            median_err = float(np.median(err)) if err.size else 0.0
            std_err = float(np.std(err, ddof=0)) if err.size else 0.0
            rmse_err = float(np.sqrt(np.mean(err ** 2))) if err.size else 0.0
            max_err = float(np.max(err)) if err.size else 0.0
            p95_err = float(np.percentile(err, 95)) if err.size else 0.0
            # Per-axis absolute error
            ex = (x - tx); ey = (y - ty)
            mae_x = float(np.mean(np.abs(ex))) if ex.size else 0.0
            rmse_x = float(np.sqrt(np.mean(ex ** 2))) if ex.size else 0.0
            mae_y = float(np.mean(np.abs(ey))) if ey.size else 0.0
            rmse_y = float(np.sqrt(np.mean(ey ** 2))) if ey.size else 0.0
            # Path lengths and average speeds
            def _path_len(xx, yy):
                if len(xx) < 2:
                    return 0.0
                dx = np.diff(xx); dy = np.diff(yy)
                return float(np.sum(np.hypot(dx, dy)))
            path_len_meas = _path_len(x, y)
            path_len_target = _path_len(tx, ty)
            avg_speed_meas = float(path_len_meas / duration) if duration > 0 else 0.0
            avg_speed_target = float(path_len_target / duration) if duration > 0 else 0.0

            # Blind-only stats
            has_blind = int(np.any(blind == 1))
            if has_blind:
                mask = blind == 1
                tb = t[mask]
                errb = err[mask]
                xb = x[mask]; yb = y[mask]
                txb = tx[mask]; tyb = ty[mask]
                exb = xb - txb; eyb = yb - tyb
                duration_b = float(tb[-1] - tb[0]) if len(tb) > 0 else 0.0
                # If multiple windows, compute total union length from active_blind_windows
                try:
                    spans = self.active_blind_windows if self.active_blind_windows else (
                        [(self.active_blind_start, self.active_blind_end)] if (self.active_blind_start is not None and self.active_blind_end is not None) else []
                    )
                    duration_b = float(sum(max(0.0, e - s) for s, e in spans)) if spans else duration_b
                except Exception:
                    pass
                auc_err_b = float(np.trapz(errb, tb)) if len(tb) > 1 else 0.0
                mean_err_b = float(np.mean(errb)) if errb.size else 0.0
                median_err_b = float(np.median(errb)) if errb.size else 0.0
                std_err_b = float(np.std(errb, ddof=0)) if errb.size else 0.0
                rmse_err_b = float(np.sqrt(np.mean(errb ** 2))) if errb.size else 0.0
                max_err_b = float(np.max(errb)) if errb.size else 0.0
                p95_err_b = float(np.percentile(errb, 95)) if errb.size else 0.0
                mae_x_b = float(np.mean(np.abs(exb))) if exb.size else 0.0
                rmse_x_b = float(np.sqrt(np.mean(exb ** 2))) if exb.size else 0.0
                mae_y_b = float(np.mean(np.abs(eyb))) if eyb.size else 0.0
                rmse_y_b = float(np.sqrt(np.mean(eyb ** 2))) if eyb.size else 0.0
                path_len_meas_b = _path_len(xb, yb)
                path_len_target_b = _path_len(txb, tyb)
                avg_speed_meas_b = float(path_len_meas_b / duration_b) if duration_b > 0 else 0.0
                avg_speed_target_b = float(path_len_target_b / duration_b) if duration_b > 0 else 0.0
            else:
                duration_b = auc_err_b = mean_err_b = median_err_b = std_err_b = rmse_err_b = max_err_b = p95_err_b = 0.0
                mae_x_b = rmse_x_b = mae_y_b = rmse_y_b = path_len_meas_b = path_len_target_b = avg_speed_meas_b = avg_speed_target_b = 0.0

            stats_dict = {
                "blind_windows_count": int(len(self.active_blind_windows)) if self.active_blind_windows else (1 if (self.active_blind_start is not None and self.active_blind_end is not None) else 0),
                "duration_s": duration,
                "samples": int(len(t)),
                "mean_error_deg": mean_err,
                "median_error_deg": median_err,
                "std_error_deg": std_err,
                "rmse_error_deg": rmse_err,
                "max_error_deg": max_err,
                "p95_error_deg": p95_err,
                "auc_error_deg_s": auc_err,
                "mae_x_deg": mae_x,
                "rmse_x_deg": rmse_x,
                "mae_y_deg": mae_y,
                "rmse_y_deg": rmse_y,
                "path_length_meas_deg": path_len_meas,
                "path_length_target_deg": path_len_target,
                "avg_speed_meas_deg_s": avg_speed_meas,
                "avg_speed_target_deg_s": avg_speed_target,
                # Blind extras
                "has_blind": int(has_blind),
                "blind_duration_s": float(sum(max(0.0, e - s) for s, e in self.active_blind_windows)) if self.active_blind_windows else (float(self.active_blind_end - self.active_blind_start) if (self.active_blind_start is not None and self.active_blind_end is not None) else duration_b),
                "blind_start_s": float(self.active_blind_start) if self.active_blind_start is not None else 0.0,
                "blind_mean_error_deg": mean_err_b,
                "blind_median_error_deg": median_err_b,
                "blind_std_error_deg": std_err_b,
                "blind_rmse_error_deg": rmse_err_b,
                "blind_max_error_deg": max_err_b,
                "blind_p95_error_deg": p95_err_b,
                "blind_auc_error_deg_s": auc_err_b,
                "blind_mae_x_deg": mae_x_b,
                "blind_rmse_x_deg": rmse_x_b,
                "blind_mae_y_deg": mae_y_b,
                "blind_rmse_y_deg": rmse_y_b,
                "blind_path_length_meas_deg": path_len_meas_b,
                "blind_path_length_target_deg": path_len_target_b,
                "blind_avg_speed_meas_deg_s": avg_speed_meas_b,
                "blind_avg_speed_target_deg_s": avg_speed_target_b,
            }
        except Exception as e:
            print(f"Failed to compute stats: {e}")
            stats_dict = {}

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
            "blind_enabled": bool(self.chk_blind.isChecked()),
            "blind_repeat_enabled": bool(self.chk_blind_repeat.isChecked()) if hasattr(self, 'chk_blind_repeat') else False,
            "blind_start_s": float(self.active_blind_start) if self.active_blind_start is not None else float(self.blind_start_sec if self.chk_blind.isChecked() else 0.0),
            "blind_duration_s": float((self.active_blind_end - self.active_blind_start) if (self.active_blind_start is not None and self.active_blind_end is not None) else (self.blind_duration_sec if self.chk_blind.isChecked() else 0.0)),
            "blind_every_s": float(self.blind_repeat_every_sec) if getattr(self, 'blind_repeat_enabled', False) else 0.0,
            "blind_windows": list(self.active_blind_windows) if self.active_blind_windows else ([] if self.active_blind_start is None else [(self.active_blind_start, self.active_blind_end)]),
            "stats": stats_dict,
            "columns": [
                "t_s","roll_deg","pitch_deg","x_deg","y_deg","target_x_deg","target_y_deg","error_deg","blind","q_w","q_x","q_y","q_z"
            ],
        }

        # Write metadata as JSON-like text header in CSV and separate meta.json
        samples_path = os.path.join(out_dir, "samples.csv")
        try:
            with open(samples_path, "w", encoding="utf-8") as f:
                f.write("# MPTT-App raw samples\n")
                for k, v in meta.items():
                    if k == "columns":
                        continue
                    f.write(f"# {k}: {v}\n")
                f.write(",".join(meta["columns"]) + "\n")
                for i in range(len(self.samples_t)):
                    row = [
                        self.samples_t[i], self.samples_roll[i], self.samples_pitch[i],
                        self.samples_x[i], self.samples_y[i], self.samples_tx[i], self.samples_ty[i],
                        self.samples_err[i], float(self.samples_blind[i] if i < len(self.samples_blind) else 0),
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

        # Stats CSV (overall)
        try:
            stats_csv = os.path.join(out_dir, "stats.csv")
            with open(stats_csv, "w", encoding="utf-8") as sf:
                sf.write("metric,value\n")
                for k, v in stats_dict.items():
                    sf.write(f"{k},{v}\n")
        except Exception as e:
            print(f"Failed to write stats.csv: {e}")

        # Blind-only samples
        try:
            if any(self.samples_blind):
                blind_samples_path = os.path.join(out_dir, "blind_samples.csv")
                with open(blind_samples_path, "w", encoding="utf-8") as bf:
                    bf.write(",".join(meta["columns"]) + "\n")
                    for i in range(len(self.samples_t)):
                        if i < len(self.samples_blind) and self.samples_blind[i]:
                            row = [
                                self.samples_t[i], self.samples_roll[i], self.samples_pitch[i],
                                self.samples_x[i], self.samples_y[i], self.samples_tx[i], self.samples_ty[i],
                                self.samples_err[i], float(self.samples_blind[i]),
                            ] + self.samples_quat[i]
                            bf.write(",".join(f"{v:.6f}" for v in row) + "\n")
        except Exception as e:
            print(f"Failed to write blind_samples.csv: {e}")

        # Blind-only stats CSV
        try:
            if stats_dict.get("has_blind", 0):
                blind_stats_csv = os.path.join(out_dir, "blind_stats.csv")
                with open(blind_stats_csv, "w", encoding="utf-8") as bsf:
                    bsf.write("metric,value\n")
                    for k, v in stats_dict.items():
                        if k.startswith("blind_") or k in ("has_blind",):
                            bsf.write(f"{k},{v}\n")
        except Exception as e:
            print(f"Failed to write blind_stats.csv: {e}")

        # Save plots: full figure (both plots) and separate error plot
        try:
            # Ensure the measurement path is visible
            self._finalize_measurement_path()
            self.canvas.draw()
            self.fig.savefig(os.path.join(out_dir, "plots.png"), dpi=150, bbox_inches='tight', facecolor='white')
        except Exception as e:
            print(f"Failed to save plots.png: {e}")
        try:
            # Save error plot alone using a lightweight Figure
            fig_err = Figure(figsize=(7, 3), dpi=150)
            ax_e = fig_err.add_subplot(111)
            ax_e.set_title('Abs Error (deg)')
            ax_e.set_xlabel('Time (s)')
            ax_e.set_ylabel('Error (deg)')
            ax_e.grid(True, linestyle=':', alpha=0.3)
            if len(self.err_times) > 0:
                ax_e.plot(list(self.err_times), list(self.err_values), color='tab:red', lw=1.0)
                ax_e.set_xlim(0.0, float(self.err_window_sec))
                # Shade blind window if present
                spans = self.active_blind_windows if self.active_blind_windows else (
                    [(self.active_blind_start, self.active_blind_end)] if (self.active_blind_start is not None and self.active_blind_end is not None) else []
                )
                for s, e in spans:
                    ax_e.axvspan(s, e, color='royalblue', alpha=0.15)
            fig_err.savefig(os.path.join(out_dir, "error_plot.png"), dpi=150, bbox_inches='tight', facecolor='white')
        except Exception as e:
            print(f"Failed to save error_plot.png: {e}")

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
