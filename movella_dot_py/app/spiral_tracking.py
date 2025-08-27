import asyncio
from bleak import BleakScanner, BleakClient
import sys
import os
import time
import numpy as np
import matplotlib.pyplot as plt
import contextlib
from matplotlib.widgets import Button, Slider, CheckButtons, TextBox
from collections import deque
import csv
from datetime import datetime
from pathlib import Path


# Add the parent directory to the Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from movella_dot_py.core.sensor import MovellaDOTSensor
from movella_dot_py.models.data_structures import SensorConfiguration
from movella_dot_py.models.enums import OutputRate, FilterProfile, PayloadMode

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
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ], dtype=np.float64)

def _quat_to_euler_deg(q: np.ndarray) -> np.ndarray:
    """Convert unit quaternion [w,x,y,z] to ZYX euler angles [roll, pitch, yaw] in degrees.
    roll: x (left/right), pitch: y (forward/backward), yaw: z (twist)
    """
    w, x, y, z = q
    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (w*x + y*z)
    cosr_cosp = 1.0 - 2.0 * (x*x + y*y)
    roll = np.degrees(np.arctan2(sinr_cosp, cosr_cosp))

    # pitch (y-axis rotation)
    sinp = 2.0 * (w*y - z*x)
    sinp = np.clip(sinp, -1.0, 1.0)
    pitch = np.degrees(np.arcsin(sinp))

    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (w*z + x*y)
    cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
    yaw = np.degrees(np.arctan2(siny_cosp, cosy_cosp))

    return np.array([roll, pitch, yaw], dtype=np.float64)

class DataLogger:
    """Handles CSV logging of sensor data, target trajectory, and error calculations"""
    
    def __init__(self, patient_id: str = "9999"):
        self.patient_id = patient_id
        self.session_start_time = datetime.now()
        self.data_buffer = []
        self.csv_file = None
        self.csv_writer = None
        self.session_start_monotonic = time.monotonic()
        
        # Create data directory structure
        self.data_dir = Path("data")
        self.patient_dir = self.data_dir / self.patient_id
        self.patient_dir.mkdir(parents=True, exist_ok=True)
        
        # Create unique CSV filename with timestamp
        timestamp_str = self.session_start_time.strftime("%Y%m%d_%H%M%S")
        self.csv_filename = f"{self.patient_id}_{timestamp_str}.csv"
        self.csv_filepath = self.patient_dir / self.csv_filename
        
        print(f"Data logger initialized for patient {patient_id}")
        print(f"Will save to: {self.csv_filepath}")
    
    def start_session(self, sensor_info=None):
        """Start a new logging session"""
        try:
            self.csv_file = open(self.csv_filepath, 'w', newline='', encoding='utf-8')
            self.csv_writer = csv.writer(self.csv_file)
            
            # Write CSV header
            header = [
                 'system_timestamp',
                 'quat_w', 'quat_x', 'quat_y', 'quat_z',
                 'euler_roll_raw', 'euler_pitch_raw', 'euler_yaw_raw',
                 'calibrated_roll_deg', 'calibrated_pitch_deg',
                 'filtered_roll_deg', 'filtered_pitch_deg',
                 'target_x_deg', 'target_y_deg', 'target_phase',
                 'user_x_deg', 'user_y_deg',
                 'error_distance_deg', 'error_x_deg', 'error_y_deg',
                 'range_setting', 'filter_enabled', 'target_speed_setting'
             ]
            self.csv_writer.writerow(header)
            
            # Write session metadata as comments
            self.csv_file.write(f"# Session started: {self.session_start_time.isoformat()}\n")
            self.csv_file.write(f"# Patient ID: {self.patient_id}\n")
            if sensor_info:
                self.csv_file.write(f"# Device: {sensor_info.get('name', 'Unknown')}\n")
                self.csv_file.write(f"# Address: {sensor_info.get('address', 'Unknown')}\n")
            self.csv_file.write(f"# \n")
            
            self.csv_file.flush()
            print(f"Started logging session to {self.csv_filepath}")
            
        except Exception as e:
            print(f"Error starting data logger: {e}")
    
    def log_sample(self, timestamp, quaternion, euler_raw, calibrated_angles, filtered_angles, 
                   target_pos, user_pos, error_data, config):
        """Log a single data sample"""
        if not self.csv_writer:
            return
            
        try:
            # Use current datetime instead of converting from timestamp
            system_timestamp = datetime.now().isoformat()
            
            # Unpack data
            x_t, y_t, theta_current = target_pos
            x, y = user_pos
            err, err_x, err_y = error_data
            roll_raw, pitch_raw, yaw_raw = euler_raw
            roll_cal, pitch_cal = calibrated_angles
            roll_filt, pitch_filt = filtered_angles
            
            row = [
                system_timestamp,
                quaternion[0], quaternion[1], quaternion[2], quaternion[3],
                roll_raw, pitch_raw, yaw_raw,
                roll_cal, pitch_cal,
                roll_filt, pitch_filt,
                x_t, y_t, theta_current,
                x, y,
                err, err_x, err_y,
                config['range'], config['filter_enabled'], config['target_speed']
            ]
            
            self.csv_writer.writerow(row)
            
            # Flush periodically to ensure data is saved
            if len(row) % 100 == 0:  # Every 100 samples
                self.csv_file.flush()
                
        except Exception as e:
            print(f"Error logging sample: {e}")
    
    def end_session(self):
        """End the logging session and close files"""
        if self.csv_file:
            try:
                # Write session summary
                session_duration = time.monotonic() - self.session_start_monotonic
                self.csv_file.write(f"# Session ended: {datetime.now().isoformat()}\n")
                self.csv_file.write(f"# Duration: {session_duration:.1f} seconds\n")
                
                self.csv_file.close()
                print(f"Session ended. Data saved to {self.csv_filepath}")
                print(f"Duration: {session_duration:.1f} seconds")
            except Exception as e:
                print(f"Error closing data logger: {e}")

async def live_plot_ball_from_quat(sensor: MovellaDOTSensor, stop_event: asyncio.Event, xy_range: float = 30.0):
    """Live 2D ball showing tilt from quaternion stream with calibration.
    X = Forward/Backward (pitch, deg), Y = Left/Right (roll, deg).
    Controls: Calibrate, Exit, Range -, Range +, Filter toggle and size. Shows current L/R, F/B and settings.
    Keyboard: c/space = calibrate to center (next sample), q/esc = exit.
    """
    plt.ion()
    fig, ax = plt.subplots(figsize=(7, 7))
    # Removed title and axis labels
    ax.set_xlim(-xy_range, xy_range)
    ax.set_ylim(-xy_range, xy_range)
    # Y flip handled in data mapping
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True, linestyle=':', alpha=0.5)
    ax.axhline(0, color='k', lw=0.5)
    ax.axvline(0, color='k', lw=0.5)

    ball, = ax.plot([0], [0], 'o', color='red', markersize=18)

    # Control window with buttons, live degree readout, and filter controls
    fig_ctrl = plt.figure(figsize=(10, 3.0))
    fig_ctrl.canvas.manager.set_window_title('Controls') if hasattr(fig_ctrl.canvas.manager, 'set_window_title') else None

    # Patient ID input field at the top
    ax_patient_id = fig_ctrl.add_axes([0.02, 0.85, 0.15, 0.10])
    patient_id_box = TextBox(ax_patient_id, 'Patient ID: ', initial="9999")
    
    # Live degree text at the top of the control window
    text_deg = fig_ctrl.text(0.5, 0.9, 'L/R: +0.0°, F/B: +0.0°', ha='center', va='center', fontsize=12)
    # Settings text at bottom
    text_settings = fig_ctrl.text(0.5, 0.05, 'Range: ±30°, Filter: OFF, N=5, Target: 60s', ha='center', va='center', fontsize=10)
    # Recording status text
    text_recording = fig_ctrl.text(0.85, 0.9, 'Recording: OFF', ha='center', va='center', fontsize=10, color='red')

    # Buttons: Calibrate, Exit, Range -, Range +
    ax_cal   = fig_ctrl.add_axes([0.03, 0.25, 0.14, 0.5])
    ax_exit  = fig_ctrl.add_axes([0.19, 0.25, 0.14, 0.5])
    ax_rminus = fig_ctrl.add_axes([0.35, 0.25, 0.14, 0.5])
    ax_rplus  = fig_ctrl.add_axes([0.51, 0.25, 0.14, 0.5])
    # Start/Stop toggle (compact) — placed before filter controls
    ax_run   = fig_ctrl.add_axes([0.60, 0.25, 0.07, 0.5])

    btn_cal   = Button(ax_cal, 'Calibrate')
    btn_exit  = Button(ax_exit, 'Exit')
    btn_rminus = Button(ax_rminus, 'Range -')
    btn_rplus  = Button(ax_rplus, 'Range +')
    btn_run    = Button(ax_run, 'Start')  # initial state: paused

    # Filter controls: toggle and window slider
    ax_filter_toggle = fig_ctrl.add_axes([0.68, 0.25, 0.10, 0.5])
    chk = CheckButtons(ax_filter_toggle, ['Filter'], [False])

    ax_filter_size = fig_ctrl.add_axes([0.80, 0.35, 0.18, 0.2])
    s_win = Slider(ax_filter_size, 'N', 3, 150, valinit=5, valstep=1)

    # Target duration slider (seconds)
    ax_target_time = fig_ctrl.add_axes([0.80, 0.10, 0.18, 0.2])
    s_target = Slider(ax_target_time, 'Target T (s)', 30, 120, valinit=60, valstep=1)

    calib_q = None
    pending_calibration = False  # apply calibration on next fresh sample
    current_range = float(xy_range)
    step = 5.0
    min_range = 5.0
    max_range = 180.0
    
    # Data logging variables
    data_logger = None
    logging_active = False

    # Error plot (absolute error in degrees) — small axes in control window
    err_window_sec = 30.0
    ax_err = fig_ctrl.add_axes([0.68, 0.70, 0.30, 0.25])
    ax_err.set_title('Abs Error (deg)', fontsize=9)
    ax_err.set_xlim(0.0, err_window_sec)
    ax_err.set_ylim(0.0, 3.0 * current_range)
    ax_err.grid(True, linestyle=':', alpha=0.3)
    ax_err.tick_params(labelsize=8)
    err_line, = ax_err.plot([], [], color='tab:red', lw=1.0)
    err_times: deque = deque()
    err_values: deque = deque()

    # Moving average buffers
    buf_roll = deque(maxlen=int(s_win.val))
    buf_pitch = deque(maxlen=int(s_win.val))

    # Animation state
    is_running = False  # Start paused so user can enter patient ID first
    theta_current = 0.0  # current parameter along the path [0, 2π)

    # Reference path (r = 2cos(2θ)) scaled to current range, and moving green target dot
    num_path_pts = 1000

    def compute_path_arrays(range_val: float):
        # Use full [0, 2π] for a closed four-petal rose; scale so max radius equals current_range
        theta = np.linspace(0.0, 2.0 * np.pi, num_path_pts)
        r = range_val * np.cos(2.0 * theta)
        x = r * np.cos(theta)
        y = r * np.sin(theta)
        return theta, x, y

    theta_path, x_path, y_path = compute_path_arrays(current_range)
    path_line, = ax.plot(x_path, y_path, color='green', lw=1.2, alpha=0.35)
    target_dot, = ax.plot([x_path[0]], [y_path[0]], 'o', color='green', markersize=12, alpha=0.9)

    t0 = time.monotonic()

    def set_range(new_range: float):
        nonlocal current_range, theta_path, x_path, y_path
        new_range = float(np.clip(new_range, min_range, max_range))
        if new_range == current_range:
            return
        current_range = new_range
        ax.set_xlim(-current_range, current_range)
        ax.set_ylim(-current_range, current_range)
        # Update reference path for new range
        theta_path, x_path, y_path = compute_path_arrays(current_range)
        path_line.set_data(x_path, y_path)
        # Update error plot Y-scale to match new range
        ax_err.set_ylim(0.0, 3.0 * current_range)
        text_settings.set_text(f"Range: ±{current_range:.0f}°, Filter: {'ON' if chk.get_status()[0] else 'OFF'}, N={int(s_win.val)}, Target: {int(s_target.val)}s")
        fig.canvas.draw_idle()

    def do_calibrate(_evt=None):
        nonlocal pending_calibration
        pending_calibration = True
        print('Will calibrate to center on next sample...')

    def do_exit(_evt=None):
        nonlocal data_logger
        stop_event.set()
        print('Exiting plot...')
        
        # End data logging session
        if data_logger:
            data_logger.end_session()
        
        with contextlib.suppress(Exception):
            plt.close(fig)
        with contextlib.suppress(Exception):
            plt.close(fig_ctrl)

    def do_minus(_evt=None):
        set_range(current_range - step)

    def do_plus(_evt=None):
        set_range(current_range + step)

    def on_toggle_run(_evt=None):
        nonlocal is_running, t0, theta_current, data_logger, logging_active
        is_running = not is_running
        if is_running:
            # resume from current theta position
            period = max(1.0, float(s_target.val))
            t0 = time.monotonic() - (theta_current / (2.0 * np.pi)) * period
            btn_run.label.set_text('Stop')
            
            # Start data logging
            patient_id = patient_id_box.text.strip() or "9999"
            if not data_logger:
                data_logger = DataLogger(patient_id)
                sensor_info = {
                    'name': getattr(sensor, '_device_name', 'Unknown'),
                    'address': getattr(sensor, '_device_address', 'Unknown')
                }
                data_logger.start_session(sensor_info)
            logging_active = True
            text_recording.set_text('Recording: ON')
            text_recording.set_color('green')
        else:
            btn_run.label.set_text('Start')
            # Pause logging (don't end session, just pause)
            logging_active = False
            text_recording.set_text('Recording: PAUSED')
            text_recording.set_color('orange')
        fig_ctrl.canvas.draw_idle()

    # Connect button events with error handling for matplotlib mouse grab issues
    try:
        btn_cal.on_clicked(do_calibrate)
        btn_exit.on_clicked(do_exit)
        btn_rminus.on_clicked(do_minus)
        btn_rplus.on_clicked(do_plus)
        btn_run.on_clicked(on_toggle_run)
    except Exception as e:
        print(f"Warning: Error connecting button events: {e}")
        print("Buttons may not respond properly. Try using keyboard shortcuts instead.")

    def on_filter_toggle(_label):
        # Reset buffers when toggling to avoid mixing states
        buf_roll.clear(); buf_pitch.clear()
        text_settings.set_text(f"Range: ±{current_range:.0f}°, Filter: {'ON' if chk.get_status()[0] else 'OFF'}, N={int(s_win.val)}, Target: {int(s_target.val)}s")
        fig_ctrl.canvas.draw_idle()

    def on_filter_size(val):
        # Update buffer sizes and clear to start fresh
        size = int(val)
        buf_roll.clear(); buf_pitch.clear()
        buf_roll.maxlen = size
        buf_pitch.maxlen = size
        text_settings.set_text(f"Range: ±{current_range:.0f}°, Filter: {'ON' if chk.get_status()[0] else 'OFF'}, N={size}, Target: {int(s_target.val)}s")
        fig_ctrl.canvas.draw_idle()

    def on_target_duration(val):
        # Only text update; motion uses slider value each frame
        text_settings.set_text(f"Range: ±{current_range:.0f}°, Filter: {'ON' if chk.get_status()[0] else 'OFF'}, N={int(s_win.val)}, Target: {int(val)}s")
        fig_ctrl.canvas.draw_idle()

    # Connect widget events with error handling
    try:
        chk.on_clicked(on_filter_toggle)
        s_win.on_changed(on_filter_size)
        s_target.on_changed(on_target_duration)
    except Exception as e:
        print(f"Warning: Error connecting widget events: {e}")
        print("Sliders and checkboxes may not respond properly.")

    # Keyboard shortcuts (bind to both figures)
    def on_key(event):
        if event.key in ('c', ' '):
            do_calibrate()
        elif event.key in ('q', 'escape'):
            do_exit()
        elif event.key in ('p',):
            on_toggle_run()

    # Connect keyboard events with error handling
    cid_key = None
    cid_key_ctrl = None
    try:
        cid_key = fig.canvas.mpl_connect('key_press_event', on_key)
        cid_key_ctrl = fig_ctrl.canvas.mpl_connect('key_press_event', on_key)
    except Exception as e:
        print(f"Warning: Error connecting keyboard events: {e}")
        print("Keyboard shortcuts may not work properly.")

    try:
        while not stop_event.is_set() and plt.fignum_exists(fig.number):
            # Update moving green target dot along reference path
            period = max(1.0, float(s_target.val))
            if is_running:
                now = time.monotonic()
                elapsed = (now - t0) % period
                theta_current = (elapsed / period) * (2.0 * np.pi)  # traverse full [0, 2π]
            # Compute target position for current theta
            r_t = current_range * np.cos(2.0 * theta_current)
            x_t = r_t * np.cos(theta_current)
            y_t = r_t * np.sin(theta_current)
            target_dot.set_data([x_t], [y_t])

            if is_running:
                data = sensor.get_collected_data()
            else:
                data = None

            if data and len(data.get('quaternions', [])) > 0:
                q_cur = np.asarray(data['quaternions'][-1], dtype=np.float64)
                q_cur = _quat_normalize(q_cur)

                # Initialize calibration if needed, or apply pending calibration on freshest sample
                if calib_q is None or pending_calibration:
                    calib_q = q_cur
                    pending_calibration = False

                # Relative rotation
                q_rel = _quat_mul(q_cur, _quat_conj(calib_q))
                q_rel = _quat_normalize(q_rel)
                roll, pitch, _yaw = _quat_to_euler_deg(q_rel)

                # Apply moving average if enabled
                if chk.get_status()[0]:
                    buf_roll.append(roll)
                    buf_pitch.append(pitch)
                    roll_f = np.mean(buf_roll) if len(buf_roll) > 0 else roll
                    pitch_f = np.mean(buf_pitch) if len(buf_pitch) > 0 else pitch
                else:
                    roll_f, pitch_f = roll, pitch

                # Map to plot: X = pitch (F/B), Y = -roll (L/R)
                x = np.clip(pitch_f, -current_range, current_range)
                y = np.clip(-roll_f, -current_range, current_range)

                ball.set_data([x], [y])

                # Update readouts
                text_deg.set_text(f"L/R: {roll_f:+.1f}°, F/B: {pitch_f:+.1f}°")
                text_settings.set_text(f"Range: ±{current_range:.0f}°, Filter: {'ON' if chk.get_status()[0] else 'OFF'}, N={int(s_win.val)}, Target: {int(s_target.val)}s")

                # Update error plot (absolute Euclidean error in degrees)
                now_t = time.monotonic()
                err = float(np.hypot(x - x_t, y - y_t))
                err_x = float(x - x_t)
                err_y = float(y - y_t)
                
                # Log data if recording is active
                if logging_active and data_logger:
                     # Get raw euler angles from current quaternion
                     euler_raw = _quat_to_euler_deg(q_cur)
                     
                     data_logger.log_sample(
                         timestamp=now_t,
                         quaternion=q_cur,
                         euler_raw=euler_raw,
                         calibrated_angles=(roll, pitch),
                         filtered_angles=(roll_f, pitch_f),
                         target_pos=(x_t, y_t, theta_current),
                         user_pos=(x, y),
                         error_data=(err, err_x, err_y),
                         config={
                             'range': current_range,
                             'filter_enabled': chk.get_status()[0],
                             'target_speed': int(s_target.val)
                         }
                     )
                
                err_times.append(now_t)
                err_values.append(err)
                # Drop old samples outside the window
                cutoff = now_t - err_window_sec
                while err_times and err_times[0] < cutoff:
                    err_times.popleft(); err_values.popleft()
                # Prepare relative time axis [0, err_window_sec]
                t_rel = [t - cutoff for t in err_times]
                err_line.set_data(t_rel, list(err_values))
                ax_err.set_xlim(0.0, err_window_sec)

            # Draw updates (both figures if present)
            fig.canvas.draw_idle()
            if plt.fignum_exists(fig_ctrl.number):
                fig_ctrl.canvas.draw_idle()

            plt.pause(0.01)
            await asyncio.sleep(0.01)
    finally:
        # Ensure data logger is properly closed
        if data_logger:
            data_logger.end_session()
            
        # Disconnect event handlers safely
        with contextlib.suppress(Exception):
            if cid_key:
                fig.canvas.mpl_disconnect(cid_key)
        with contextlib.suppress(Exception):
            if cid_key_ctrl:
                fig_ctrl.canvas.mpl_disconnect(cid_key_ctrl)
        plt.ioff()
        with contextlib.suppress(Exception):
            plt.close(fig)
        with contextlib.suppress(Exception):
            plt.close(fig_ctrl)

async def main():
    # Scan for sensors
    print("Scanning for Movella DOT sensors (5 seconds)...")
    
    devices = await BleakScanner.discover(timeout=5.0)
    print(f"Found {len(devices)} devices:")
    for device in devices:
        print(f"  {device.name} ({device.address})")
    dot_devices = [d for d in devices if d.name and ("Xsens DOT" in d.name or "Movella DOT" in d.name)]
    
    if not dot_devices:
        print("No Movella DOT sensors found")
        return
        
    # Limit to maximum 1 sensor
    # Should implement multi-sensor support in the future, to select the sensor to use.
    max_sensors = 1
    dot_devices = dot_devices[:max_sensors]
    print(f"Found {len(dot_devices)} Movella DOT sensors")
    
    # Create sensor instances
    sensors = []
    config = SensorConfiguration(
        output_rate=OutputRate.RATE_30,
        filter_profile=FilterProfile.GENERAL,
        payload_mode=PayloadMode.ORIENTATION_QUATERNION
    )
    
    # Connect and configure all sensors
    for device in dot_devices:
        try:
            sensor = MovellaDOTSensor(config)
            sensor.client = BleakClient(device.address)
            print(f"\nConnecting to {device.name} ({device.address})...")
            await sensor.client.connect()
            sensor.is_connected = True
            sensor._device_address = device.address
            sensor._device_name = device.name
            
            # Get and print device info
            print("\nReading device information...")
            device_info = await sensor.get_device_info()
            
            sensor._device_tag = device_info.device_tag
            for key, value in device_info.__dict__.items():
                print(f"{key}: {value}")
            print(f"Current Output Rate: {device_info.output_rate} Hz")
            print(f"Current Filter Profile: {device_info.filter_profile.name}")
            
            # Identify the sensor
            print("\nIdentifying sensor...")
            await sensor.identify_sensor()
            await asyncio.sleep(2)  # Wait for LED blinking
            
            # Configure sensor
            await sensor.configure_sensor()
            sensors.append(sensor)
            print(f"Successfully connected and configured {device.name}")
            
        except Exception as e:
            print(f"Failed to connect to {device.name}: {str(e)}")
    
    if not sensors:
        print("No sensors were successfully connected")
        return
    
    try:
        # Start measurement on all sensors
        print("\nStarting measurements on all sensors...")
        await asyncio.gather(*(sensor.start_measurement() for sensor in sensors))

        print("\nOpening tilt ball plot (press 'c' to calibrate). Close window or press Exit to stop.")
        stop_plot = asyncio.Event()
        plot_task = asyncio.create_task(
            live_plot_ball_from_quat(sensors[0], stop_plot, xy_range=30.0)
        )

        try:
            await plot_task
        except asyncio.CancelledError:
            pass
        except KeyboardInterrupt:
            print("\nStopping on user request...")
        finally:
            stop_plot.set()
            with contextlib.suppress(Exception):
                await plot_task

        print("\nStopping measurements...")
        await asyncio.gather(*(sensor.stop_measurement() for sensor in sensors))
    except Exception as e:
        print(f"Error during measurement: {str(e)}")
    finally:
        # Disconnect all sensors
        print("\nDisconnecting all sensors...")
        await asyncio.gather(*(sensor.disconnect() for sensor in sensors))

if __name__ == "__main__":
    asyncio.run(main())
