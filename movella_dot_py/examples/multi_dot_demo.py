import asyncio
from bleak import BleakScanner, BleakClient
import sys
import os
import time
import numpy as np
import matplotlib.pyplot as plt
import contextlib


# Add the parent directory to the Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from movella_dot_py.core.sensor import MovellaDOTSensor
from movella_dot_py.models.data_structures import SensorConfiguration
from movella_dot_py.models.enums import OutputRate, FilterProfile, PayloadMode

async def live_plot_euler(sensor: MovellaDOTSensor, stop_event: asyncio.Event, window_seconds: float = 10.0):
    """Live plot roll, pitch, yaw from a single sensor.
    Updates at ~20 Hz without blocking the asyncio loop.
    """
    plt.ion()
    fig, ax = plt.subplots(figsize=(9, 4))
    line_roll, = ax.plot([], [], label="Roll")
    line_pitch, = ax.plot([], [], label="Pitch")
    line_yaw, = ax.plot([], [], label="Yaw")

    ax.set_title(f"Live Euler Angles - {getattr(sensor, '_device_tag', '') or getattr(sensor, '_device_address', 'Sensor')}")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angle (deg)")
    ax.set_ylim(-180, 180)
    ax.grid(True)
    ax.legend(loc="upper right")

    try:
        while not stop_event.is_set() and plt.fignum_exists(fig.number):
            data = sensor.get_collected_data()
            if data and len(data.get('timestamps', [])) > 0 and len(data.get('euler_angles', [])) > 0:
                ts = np.array(data['timestamps'], dtype=np.float64)
                eul = np.array(data['euler_angles'], dtype=np.float64)  # shape (N, 3) => roll, pitch, yaw

                # Convert microseconds to seconds relative to first timestamp
                t = (ts - ts[0]) / 1e6

                # Keep a sliding window
                t_end = t[-1]
                t_start = max(0.0, t_end - window_seconds)
                mask = t >= t_start

                t_w = t[mask]
                roll = eul[mask, 0]
                pitch = eul[mask, 1]
                yaw = eul[mask, 2]

                line_roll.set_data(t_w, roll)
                line_pitch.set_data(t_w, pitch)
                line_yaw.set_data(t_w, yaw)

                # Update x-limits to follow the window
                ax.set_xlim(t_start, max(window_seconds, t_end))
                # Y already fixed to [-180, 180]

                fig.canvas.draw_idle()
            # Allow GUI to process events and throttle updates
            plt.pause(0.01)
            await asyncio.sleep(0.01)
    finally:
        plt.ioff()
        try:
            plt.close(fig)
        except Exception:
            pass

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

async def live_plot_tilt_from_quat(sensor: MovellaDOTSensor, stop_event: asyncio.Event, window_seconds: float = 10.0, show_yaw: bool = False):
    """Live, human-readable tilt plot from quaternion stream with calibration.
    - Press 'c' or space in the plot window to set the current pose as zero.
    - Plots Left/Right (roll) and Forward/Backward (pitch) in degrees relative to calibration.
    """
    plt.ion()
    fig, ax = plt.subplots(figsize=(9, 4))
    line_roll, = ax.plot([], [], label="Left/Right (Roll)")
    line_pitch, = ax.plot([], [], label="Forward/Backward (Pitch)")
    line_yaw = None
    if show_yaw:
        line_yaw, = ax.plot([], [], label="Twist (Yaw)", linestyle='--', alpha=0.6)

    ax.set_title("Live Tilt (press 'c' to calibrate)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angle (deg)")
    ax.set_ylim(-90, 90)
    ax.grid(True)
    ax.legend(loc="upper right")

    calib_q = None  # quaternion at rest [w,x,y,z]

    def on_key(event):
        nonlocal calib_q
        if event.key in ("c", " "):
            data = sensor.get_collected_data()
            quats = data.get('quaternions', []) if data else []
            if quats:
                calib_q = _quat_normalize(np.asarray(quats[-1], dtype=np.float64))
                print("Calibrated zero pose.")
            else:
                print("No data yet to calibrate.")
        elif event.key == "r":
            calib_q = None
            print("Calibration cleared.")

    cid = fig.canvas.mpl_connect('key_press_event', on_key)

    try:
        while not stop_event.is_set() and plt.fignum_exists(fig.number):
            data = sensor.get_collected_data()
            if data and len(data.get('timestamps', [])) > 0 and len(data.get('quaternions', [])) > 0:
                ts = np.asarray(data['timestamps'], dtype=np.float64)
                qs = np.asarray(data['quaternions'], dtype=np.float64)  # shape (N, 4) => w,x,y,z
                qs = _quat_normalize(qs)

                # Convert microseconds to seconds relative to first timestamp
                t = (ts - ts[0]) / 1e6

                # Sliding window
                t_end = t[-1]
                t_start = max(0.0, t_end - window_seconds)
                mask = t >= t_start
                t_w = t[mask]
                qs_w = qs[mask]

                if qs_w.size > 0:
                    # Initialize calibration if not set yet
                    if calib_q is None:
                        calib_q = qs_w[0]

                    # Compute relative rotation q_rel = q_curr * conj(q_calib)
                    rel_angles = []
                    q_calib_conj = _quat_conj(calib_q)
                    for q_cur in qs_w:
                        q_rel = _quat_mul(q_cur, q_calib_conj)
                        q_rel = _quat_normalize(q_rel)
                        rel_angles.append(_quat_to_euler_deg(q_rel))
                    rel_angles = np.vstack(rel_angles)  # shape (M, 3)

                    roll = rel_angles[:, 0]
                    pitch = rel_angles[:, 1]
                    yaw = rel_angles[:, 2]

                    line_roll.set_data(t_w, roll)
                    line_pitch.set_data(t_w, pitch)
                    if show_yaw and line_yaw is not None:
                        line_yaw.set_data(t_w, yaw)

                    # Update x-limits
                    ax.set_xlim(t_start, max(window_seconds, t_end))
                    fig.canvas.draw_idle()

            plt.pause(0.01)
            await asyncio.sleep(0.01)
    finally:
        try:
            fig.canvas.mpl_disconnect(cid)
        except Exception:
            pass
        plt.ioff()
        with contextlib.suppress(Exception):
            plt.close(fig)

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
        
    # Limit to maximum 5 sensors
    max_sensors = 1
    dot_devices = dot_devices[:max_sensors]
    print(f"Found {len(dot_devices)} Movella DOT sensors")
    
    # Create sensor instances
    sensors = []
    # Change to your desired values.
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

        print("\nOpening live tilt plot (press 'c' to calibrate). Close window to stop.")
        stop_plot = asyncio.Event()
        plot_task = asyncio.create_task(live_plot_tilt_from_quat(sensors[0], stop_plot, window_seconds=10.0, show_yaw=False))

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
