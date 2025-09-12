# Preface

This repository is an research-specific adaptation of the "Movella DOT Python SDK" originally authored by the upstream project maintainers. We acknowledge and thank the original authors for their work.

Original project: [https://github.com/jiminghe/Xsens_DOT_PC_Reader](https://github.com/jiminghe/Xsens_DOT_PC_Reader)

What we changed or added for our use case:
- Interactive tilt-ball demo for quaternion payloads in `examples/multi_dot_demo.py`:
  - Live 2D ball showing tilt derived from orientation quaternions
  - One-click calibration
  - Start/Stop toggle, adjustable range, optional moving-average filter with window size
  - Moving target path and live absolute error plot for quick training/feedback
- Safer connect/measure/cleanup flow with robust try/finally and graceful shutdown
- Defaults simplified to 1 sensor to reduce BLE contention; easily adjustable
- Mac-friendly quickstart and troubleshooting notes below



This repo is a standalone copy. It does not track the original as an upstream remote. If we need upstream updates later, we will re-import or manually port changes.

## Requirements

- Python 3.7 or higher
- Windows 10 or higher (for BLE support) or mac os
- Movella DOT sensor(s)

## Quickstart

### Windows (PowerShell/Command Prompt)

1) Create a virtual environment and activate it
```cmd
python -m venv .venv
.venv\Scripts\activate
```

2) Install dependencies
```cmd
pip install -r requirements_win.txt
```

3) Run the app
```cmd
python -m mppt.app.mppt
```

### macOS/Linux (bash/zsh)

1) Create a virtual environment and activate it
```bash
python3 -m venv .venv
source .venv/bin/activate
```
2) Install dependencies
```bash
pip install -r requirements.txt
```

3) Run the app
```bash
python3 -m mppt.app.mppt
```

Tested with Python 3.10+ on macOS. BLE uses CoreBluetooth via Bleak.

# MPTT-App

This app connects to a Movella/Xsens DOT sensor via BLE. 

## GUI

The GUI is implemented with PySide6 and embeds a Matplotlib canvas. Async BLE is integrated with the Qt event loop via qasync.

## Setup

On macOS with zsh:

```sh
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Run

```sh
python3 -m mppt.app.mppt
```

## Notes
- Ensure Bluetooth is enabled.
- Some BLE functionality may require granting permissions on macOS.

## Roadmap / TODO


## Configuration Options of originalsoftware. 

### Output Rates (OutputRate)
- `RATE_1`: 1 Hz
- `RATE_4`: 4 Hz
- `RATE_10`: 10 Hz
- `RATE_12`: 12 Hz
- `RATE_15`: 15 Hz
- `RATE_20`: 20 Hz
- `RATE_30`: 30 Hz
- `RATE_60`: 60 Hz
- `RATE_120`: 120 Hz (Recording only)

### Filter Profiles (FilterProfile)
- `GENERAL`: Default profile for general human motion
- `DYNAMIC`: Optimized for fast and jerky human motion like sprinting

### Payload Modes (PayloadMode)
1. Basic Orientations:
   - `ORIENTATION_EULER`: Euler angles
   - `ORIENTATION_QUATERNION`: Quaternion orientation
   - `COMPLETE_EULER`: Euler angles with free acceleration
   - `COMPLETE_QUATERNION`: Quaternion with free acceleration

2. Extended Data:
   - `EXTENDED_EULER`: Euler angles, free acceleration, status, and clipping data
   - `EXTENDED_QUATERNION`: Quaternion, free acceleration, status, and clipping data

3. Motion Data:
   - `FREE_ACCELERATION`: Linear acceleration with gravity compensated
   - `RATE_QUANTITIES`: Acceleration and angular velocity
   - `RATE_QUANTITIES_WITH_MAG`: Acceleration, angular velocity, and magnetic field

4. Custom Modes:
   - `CUSTOM_MODE_1`: Euler angles, free acceleration, angular velocity
   - `CUSTOM_MODE_2`: Euler angles, free acceleration, magnetic field
   - `CUSTOM_MODE_3`: Quaternion and angular velocity
   - `CUSTOM_MODE_5`: Quaternion, acceleration, angular velocity

### Example Configuration

```python
# High-frequency dynamic motion configuration
config = SensorConfiguration(
    output_rate=OutputRate.RATE_60,
    filter_profile=FilterProfile.GENERAL,
    payload_mode=PayloadMode.ORIENTATION_EULER  # Full motion data
)

# Basic orientation tracking configuration
config = SensorConfiguration(
    output_rate=OutputRate.RATE_30,
    filter_profile=FilterProfile.GENERAL,
    payload_mode=PayloadMode.ORIENTATION_EULER
)
```

## Data Collection

The SDK provides various methods to access collected data:

- Quaternions
- Euler angles
- Acceleration (with and without gravity)
- Angular velocity
- Magnetic field data
- Status information
- Clipping detection

## Additional Features

- Device identification (LED blinking)
- Battery status monitoring
- Power saving options
- Device tag configuration
- Time synchronization
- Factory reset capabilities

## Known Limitations

- Some payload modes (`HIGH_FIDELITY`, `HIGH_FIDELITY_WITH_MAG`, `CUSTOM_MODE_4`) are only available with the official Movella SDK
- Maximum of 5 simultaneous sensor connections recommended
- Windows 10 or higher required for BLE functionality

## Build shareable apps (macOS and Windows)

Build on each OS natively (you cannot build a Windows .exe from macOS, and vice versa).

Prereqs (inside your venv):

```bash
pip install pyinstaller
```

### macOS

1) Build the app bundle

```bash
.venv/bin/pyinstaller --noconfirm mppt.spec
```

Output: `dist/MPPT-App.app`

2) Build a DMG (drag-to-Applications installer)

```bash
brew install create-dmg   # once
./build_dmg.sh
```

Output: `dist/MPPT-App.dmg`

Notes:
- On first launch of an unsigned app, right-click the app -> Open -> Open.
- macOS will prompt for Bluetooth access on first run.

### Windows

1) Install deps (PowerShell)

```powershell
python -m venv .venv
. .venv\Scripts\Activate.ps1
pip install -r requirements_win.txt
pip install pyinstaller
```

2) Build the app

```powershell
pyinstaller --noconfirm pyinstaller-win.spec
```

Output: `dist\MPPT-App\MPPT-App.exe` (+ DLLs)

3) Optional: build a Windows installer (.exe)

```powershell
# Install Inno Setup first (https://jrsoftware.org/isinfo.php)
.\n+windows\build_windows.ps1 -Python ".venv\Scripts\python.exe"
```

Output: `MPPT-App-Setup.exe` (path shown by Inno Setup)

Notes:
- Build must be done on the target OS (mac on macOS, win on Windows).
- If antivirus flags the unsigned build, add an allow-list or code-sign the binaries.
- If Qt/matplotlib plugins are missing, update PyInstaller and rebuild.


## Acknowledgments

- [Movella DOT BLE Service Specification](https://www.xsens.com/hubfs/Downloads/Manuals/Xsens%20DOT%20BLE%20Services%20Specifications.pdf)
- [Movella DOT User Manual](https://www.movella.com/hubfs/Movella%20DOT%20User%20Manual.pdf)
- [Bleak](https://github.com/hbldh/bleak) BLE library