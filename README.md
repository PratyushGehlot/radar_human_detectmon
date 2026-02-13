# Radar Human Detection & Fall Monitor

An ESP32-S3-BOX-3 based real-time human presence detection and fall monitoring system using a ceiling-mounted **LD6001 mmWave radar sensor**. The system classifies human posture (standing, sitting, lying, sleeping) and detects falls with audio alerts — all without cameras, preserving privacy.

<img width="1181" height="724" alt="3d_point_cloud" src="https://github.com/user-attachments/assets/f42bbdeb-e5ca-42a0-b9d1-09aac4e541da" />

## Features

- **Real-time posture classification** — Standing, Sitting, Lying, Sleeping
- **Fall detection** with multi-evidence state machine and audio alarm
- **Multi-target tracking** — up to 5 simultaneous people
- **Privacy-preserving** — radar-only sensing, no camera or image capture
- **Touchscreen UI** — 4 swipeable screens (boot, settings, monitor, debug)
- **WiFi streaming** — raw point cloud data streamed to PC for 3D visualization
- **Audio alerts** — distinct sounds for presence detection and fall events
- **Configurable thresholds** — all detection parameters tunable at compile time

## Hardware

| Component | Model | Interface |
|---|---|---|
| MCU + Display | [ESP32-S3-BOX-3](https://github.com/espressif/esp-box) | — |
| Radar Sensor | LD6001 mmWave | UART1 (TX: GPIO 38, RX: GPIO 39, 115200 baud) |
| Audio Codec | ES8311 (built-in) | I2S + I2C |
| Touch Panel | GT911 (built-in) | I2C |
| Flash | 16 MB QIO | SPI |
| PSRAM | 8 MB Octal | SPI (80 MHz) |

### Wiring

The radar sensor connects to the ESP32-S3-BOX-3 via the UART expansion header:

| Radar Pin | ESP32-S3 GPIO | Function |
|---|---|---|
| TX | GPIO 39 | Radar data output → ESP RX |
| RX | GPIO 38 | ESP TX → Radar commands |
| VCC | 3.3V | Power |
| GND | GND | Ground |

The radar must be **ceiling-mounted, pointing downward**. Set `GRID_Z` in `main/app/radar_sensor.c` to match your actual ceiling height (default: 3.0m).

## Detection Pipeline

```
UART RX → Point Filter → DBSCAN Cluster → Feature Extract → Posture Classify → Fall FSM → Track & Smooth → UI/Audio
```

1. **Point Filtering** — Weighted confidence score (SNR, abs, DPK), threshold ≥ 0.4
2. **DBSCAN Clustering** — Groups nearby points into human targets (eps=0.55m, min_samples=5)
3. **Feature Extraction** — Per-cluster z_max, z_range, xy_span, velocity mean/variance
4. **Posture Classification** — Rule-based using height, vertical extent, and spatial spread
5. **Fall Detection** — 4-state FSM (NONE → SUSPECT → CONFIRMED → COOLDOWN) with multi-evidence scoring
6. **Target Tracking** — Nearest-neighbor association + EMA smoothing (α=0.4) across frames

### Fall Detection State Machine

| Transition | Condition |
|---|---|
| NONE → SUSPECT | 2+ of: downward velocity, z-drop, acceleration spike, height collapse, posture transition |
| SUSPECT → CONFIRMED | Evidence sustained for 2+ consecutive frames |
| CONFIRMED → COOLDOWN | 5s hold time + 5 consecutive upright frames |
| COOLDOWN → NONE | Upright posture detected |

## Project Structure

```
radar_human_detectmon/
├── main/
│   ├── app/                        # Core application
│   │   ├── esp32s3_box3_appmain.c  # Entry point (app_main)
│   │   ├── radar_sensor.c/.h       # Radar UART, detection, tracking
│   │   ├── wifi_stream.c/.h        # WiFi AP + TCP streaming server
│   │   └── app_play_wav.c/.h       # WAV audio playback via I2S
│   ├── gui/                        # LVGL UI (SquareLine Studio export)
│   │   ├── screens/                # Screen1–Screen4 definitions
│   │   ├── images/                 # Embedded PNG icons (posture, alert)
│   │   ├── ui_events.c             # Monitor timer, audio triggers, gestures
│   │   └── ui.c                    # UI initialization
│   └── CMakeLists.txt
├── components/bsp/                 # Board Support Package (modified for BOX-3)
├── managed_components/             # Auto-downloaded ESP-IDF dependencies
├── spiffs/                         # Audio files (flashed to SPIFFS partition)
│   ├── bootaudio.wav
│   ├── attention.wav               # Fall alert sound
│   └── humanpresensedetected.wav   # Presence notification
├── pc_app/                         # PC companion tools
│   ├── main.py    # 3D point cloud visualizer (Over WiFi/COM)
│   ├── HumanRadar_PC_Visualizer.cp313-win_amd64.pyd #pc app library
│   └── config.json					# GUI configuration
├── partitions.csv                  # Flash partition table (16 MB)
├── sdkconfig.defaults              # Default build configuration
└── idf_component.yml               # Component manager dependencies
```

## Prerequisites

- [ESP-IDF v5.5.2](https://docs.espressif.com/projects/esp-idf/en/v5.5.2/esp32s3/get-started/) or later
- [VS Code](https://code.visualstudio.com/) with [ESP-IDF Extension](https://marketplace.visualstudio.com/items?itemName=espressif.esp-idf-extension) (recommended)
- Python 3.11+ (for PC visualization app)
- [SquareLine Studio](https://squareline.io/) (only if modifying the UI layout)

## Build & Flash

### Using ESP-IDF CLI

```bash
# Set target
idf.py set-target esp32s3

# Build firmware + SPIFFS image
idf.py build

# Flash to device (adjust COM port)
idf.py -p COM9 flash

# Monitor serial output
idf.py -p COM9 monitor

# Build, flash, and monitor in one step
idf.py -p COM9 flash monitor
```

### Using VS Code

1. Open the project folder in VS Code
2. ESP-IDF extension auto-detects the project
3. Use the toolbar buttons: **Build** → **Flash** → **Monitor**
4. Port and target are configured in `.vscode/settings.json`

## PC Visualization App

A Python companion app connects to the ESP32's WiFi AP to visualize the radar point cloud in 3D.

```bash
# Connect your PC to the ESP32 WiFi AP
# SSID: ESP32S3BOX3_RadarSensor

# Run the visualizer
cd pc_app main.py
```

The app connects to `192.168.4.1:3333` and renders the point cloud with cluster labels and posture overlay.

## UI Screens

| Screen | Description |
|---|---|
| Screen 1 | Boot splash with progress bar animation |
<img width="513" height="454" alt="image" src="https://github.com/user-attachments/assets/65c29b7b-af1d-411f-a2fd-0fa33e5d724b" />

| Screen 2 | Settings — height, range, sensitivity sliders, audio alert toggle |
<img width="518" height="454" alt="image" src="https://github.com/user-attachments/assets/2538e131-d343-4ad0-8e64-85d17b076c7a" />

| Screen 3 | **Main monitor** — posture icon, detection status, fall/safe indicator |
<img width="518" height="455" alt="image" src="https://github.com/user-attachments/assets/5a604ad6-2ffc-454b-9306-b36b5a1a3d2d" />

| Screen 4 | Debug log — target coordinates, WiFi client IP |
<img width="513" height="450" alt="image" src="https://github.com/user-attachments/assets/f908e96e-ec85-40b2-a2d1-d6ea9cc3fe86" />

Navigate between screens by swiping left/right on the touchscreen.

## Configuration

All detection parameters are configurable via `RADAR_CONFIG_DEFAULT()` in `main/app/radar_sensor.h`:

| Parameter | Default | Description |
|---|---|---|
| `eps` | 0.55m | DBSCAN clustering radius |
| `min_samples` | 5 | Minimum points per cluster |
| `standing_z` | 1.0m | Height threshold for standing posture |
| `sitting_z` | 0.6m | Height threshold for sitting posture |
| `lying_z` | 0.25m | Height threshold for lying posture |
| `fall_v_threshold` | -0.3 m/s | Downward velocity for fall evidence |
| `fall_z_drop_threshold` | 0.10m | Z-drop per frame for fall evidence |
| `fall_height_collapse` | 0.25m | Max height drop for fall evidence |
| `fall_hold_time_us` | 5,000,000 | Fall confirmed hold time (5 seconds) |
| `track_gate_radius` | 0.8m | Max distance for track association |

### Tuning Tips

1. Enable debug logging: call `esp_log_level_set("radar_sensor", ESP_LOG_DEBUG)` in `app_main()`
2. Perform test actions (stand, sit, lie down, fall) and observe logged feature values
3. Set `GRID_Z` to your actual ceiling height
4. Adjust posture thresholds to midpoints between observed z_max values for each posture
5. For fall detection, set thresholds just above your noise floor based on logged `zdrop`, `v`, `a` values

## Flash Partition Layout

| Partition | Size | Purpose |
|---|---|---|
| `ota_0` | 4200 KB | Application firmware |
| `storage` | 2600 KB | SPIFFS (audio files) |
| `model` | 8600 KB | Reserved for ML models |
| `nvs` | 24 KB | Non-volatile storage |

## FreeRTOS Tasks

| Task | Stack | Priority | Description |
|---|---|---|---|
| `radar_rx_task` | 8 KB | 10 | UART read + full detection pipeline |
| `tcp_accept_task` | 4 KB | 5 | WiFi TCP client accept loop |
| `LVGL task` | configurable | configurable | Display rendering loop |
| `Lowpower Task` | 4 KB | 5 | Power management monitor |
| `alert_audio` | 4 KB | 5 | Transient — plays alert WAV |
| `boot_audio` | 4 KB | 5 | Transient — plays boot WAV |

## License

This project is provided as-is for educational and research purposes.
