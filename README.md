# Teknofest 2021 International UAV Competition

**Red Color Detection & Autonomous UAV Missions**

---

## Table of Contents
- [Overview](#overview)
- [Features](#features)
- [Package Structure](#package-structure)
- [Dependencies](#dependencies)
- [Installation](#installation)
- [Algorithm Flow](#algorithm-flow)
- [Usage Examples](#usage-examples)
  - [Simple Detection](#simple-detection)
  - [Mission 1: SimpleDetectionMission](#mission-1-simpledetectionmission)
  - [Mission 2: RedDetectMission](#mission-2-reddetectmission)
- [Development](#development)
- [Contributing](#contributing)
- [License](#license)

---

## Overview
This repository implements real-time red-color detection and autonomous UAV missions using OpenCV and DroneKit. It was developed for the Teknofest 2021 International UAV Competition.

- **Detection**: HSV-based red-color thresholding, morphology, and contour extraction.
- **Missions**: Class-based abstractions for simple detection and autonomous flight control.

## Features
- Modular `detection` package with an abstract base class and `RedColorDetector`.
- `missions` package with:
  - `SimpleDetectionMission` (FPS overlay + optional video recording)
  - `RedDetectMission` (DroneKit-based red detection + yaw control)
- CLI entry points for each mission.
- Configurable parameters via command-line flags.

## Package Structure
```
2021-Teknofest-International-UAV-Competition/
├── detection/
│   ├── __init__.py
│   ├── base.py               # Abstract Detector
│   └── red_color_detector.py # HSV threshold logic
├── missions/
│   ├── __init__.py
│   ├── simple_detection_mission.py  # Mission1
│   ├── flight_mission.py      # Abstract FlightMission
│   └── red_detect_mission.py  # Mission2
├── apps/
│   └── base_app.py            # CLI app base
├── requirements.txt
├── setup.py
├── .gitignore
└── README.md
```

## Dependencies
- Python 3.6+
- numpy >= 1.19.0
- opencv-python >= 4.5.0
- dronekit >= 2.0.0
- pymavlink >= 2.4.0
- RPi.GPIO >= 0.7.0 (optional for Raspberry Pi)

## Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/your-org/2021-Teknofest-International-UAV-Competition.git
   cd 2021-Teknofest-International-UAV-Competition
   ```
2. Create and activate a virtual environment:
   ```bash
   python3 -m venv venv
   source venv/bin/activate  # Linux/macOS
   venv\Scripts\activate     # Windows
   ```
3. Install requirements:
   ```bash
   pip install -r requirements.txt
   ```
4. (Optional) Install as a package:
   ```bash
   pip install -e .
   ```

## Algorithm Flow
1. **Capture Frame**: Read from camera index.
2. **Preprocessing**: Rotate 180°, apply Gaussian blur.
3. **HSV Thresholding**: Mask red range, morphological clean-up.
4. **Contour Detection**: Find largest contour, compute bounding box.
5. **Output**:
   - For detection: draw box and display or save image.
   - For missions: compute area/center, send MAVLink commands for yaw or waypoint.

## Usage Examples

### Simple Detection
Display live detection with FPS:
```bash
python -m missions.simple_detection_mission --camera 0
```
Record output video:
```bash
python -m missions.simple_detection_mission --camera 0 --record --output=red.avi
```

### Mission 1: SimpleDetectionMission
```bash
python -m missions.simple_detection_mission --camera 0 --record --output=mission1.avi
```

### Mission 2: RedDetectMission
Arm, takeoff at 10m, then detect and turn toward red object:
```bash
python -m missions.red_detect_mission --conn 127.0.0.1:14551 --field 150 --alt 10
```

## Development
- Write new detectors under `detection/` by subclassing `Detector`.
- Add missions under `missions/` by subclassing `FlightMission`.
- Ensure CLI entry points and docstrings are updated.

