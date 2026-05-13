# Guidance, Navigation and Control - Engineering Portfolio

[![CI](https://github.com/prajwalbekal/GUIDANCE-NAVIGATION-CONTROL/actions/workflows/validate.yml/badge.svg)](https://github.com/prajwalbekal/GUIDANCE-NAVIGATION-CONTROL/actions)
![Languages](https://img.shields.io/badge/code-Python%20%7C%20Arduino-informational)
![Domain](https://img.shields.io/badge/domain-Aerospace%20%7C%20Controls%20%7C%20GNC-blueviolet)
![License](https://img.shields.io/badge/license-MIT-green)

Three from-scratch GNC projects with deterministic, reproducible validation, built as a portfolio for aerospace, mechatronics, controls, and test-engineering roles.

> Guidance says where to go. Navigation says where you are. Control closes the loop.

## What's Inside

| # | Project | What it does | One-line pitch |
| --- | --- | --- | --- |
| 1 | [`projects/attitude_control`](projects/attitude_control) | 3-axis spacecraft attitude simulator with PID and LQR-style feedback | Rigid-body dynamics plus closed-loop attitude control for pointing |
| 2 | [`projects/kalman_filter`](projects/kalman_filter) | Extended Kalman Filter for noisy IMU and GPS sensor fusion | Real-time state estimation from messy sensors |
| 3 | [`projects/embedded_controls`](projects/embedded_controls) | Arduino closed-loop PID plus desktop plant simulator | Hardware-side controller with a software twin for safer tuning |

Each project folder has its own README with theory, equations, usage, tuning notes, and resume talking points.

## Results Snapshot

All numbers below are produced deterministically by `scripts/run_all.py` and checked by `scripts/validate.py`.

| Project | Metric | Result |
| --- | --- | ---: |
| Attitude Control | Final attitude error | `0.5032 deg` |
| Attitude Control | Final body-rate norm | `0.00361 rad/s` |
| EKF Sensor Fusion | Position RMSE | `0.6911 m` |
| Embedded PID | Final tracking error | `0.0494 units` |
| Embedded PID | Overshoot | `2.9259 units` |

## Visual Results

| Attitude Control | EKF Sensor Fusion | Embedded PID |
| :---: | :---: | :---: |
| ![Attitude error response](docs/images/attitude_error.png) | ![EKF trajectory estimate](docs/images/ekf_trajectory.png) | ![PID closed-loop response](docs/images/pid_response.png) |
| Attitude error vs time for the demo slew | EKF estimate overlaid on the truth trajectory | Desktop plant response under closed-loop PID |

## Quick Start

### Linux / macOS

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python -m pytest -q
python scripts/run_all.py
```

### Windows PowerShell

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
python -m pytest -q
python scripts\run_all.py
```

Generated CSV files and plots are written to `results/`.

If you do not want to install `pytest`, the built-in validation checks still run:

```bash
python scripts/validate.py
```

## Skills Demonstrated

- **Control systems** - PID tuning, LQR-style state feedback, overshoot tracking, settling behavior, actuator limits
- **State estimation** - Extended Kalman Filter, IMU/GPS sensor fusion, noise modeling, gyro-bias estimation
- **Rigid-body dynamics** - quaternion kinematics, body rates, Euler rotational equations, RK4 integration
- **Embedded control** - Arduino sensor-read to actuator-write loop, serial gain tuning, PWM output, anti-windup
- **Verification engineering** - deterministic seeds, validation thresholds, pytest tests, GitHub Actions CI
- **Python toolchain** - NumPy, Matplotlib, pytest, reproducible scripts

## Expected Validation Thresholds

`python scripts/run_all.py` should produce:

- Attitude error below `1.0 deg`
- Body-rate norm below `0.02 rad/s`
- EKF position RMSE below `1.5 m`
- Embedded PID final tracking error below `1.0 unit`

Small numerical differences are normal across Python and NumPy versions.

## Project Layout

```text
projects/
  attitude_control/      # 3-axis rigid-body dynamics + PID / LQR-style feedback
  kalman_filter/         # EKF for IMU + GPS sensor fusion
  embedded_controls/     # Arduino firmware + Python plant simulator
scripts/
  run_all.py             # Runs every demo end-to-end
  validate.py            # Deterministic threshold checks
tests/                   # pytest suite
docs/images/             # Generated result plots
.github/workflows/       # GitHub Actions CI
```

## Author

Prajwal Bekal  
M.Sc. Mechatronics and Cyber-Physical Systems, Deggendorf Institute of Technology  
[GitHub](https://github.com/prajwalbekal) | [LinkedIn](https://de.linkedin.com/in/prajwal-bekal-5117b1150)

