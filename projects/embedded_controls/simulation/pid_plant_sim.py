from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class PidConfig:
    kp: float = 3.2
    ki: float = 0.75
    kd: float = 0.22
    dt: float = 0.02
    duration: float = 40.0
    setpoint: float = 70.0
    output_min: float = 0.0
    output_max: float = 100.0
    integral_min: float = -100.0
    integral_max: float = 100.0


@dataclass(frozen=True)
class PlantConfig:
    ambient: float = 20.0
    time_constant: float = 5.5
    gain: float = 0.95
    disturbance_start: float = 22.0
    disturbance: float = -9.0


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def simulate_pid(
    pid: PidConfig | None = None,
    plant: PlantConfig | None = None,
) -> dict[str, np.ndarray | float]:
    pid = pid or PidConfig()
    plant = plant or PlantConfig()
    steps = int(pid.duration / pid.dt) + 1
    history = np.zeros((steps, 6))

    measurement = plant.ambient
    integral = 0.0
    previous_error = pid.setpoint - measurement
    max_measurement_after_rise = measurement

    for k in range(steps):
        time_s = k * pid.dt
        error = pid.setpoint - measurement
        integral = clamp(integral + error * pid.dt, pid.integral_min, pid.integral_max)
        derivative = (error - previous_error) / pid.dt
        previous_error = error

        control = pid.kp * error + pid.ki * integral + pid.kd * derivative
        control = clamp(control, pid.output_min, pid.output_max)

        disturbance = plant.disturbance if time_s >= plant.disturbance_start else 0.0
        target = plant.ambient + plant.gain * control + disturbance
        measurement += ((target - measurement) / plant.time_constant) * pid.dt

        if time_s > 5.0:
            max_measurement_after_rise = max(max_measurement_after_rise, measurement)
        history[k] = np.array([time_s, pid.setpoint, measurement, control, error, disturbance], dtype=float)

    final_error = pid.setpoint - history[-1, 2]
    overshoot = max(0.0, max_measurement_after_rise - pid.setpoint)
    return {
        "history": history,
        "final_error": float(final_error),
        "overshoot": float(overshoot),
        "settling_error_abs": float(np.mean(np.abs(pid.setpoint - history[-250:, 2]))),
    }


def _write_csv(path: Path, history: np.ndarray) -> None:
    header = "time_s,setpoint,measurement,control,error,disturbance"
    np.savetxt(path, history, delimiter=",", header=header, comments="")


def _plot(out_dir: Path, history: np.ndarray) -> None:
    try:
        import matplotlib.pyplot as plt
    except ModuleNotFoundError:
        print("matplotlib is not installed; skipping PID plots")
        return

    time = history[:, 0]
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(time, history[:, 1], label="setpoint", linewidth=2)
    ax.plot(time, history[:, 2], label="measurement")
    ax.set_title("PID closed-loop response")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("engineering units")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_dir / "pid_response.png", dpi=160)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(time, history[:, 3], label="control")
    ax.plot(time, history[:, 5], label="disturbance")
    ax.set_title("Actuator command and disturbance")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("command / disturbance")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_dir / "pid_control.png", dpi=160)
    plt.close(fig)


def run_pid_demo(out_dir: str | Path = "results/embedded_pid") -> dict[str, float]:
    out_path = Path(out_dir)
    out_path.mkdir(parents=True, exist_ok=True)
    result = simulate_pid()
    history = result["history"]
    assert isinstance(history, np.ndarray)
    _write_csv(out_path / "pid_history.csv", history)
    _plot(out_path, history)
    return {
        "final_error": float(result["final_error"]),
        "overshoot": float(result["overshoot"]),
        "settling_error_abs": float(result["settling_error_abs"]),
    }


if __name__ == "__main__":
    metrics = run_pid_demo()
    for key, value in metrics.items():
        print(f"{key}: {value:.6f}")
