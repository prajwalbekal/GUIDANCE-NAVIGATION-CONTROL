from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class AttitudeConfig:
    inertia: np.ndarray
    dt: float = 0.01
    duration: float = 60.0
    max_torque: float = 0.08
    kp: np.ndarray = field(default_factory=lambda: np.array([0.17, 0.19, 0.15]))
    kd: np.ndarray = field(default_factory=lambda: np.array([0.78, 0.85, 0.72]))


def quat_normalize(q: np.ndarray) -> np.ndarray:
    return q / np.linalg.norm(q)


def quat_conjugate(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=float)


def quat_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        dtype=float,
    )


def quat_from_euler(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = np.cos(roll / 2.0), np.sin(roll / 2.0)
    cp, sp = np.cos(pitch / 2.0), np.sin(pitch / 2.0)
    cy, sy = np.cos(yaw / 2.0), np.sin(yaw / 2.0)
    return quat_normalize(
        np.array(
            [
                cr * cp * cy + sr * sp * sy,
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
            ],
            dtype=float,
        )
    )


def quat_angle_error_deg(q_actual: np.ndarray, q_desired: np.ndarray) -> float:
    q_err = quat_multiply(q_desired, quat_conjugate(q_actual))
    q_err = quat_normalize(q_err)
    angle = 2.0 * np.arccos(np.clip(abs(q_err[0]), -1.0, 1.0))
    return float(np.degrees(angle))


def attitude_controller(
    q: np.ndarray,
    omega: np.ndarray,
    q_desired: np.ndarray,
    cfg: AttitudeConfig,
    mode: str = "pid",
) -> np.ndarray:
    q_err = quat_multiply(q_desired, quat_conjugate(q))
    q_err = quat_normalize(q_err)
    if q_err[0] < 0.0:
        q_err = -q_err

    if mode == "pid":
        torque = cfg.kp * q_err[1:4] - cfg.kd * omega
    elif mode == "lqr":
        angle_error = 2.0 * q_err[1:4]
        k_angle = np.array([0.18, 0.20, 0.16])
        k_rate = np.array([1.05, 1.12, 0.98])
        torque = k_angle * angle_error - k_rate * omega
    else:
        raise ValueError("mode must be 'pid' or 'lqr'")

    return np.clip(torque, -cfg.max_torque, cfg.max_torque)


def state_derivative(
    state: np.ndarray,
    torque: np.ndarray,
    inertia: np.ndarray,
    inertia_inv: np.ndarray,
) -> np.ndarray:
    q = state[:4]
    omega = state[4:7]
    wx, wy, wz = omega
    omega_matrix = np.array(
        [
            [0.0, -wx, -wy, -wz],
            [wx, 0.0, wz, -wy],
            [wy, -wz, 0.0, wx],
            [wz, wy, -wx, 0.0],
        ],
        dtype=float,
    )
    q_dot = 0.5 * omega_matrix @ q
    omega_dot = inertia_inv @ (torque - np.cross(omega, inertia @ omega))
    return np.concatenate([q_dot, omega_dot])


def rk4_step(
    state: np.ndarray,
    torque: np.ndarray,
    cfg: AttitudeConfig,
    inertia_inv: np.ndarray,
) -> np.ndarray:
    f = lambda s: state_derivative(s, torque, cfg.inertia, inertia_inv)
    dt = cfg.dt
    k1 = f(state)
    k2 = f(state + 0.5 * dt * k1)
    k3 = f(state + 0.5 * dt * k2)
    k4 = f(state + dt * k3)
    new_state = state + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
    new_state[:4] = quat_normalize(new_state[:4])
    return new_state


def simulate_attitude(
    cfg: AttitudeConfig | None = None,
    mode: str = "pid",
    q0: np.ndarray | None = None,
    omega0: np.ndarray | None = None,
    q_desired: np.ndarray | None = None,
) -> dict[str, np.ndarray | float]:
    cfg = cfg or AttitudeConfig(inertia=np.diag([6.0, 7.5, 5.2]))
    q_desired = q_desired if q_desired is not None else np.array([1.0, 0.0, 0.0, 0.0])
    q0 = q0 if q0 is not None else quat_from_euler(np.radians(35), np.radians(-25), np.radians(50))
    omega0 = omega0 if omega0 is not None else np.array([0.06, -0.045, 0.035])

    steps = int(cfg.duration / cfg.dt) + 1
    state = np.concatenate([quat_normalize(q0), omega0.astype(float)])
    inertia_inv = np.linalg.inv(cfg.inertia)

    history = np.zeros((steps, 13))
    for idx in range(steps):
        time = idx * cfg.dt
        q = state[:4]
        omega = state[4:7]
        torque = attitude_controller(q, omega, q_desired, cfg, mode=mode)
        history[idx] = np.array(
            [
                time,
                quat_angle_error_deg(q, q_desired),
                np.linalg.norm(omega),
                *q,
                *omega,
                *torque,
            ],
            dtype=float,
        )
        if idx < steps - 1:
            state = rk4_step(state, torque, cfg, inertia_inv)

    return {
        "history": history,
        "final_angle_error_deg": float(history[-1, 1]),
        "final_rate_norm_rad_s": float(history[-1, 2]),
        "max_torque_abs": float(np.max(np.abs(history[:, 8:11]))),
    }


def _write_csv(path: Path, history: np.ndarray) -> None:
    header = "time_s,angle_error_deg,rate_norm_rad_s,qw,qx,qy,qz,wx,wy,wz,tx,ty,tz"
    np.savetxt(path, history, delimiter=",", header=header, comments="")


def _plot(history: np.ndarray, out_dir: Path) -> None:
    try:
        import matplotlib.pyplot as plt
    except ModuleNotFoundError:
        print("matplotlib is not installed; skipping attitude plots")
        return

    time = history[:, 0]
    plots = [
        ("attitude_error.png", "Attitude error", "deg", history[:, 1:2], ["angle error"]),
        ("body_rates.png", "Body rates", "rad/s", history[:, 7:10], ["wx", "wy", "wz"]),
        ("control_torque.png", "Control torque", "N*m", history[:, 10:13], ["tx", "ty", "tz"]),
    ]
    for filename, title, ylabel, series, labels in plots:
        fig, ax = plt.subplots(figsize=(8, 4.5))
        for col, label in zip(series.T, labels):
            ax.plot(time, col, label=label)
        ax.set_title(title)
        ax.set_xlabel("time (s)")
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)
        ax.legend()
        fig.tight_layout()
        fig.savefig(out_dir / filename, dpi=160)
        plt.close(fig)


def run_attitude_demo(out_dir: str | Path = "results/attitude") -> dict[str, float]:
    out_path = Path(out_dir)
    out_path.mkdir(parents=True, exist_ok=True)
    result = simulate_attitude()
    history = result["history"]
    assert isinstance(history, np.ndarray)
    _write_csv(out_path / "attitude_history.csv", history)
    _plot(history, out_path)
    return {
        "final_angle_error_deg": float(result["final_angle_error_deg"]),
        "final_rate_norm_rad_s": float(result["final_rate_norm_rad_s"]),
        "max_torque_abs": float(result["max_torque_abs"]),
    }


if __name__ == "__main__":
    metrics = run_attitude_demo()
    for key, value in metrics.items():
        print(f"{key}: {value:.6f}")
