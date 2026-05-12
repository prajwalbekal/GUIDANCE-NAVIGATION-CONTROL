from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class SensorConfig:
    dt: float = 0.02
    duration: float = 80.0
    gps_period: float = 1.0
    accel_noise_std: float = 0.10
    gyro_noise_std: float = 0.006
    gps_pos_noise_std: float = 1.25
    gps_vel_noise_std: float = 0.18
    gyro_bias_true: float = 0.012
    random_seed: int = 7


def wrap_angle(angle: float | np.ndarray) -> float | np.ndarray:
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def controls_at(time_s: float) -> tuple[float, float, float]:
    ax_body = 0.35 * np.sin(0.16 * time_s) + 0.12
    ay_body = 0.22 * np.cos(0.11 * time_s)
    yaw_rate = 0.055 * np.sin(0.07 * time_s) + 0.018
    return ax_body, ay_body, yaw_rate


def truth_step(state: np.ndarray, cfg: SensorConfig, time_s: float) -> np.ndarray:
    px, py, vx, vy, yaw = state
    ax_body, ay_body, yaw_rate = controls_at(time_s)
    c, s = np.cos(yaw), np.sin(yaw)
    ax_world = c * ax_body - s * ay_body
    ay_world = s * ax_body + c * ay_body
    px += vx * cfg.dt + 0.5 * ax_world * cfg.dt**2
    py += vy * cfg.dt + 0.5 * ay_world * cfg.dt**2
    vx += ax_world * cfg.dt
    vy += ay_world * cfg.dt
    yaw = wrap_angle(yaw + yaw_rate * cfg.dt)
    return np.array([px, py, vx, vy, yaw], dtype=float)


def simulate_sensors(cfg: SensorConfig | None = None) -> dict[str, np.ndarray]:
    cfg = cfg or SensorConfig()
    rng = np.random.default_rng(cfg.random_seed)
    steps = int(cfg.duration / cfg.dt) + 1
    gps_stride = max(1, int(round(cfg.gps_period / cfg.dt)))

    truth = np.zeros((steps, 6))
    imu = np.zeros((steps, 4))
    gps = np.full((steps, 5), np.nan)
    state = np.array([0.0, 0.0, 2.5, -0.4, np.radians(8.0)], dtype=float)

    for k in range(steps):
        time_s = k * cfg.dt
        truth[k] = np.array([time_s, *state], dtype=float)
        ax_body, ay_body, yaw_rate = controls_at(time_s)
        imu[k] = np.array(
            [
                time_s,
                ax_body + rng.normal(0.0, cfg.accel_noise_std),
                ay_body + rng.normal(0.0, cfg.accel_noise_std),
                yaw_rate + cfg.gyro_bias_true + rng.normal(0.0, cfg.gyro_noise_std),
            ],
            dtype=float,
        )
        if k % gps_stride == 0:
            gps[k] = np.array(
                [
                    time_s,
                    state[0] + rng.normal(0.0, cfg.gps_pos_noise_std),
                    state[1] + rng.normal(0.0, cfg.gps_pos_noise_std),
                    state[2] + rng.normal(0.0, cfg.gps_vel_noise_std),
                    state[3] + rng.normal(0.0, cfg.gps_vel_noise_std),
                ],
                dtype=float,
            )
        if k < steps - 1:
            state = truth_step(state, cfg, time_s)

    return {"truth": truth, "imu": imu, "gps": gps}


def predict(x: np.ndarray, p: np.ndarray, imu_row: np.ndarray, cfg: SensorConfig) -> tuple[np.ndarray, np.ndarray]:
    _, ax_body, ay_body, gyro_meas = imu_row
    px, py, vx, vy, yaw, gyro_bias = x
    dt = cfg.dt
    yaw_rate = gyro_meas - gyro_bias
    c, s = np.cos(yaw), np.sin(yaw)
    ax_world = c * ax_body - s * ay_body
    ay_world = s * ax_body + c * ay_body

    x_pred = np.array(
        [
            px + vx * dt + 0.5 * ax_world * dt**2,
            py + vy * dt + 0.5 * ay_world * dt**2,
            vx + ax_world * dt,
            vy + ay_world * dt,
            wrap_angle(yaw + yaw_rate * dt),
            gyro_bias,
        ],
        dtype=float,
    )

    dax_dyaw = -s * ax_body - c * ay_body
    day_dyaw = c * ax_body - s * ay_body
    f = np.eye(6)
    f[0, 2] = dt
    f[1, 3] = dt
    f[0, 4] = 0.5 * dax_dyaw * dt**2
    f[1, 4] = 0.5 * day_dyaw * dt**2
    f[2, 4] = dax_dyaw * dt
    f[3, 4] = day_dyaw * dt
    f[4, 5] = -dt

    q = np.diag(
        [
            0.02 * dt,
            0.02 * dt,
            0.18 * dt,
            0.18 * dt,
            0.012 * dt,
            0.00002 * dt,
        ]
    )
    p_pred = f @ p @ f.T + q
    return x_pred, p_pred


def update_gps(x: np.ndarray, p: np.ndarray, gps_row: np.ndarray, cfg: SensorConfig) -> tuple[np.ndarray, np.ndarray]:
    z = gps_row[1:5]
    h = np.array(
        [
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
        ],
        dtype=float,
    )
    r = np.diag(
        [
            cfg.gps_pos_noise_std**2,
            cfg.gps_pos_noise_std**2,
            cfg.gps_vel_noise_std**2,
            cfg.gps_vel_noise_std**2,
        ]
    )
    innovation = z - h @ x
    s = h @ p @ h.T + r
    k = p @ h.T @ np.linalg.inv(s)
    x_upd = x + k @ innovation
    x_upd[4] = wrap_angle(x_upd[4])
    i = np.eye(len(x))
    p_upd = (i - k @ h) @ p @ (i - k @ h).T + k @ r @ k.T
    return x_upd, p_upd


def run_ekf(cfg: SensorConfig | None = None) -> dict[str, np.ndarray | float]:
    cfg = cfg or SensorConfig()
    data = simulate_sensors(cfg)
    truth = data["truth"]
    imu = data["imu"]
    gps = data["gps"]
    steps = len(truth)

    x = np.array([gps[0, 1], gps[0, 2], gps[0, 3], gps[0, 4], np.radians(5.0), 0.0], dtype=float)
    p = np.diag([4.0, 4.0, 0.5, 0.5, np.radians(12.0) ** 2, 0.05**2])
    estimates = np.zeros((steps, 8))

    for k in range(steps):
        if k > 0:
            x, p = predict(x, p, imu[k - 1], cfg)
        if not np.isnan(gps[k, 1]):
            x, p = update_gps(x, p, gps[k], cfg)
        estimates[k] = np.array([truth[k, 0], *x, np.trace(p)], dtype=float)

    pos_error = estimates[:, 1:3] - truth[:, 1:3]
    yaw_error = wrap_angle(estimates[:, 5] - truth[:, 5])
    gps_valid = ~np.isnan(gps[:, 1])
    gps_pos_error = gps[gps_valid, 1:3] - truth[gps_valid, 1:3]

    return {
        "truth": truth,
        "imu": imu,
        "gps": gps,
        "estimates": estimates,
        "position_rmse_m": float(np.sqrt(np.mean(np.sum(pos_error**2, axis=1)))),
        "yaw_rmse_deg": float(np.degrees(np.sqrt(np.mean(yaw_error**2)))),
        "raw_gps_position_rmse_m": float(np.sqrt(np.mean(np.sum(gps_pos_error**2, axis=1)))),
    }


def _write_csv(out_dir: Path, result: dict[str, np.ndarray | float]) -> None:
    truth = result["truth"]
    estimates = result["estimates"]
    gps = result["gps"]
    assert isinstance(truth, np.ndarray)
    assert isinstance(estimates, np.ndarray)
    assert isinstance(gps, np.ndarray)
    merged = np.column_stack([truth, estimates[:, 1:7], gps[:, 1:5]])
    header = (
        "time_s,true_px,true_py,true_vx,true_vy,true_yaw,"
        "est_px,est_py,est_vx,est_vy,est_yaw,est_gyro_bias,"
        "gps_px,gps_py,gps_vx,gps_vy"
    )
    np.savetxt(out_dir / "ekf_history.csv", merged, delimiter=",", header=header, comments="")


def _plot(out_dir: Path, result: dict[str, np.ndarray | float]) -> None:
    try:
        import matplotlib.pyplot as plt
    except ModuleNotFoundError:
        print("matplotlib is not installed; skipping EKF plots")
        return

    truth = result["truth"]
    estimates = result["estimates"]
    gps = result["gps"]
    assert isinstance(truth, np.ndarray)
    assert isinstance(estimates, np.ndarray)
    assert isinstance(gps, np.ndarray)

    valid = ~np.isnan(gps[:, 1])
    fig, ax = plt.subplots(figsize=(6.5, 6.0))
    ax.plot(truth[:, 1], truth[:, 2], label="truth", linewidth=2)
    ax.plot(estimates[:, 1], estimates[:, 2], label="EKF estimate")
    ax.scatter(gps[valid, 1], gps[valid, 2], s=10, alpha=0.45, label="GPS")
    ax.axis("equal")
    ax.set_title("2D trajectory")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_dir / "trajectory.png", dpi=160)
    plt.close(fig)

    time = truth[:, 0]
    pos_error = np.linalg.norm(estimates[:, 1:3] - truth[:, 1:3], axis=1)
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(time, pos_error)
    ax.set_title("EKF position error")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("error (m)")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_dir / "position_error.png", dpi=160)
    plt.close(fig)

    yaw_error = np.degrees(wrap_angle(estimates[:, 5] - truth[:, 5]))
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(time, yaw_error)
    ax.set_title("EKF yaw error")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("error (deg)")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_dir / "yaw_error.png", dpi=160)
    plt.close(fig)


def run_ekf_demo(out_dir: str | Path = "results/ekf") -> dict[str, float]:
    out_path = Path(out_dir)
    out_path.mkdir(parents=True, exist_ok=True)
    result = run_ekf()
    _write_csv(out_path, result)
    _plot(out_path, result)
    return {
        "position_rmse_m": float(result["position_rmse_m"]),
        "yaw_rmse_deg": float(result["yaw_rmse_deg"]),
        "raw_gps_position_rmse_m": float(result["raw_gps_position_rmse_m"]),
    }


if __name__ == "__main__":
    metrics = run_ekf_demo()
    for key, value in metrics.items():
        print(f"{key}: {value:.6f}")
