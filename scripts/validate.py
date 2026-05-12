from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from projects.attitude_control.attitude_sim import simulate_attitude
from projects.embedded_controls.simulation.pid_plant_sim import simulate_pid
from projects.kalman_filter.ekf_sensor_fusion import run_ekf


def main() -> None:
    attitude = simulate_attitude()
    ekf = run_ekf()
    pid = simulate_pid()

    checks = [
        ("attitude final error < 1 deg", attitude["final_angle_error_deg"] < 1.0),
        ("attitude final rate < 0.02 rad/s", attitude["final_rate_norm_rad_s"] < 0.02),
        ("EKF position RMSE < 1.5 m", ekf["position_rmse_m"] < 1.5),
        ("EKF beats raw GPS position RMSE", ekf["position_rmse_m"] < ekf["raw_gps_position_rmse_m"]),
        ("EKF yaw RMSE < 5 deg", ekf["yaw_rmse_deg"] < 5.0),
        ("PID final error < 1 unit", abs(pid["final_error"]) < 1.0),
        ("PID settling error < 1.2 units", pid["settling_error_abs"] < 1.2),
        ("PID overshoot < 8 units", pid["overshoot"] < 8.0),
    ]

    failed = False
    for label, passed in checks:
        print(f"{'PASS' if passed else 'FAIL'} - {label}")
        failed = failed or not passed

    if failed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

