from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from projects.attitude_control.attitude_sim import run_attitude_demo
from projects.embedded_controls.simulation.pid_plant_sim import run_pid_demo
from projects.kalman_filter.ekf_sensor_fusion import run_ekf_demo


def main() -> None:
    results_dir = Path("results")
    results_dir.mkdir(exist_ok=True)

    attitude = run_attitude_demo(results_dir / "attitude")
    ekf = run_ekf_demo(results_dir / "ekf")
    pid = run_pid_demo(results_dir / "embedded_pid")

    print("GNC project validation summary")
    print("-" * 36)
    print(f"Attitude final error: {attitude['final_angle_error_deg']:.4f} deg")
    print(f"Attitude final rate:  {attitude['final_rate_norm_rad_s']:.5f} rad/s")
    print(f"EKF position RMSE:    {ekf['position_rmse_m']:.4f} m")
    print(f"PID final error:      {pid['final_error']:.4f}")
    print(f"PID overshoot:        {pid['overshoot']:.4f}")
    print(f"Results written to:   {results_dir.resolve()}")


if __name__ == "__main__":
    main()
