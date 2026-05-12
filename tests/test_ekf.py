from projects.kalman_filter.ekf_sensor_fusion import run_ekf


def test_ekf_beats_raw_gps_noise() -> None:
    result = run_ekf()
    assert result["position_rmse_m"] < 1.5
    assert result["position_rmse_m"] < result["raw_gps_position_rmse_m"]
    assert result["yaw_rmse_deg"] < 5.0

