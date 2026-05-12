from projects.attitude_control.attitude_sim import simulate_attitude


def test_attitude_controller_converges() -> None:
    result = simulate_attitude()
    assert result["final_angle_error_deg"] < 1.0
    assert result["final_rate_norm_rad_s"] < 0.02

