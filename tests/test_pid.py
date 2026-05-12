from projects.embedded_controls.simulation.pid_plant_sim import simulate_pid


def test_pid_tracks_setpoint_after_disturbance() -> None:
    result = simulate_pid()
    assert abs(result["final_error"]) < 1.0
    assert result["settling_error_abs"] < 1.2
    assert result["overshoot"] < 8.0

