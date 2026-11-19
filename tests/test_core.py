import json

import pytest

from control_sim_lab import PIDController, Plant, compare, simulate, step_metrics
from control_sim_lab.__main__ import main


def test_first_order_open_loop_settles_to_gain():
    plant = Plant(kind="first_order", gain=2.0, tau=1.0)
    state = plant.initial_state()
    for _ in range(1000):
        state = plant.step(state, 1.0, 0.01)
    assert state[0] == pytest.approx(2.0, abs=0.05)


@pytest.mark.parametrize("kwargs", [
    {"kind": "first_order", "tau": 0.0},
    {"kind": "second_order", "omega_n": -1.0},
    {"kind": "second_order", "zeta": -0.1},
    {"kind": "unknown"},
])
def test_invalid_plant_rejected(kwargs):
    with pytest.raises(ValueError):
        Plant(**kwargs)


def test_invalid_controller_rejected():
    with pytest.raises(ValueError):
        PIDController("bad", kp=float("nan"))
    with pytest.raises(ValueError):
        PIDController("bad", kp=1.0, u_min=2.0, u_max=1.0)


def test_p_has_offset_pi_removes_it():
    plant = Plant(kind="first_order", gain=1.0, tau=1.0)
    p = PIDController("P", kp=2.0, u_min=-2.0, u_max=2.0)
    pi = PIDController("PI", kp=2.0, ki=1.5, u_min=-2.0, u_max=2.0)
    p_result = simulate(plant, p, 1.0, 5.0, 0.05)
    pi_result = simulate(plant, pi, 1.0, 5.0, 0.05)
    p_metrics = step_metrics(p_result["times"], p_result["outputs"], 1.0)
    pi_metrics = step_metrics(pi_result["times"], pi_result["outputs"], 1.0)
    assert p_metrics["steady_state_error"] > 0.1
    assert abs(pi_metrics["steady_state_error"]) < abs(p_metrics["steady_state_error"])


def test_metrics_on_settling_response():
    plant = Plant(kind="first_order", gain=1.0, tau=1.0)
    controller = PIDController("PI", kp=2.0, ki=1.5, u_min=-2.0, u_max=2.0)
    traj = simulate(plant, controller, 1.0, 5.0, 0.05)
    metrics = step_metrics(traj["times"], traj["outputs"], 1.0)
    assert metrics["overshoot"] >= 0.0
    assert metrics["rise_time"] is not None
    assert metrics["settling_time"] is not None
    assert metrics["iae"] > 0


def test_simulate_shape_and_dt():
    plant = Plant(kind="first_order", gain=1.0, tau=1.0)
    controller = PIDController("P", kp=1.0)
    traj = simulate(plant, controller, 1.0, 2.0, 0.1)
    assert len(traj["times"]) == 21
    assert traj["times"][-1] == pytest.approx(2.0)
    assert len(traj["outputs"]) == len(traj["controls"]) == len(traj["errors"])


@pytest.mark.parametrize("duration,dt", [(0.0, 0.1), (1.0, 2.0), (-1.0, 0.1)])
def test_invalid_simulation_arguments(duration, dt):
    plant = Plant()
    controller = PIDController("P", kp=1.0)
    with pytest.raises(ValueError):
        simulate(plant, controller, 1.0, duration, dt)


def test_compare_groups_controllers():
    plant = Plant(kind="first_order", gain=1.0, tau=1.0)
    controllers = [PIDController("P", kp=2.0), PIDController("PI", kp=2.0, ki=1.5)]
    result = compare(plant, controllers, 1.0, 5.0, 0.05)
    assert [case["controller"] for case in result["cases"]] == ["P", "PI"]
    assert len(result["cases"][0]["metrics"]) == 6


def test_cli_demo_end_to_end(tmp_path):
    output = tmp_path / "results"
    docs = tmp_path / "data.json"
    assert main(["demo", "--output", str(output), "--docs", str(docs)]) == 0
    report = json.loads((output / "report.json").read_text(encoding="utf-8"))
    assert report["data_kind"] == "synthetic-demo"
    assert len(report["result"]["cases"]) == 3
    assert (output / "comparison.csv").exists()
    assert (output / "trajectories.csv").exists()
    assert (output / "report.md").exists()
    assert json.loads(docs.read_text(encoding="utf-8"))["data_kind"] == "synthetic-demo"


def test_cli_compare_with_config(tmp_path):
    config = {
        "plant": {"kind": "first_order", "gain": 1.0, "tau": 1.0},
        "setpoint": 1.0, "duration": 3.0, "dt": 0.05,
        "controllers": [{"name": "P", "kp": 2.0, "u_min": -2.0, "u_max": 2.0}],
    }
    config_path = tmp_path / "params.json"
    config_path.write_text(json.dumps(config), encoding="utf-8")
    output = tmp_path / "report"
    assert main(["compare", "--config", str(config_path), "--output", str(output)]) == 0
    report = json.loads((output / "report.json").read_text(encoding="utf-8"))
    assert report["result"]["cases"][0]["controller"] == "P"


def test_cli_missing_config_returns_nonzero(tmp_path):
    assert main(["compare", "--config", str(tmp_path / "missing.json"),
                 "--output", str(tmp_path / "out")]) == 2
