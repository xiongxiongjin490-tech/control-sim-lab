"""Discrete PID simulation and step-response metrics, implemented independently.

Plants are simple linear time-invariant models integrated with forward Euler
steps. This is a control-study sandbox, not a physical plant model, hardware
driver, or tuned production controller. No third-party numerical package is used.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass(frozen=True)
class Plant:
    """LTI test plant.

    ``first_order``:  tau * dy/dt + y = gain * u
    ``second_order``: ddy + 2*zeta*omega_n*dy + omega_n^2*y = omega_n^2*gain*u
    """

    kind: str = "first_order"
    gain: float = 1.0
    tau: float = 1.0
    omega_n: float = 1.0
    zeta: float = 1.0

    def __post_init__(self) -> None:
        if self.kind not in ("first_order", "second_order"):
            raise ValueError("Plant kind must be first_order or second_order")
        if not math.isfinite(self.gain):
            raise ValueError("gain must be finite")
        if self.kind == "first_order" and (not math.isfinite(self.tau) or self.tau <= 0):
            raise ValueError("tau must be a positive finite number")
        if self.kind == "second_order":
            if not math.isfinite(self.omega_n) or self.omega_n <= 0:
                raise ValueError("omega_n must be a positive finite number")
            if not math.isfinite(self.zeta) or self.zeta < 0:
                raise ValueError("zeta must be nonnegative and finite")

    def initial_state(self) -> tuple[float, float]:
        return (0.0, 0.0)

    def step(self, state: tuple[float, float], u: float, dt: float) -> tuple[float, float]:
        y, dy = state
        if self.kind == "first_order":
            y_next = y + dt * (self.gain * u - y) / self.tau
            return y_next, 0.0
        ddy = self.omega_n ** 2 * (self.gain * u - y) - 2 * self.zeta * self.omega_n * dy
        dy_next = dy + ddy * dt
        y_next = y + dy_next * dt
        return y_next, dy_next


@dataclass
class PIDController:
    name: str
    kp: float
    ki: float = 0.0
    kd: float = 0.0
    u_min: float = -1e9
    u_max: float = 1e9
    _integral: float = field(default=0.0, repr=False)
    _prev_error: float | None = field(default=None, repr=False)

    def __post_init__(self) -> None:
        for label, value in (("kp", self.kp), ("ki", self.ki), ("kd", self.kd)):
            if not math.isfinite(value):
                raise ValueError(f"{label} must be finite")
        if self.u_min >= self.u_max:
            raise ValueError("Require u_min < u_max")

    def step(self, error: float, dt: float) -> float:
        if not math.isfinite(error) or dt <= 0:
            raise ValueError("error must be finite and dt positive")
        self._integral += error * dt
        derivative = 0.0 if self._prev_error is None else (error - self._prev_error) / dt
        raw = self.kp * error + self.ki * self._integral + self.kd * derivative
        u = min(self.u_max, max(self.u_min, raw))
        # Conditional anti-windup: do not keep the integral that drove saturation.
        if raw != u and self.ki != 0.0:
            self._integral -= error * dt
        self._prev_error = error
        return u


def simulate(plant: Plant, controller: PIDController, setpoint: float,
             duration: float, dt: float) -> dict:
    if not math.isfinite(setpoint):
        raise ValueError("setpoint must be finite")
    if not math.isfinite(duration) or duration <= 0:
        raise ValueError("duration must be positive")
    if not math.isfinite(dt) or dt <= 0:
        raise ValueError("dt must be positive")
    if dt > duration:
        raise ValueError("dt must not exceed duration")

    state = plant.initial_state()
    times: list[float] = []
    outputs: list[float] = []
    controls: list[float] = []
    errors: list[float] = []

    steps = int(round(duration / dt))
    for index in range(steps + 1):
        t = index * dt
        y, _ = state
        error = setpoint - y
        u = controller.step(error, dt) if index > 0 else controller.kp * error
        times.append(t)
        outputs.append(y)
        controls.append(u)
        errors.append(error)
        if index < steps:
            state = plant.step(state, u, dt)

    return {
        "controller": controller.name,
        "setpoint": setpoint,
        "dt": dt,
        "times": times,
        "outputs": outputs,
        "controls": controls,
        "errors": errors,
    }


def step_metrics(times: list[float], outputs: list[float], setpoint: float,
                 band: float = 0.05) -> dict:
    if len(outputs) < 2:
        raise ValueError("Need at least two samples for metrics")
    if not math.isfinite(setpoint) or setpoint == 0:
        raise ValueError("setpoint must be a nonzero finite value for step metrics")
    if not (0 < band < 1):
        raise ValueError("band must be in (0, 1)")

    final_window = max(1, len(outputs) // 10)
    steady_state = sum(outputs[-final_window:]) / final_window
    ss_error = setpoint - steady_state

    peak = max(outputs)
    overshoot = max(0.0, (peak - setpoint) / setpoint) if setpoint > 0 else max(0.0, (setpoint - peak) / setpoint)

    low = 0.1 * setpoint
    high = 0.9 * setpoint
    rise_time: float | None = None
    t_low = next((t for t, y in zip(times, outputs) if y >= low), None)
    t_high = next((t for t, y in zip(times, outputs) if y >= high), None)
    if t_low is not None and t_high is not None:
        rise_time = t_high - t_low

    lower = setpoint * (1 - band)
    upper = setpoint * (1 + band)
    settling_time: float | None = None
    for t, y in zip(times, outputs):
        if y < lower or y > upper:
            settling_time = t
    if settling_time is not None:
        settling_time = times[-1] if settling_time == times[-1] else settling_time

    iae = sum(abs(error) * (times[1] - times[0]) for error in
              (setpoint - y for y in outputs))

    return {
        "steady_state_output": steady_state,
        "steady_state_error": ss_error,
        "overshoot": overshoot,
        "rise_time": rise_time,
        "settling_time": settling_time,
        "iae": iae,
    }


def compare(plant: Plant, controllers: list[PIDController], setpoint: float,
            duration: float, dt: float) -> dict:
    results: list[dict] = []
    for controller in controllers:
        trajectory = simulate(plant, controller, setpoint, duration, dt)
        metrics = step_metrics(trajectory["times"], trajectory["outputs"], setpoint)
        results.append({"controller": controller.name, "metrics": metrics,
                        "final_output": trajectory["outputs"][-1]})
    return {"plant": plant.kind, "setpoint": setpoint, "dt": dt, "cases": results}
