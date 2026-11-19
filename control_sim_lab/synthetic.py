"""Fixed demonstration plant and controller set.

The plant is an artificial linear model; the parameters are illustrative and do
not describe any real machine, thermal system, motor, or industrial loop.
"""

from __future__ import annotations

from .core import PIDController, Plant


def demo_plant() -> Plant:
    return Plant(kind="first_order", gain=1.0, tau=1.0)


def demo_controllers() -> list[PIDController]:
    limit = {"u_min": -2.0, "u_max": 2.0}
    return [
        PIDController("P", kp=2.0, **limit),
        PIDController("PI", kp=2.0, ki=1.5, **limit),
        PIDController("PID", kp=2.0, ki=1.5, kd=0.4, **limit),
    ]


DEMO_SETPOINT = 1.0
DEMO_DURATION = 5.0
DEMO_DT = 0.05
