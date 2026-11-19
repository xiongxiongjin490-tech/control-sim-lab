# Control Sim Lab · PID step-response sandbox

[中文](README.md) · [Sources](SOURCES.md)

Simulate first-/second-order linear plants and P/PI/PID controllers in discrete time,
then report overshoot, steady-state error, rise time, settling time and IAE side by side.
The project is implemented independently and bundles no real plant model, hardware
driver, or field-tuned parameter set.

**The bundled demo uses an artificial plant and illustrative gains. They are not
measurements from any real motor, thermal system, or industrial loop.** Metrics and full
trajectories are in [results/](results/).

## Features

- First-order inertia plant `τ·dy/dt + y = K·u` and standard second-order plant,
  integrated with forward Euler steps.
- P/PI/PID three-term controller with output saturation and conditional integral
  anti-windup.
- Step-response metrics: overshoot, steady-state error, 10%→90% rise time, ±5% settling
  time, IAE.
- Exports a JSON report, controller comparison CSV, full trajectory CSV and Markdown
  report, with the runtime environment recorded.

## Quick start

Python 3.10+. Runtime depends on the standard library only; pytest is needed for tests.

```bash
python -m pip install -e ".[dev]"
python -m control_sim_lab demo
python -m pytest
```

Rebuild `results/` and `docs/data.json` by default.

## Comparing your own configuration

```bash
python -m control_sim_lab compare --config params.json --output my-report
```

The config provides a plant, setpoint, duration, step size and a list of controllers.
A second-order plant uses `{"kind": "second_order", "omega_n": 1.0, "zeta": 1.0, "gain": 1.0}`.
External input defaults to `external-unverified`; the source label is a user declaration,
not an authentication. The CLI exits with code 2 on invalid input.

## Interpreting results

`report.md` is the entry point; `comparison.csv` keeps per-controller metrics,
`trajectories.csv` keeps per-timestep output, control and error. Settling time is the
last time the response leaves the ±5% band; if it has not entered the band by the end of
the simulation, settling time is recorded as the simulation duration. Euler discretization
is a study approximation; larger step sizes increase numerical error. Metrics are comparable
only under the same plant, step size and setpoint convention.

## Boundaries

This is an independent discrete simulation and metrics utility inspired by classical
control teaching and scheme comparison. It does not reproduce any real-machine control
pipeline and is not tied to a specific actuator or sensor. See [SOURCES.md](SOURCES.md).
Code is MIT licensed; cited materials retain their original rights.

## Reproducible environment

Python 3.12. The lock file pins development, build and cross-platform runtime dependencies:

```bash
python -m venv .venv
# Windows: .venv\Scripts\activate
# macOS / Linux: source .venv/bin/activate
python -m pip install -r requirements.lock
python -m pip install --no-build-isolation --no-deps -e .
python -m pytest -q
python -m control_sim_lab demo
```
