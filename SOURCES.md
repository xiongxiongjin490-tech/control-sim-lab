# Sources and provenance

1. Classical linear control concepts: first-order inertia plant, second-order
   standard form, P/PI/PID three-term control, step-response metrics (rise time,
   settling time, overshoot, steady-state error, IAE). The discrete simulation and
   metric code here is an independent standard-library implementation.
2. Forward Euler integration is used for the discrete-time update; the fixed
   step size is reported in every report. Euler discretization is a study
   approximation, not a high-fidelity plant simulation.
3. Python standard library documentation: [math](https://docs.python.org/3/library/math.html),
   [csv](https://docs.python.org/3/library/csv.html), [json](https://docs.python.org/3/library/json.html)
   and [platform](https://docs.python.org/3/library/platform.html).

The demo cases in `control_sim_lab/synthetic.py` use a fixed first-order plant under
label `synthetic-demo`. Controller gains are illustrative and do not come from any real
motor, thermal loop, process unit, or industrial controller.

Files in `results/` can be rebuilt with `python -m control_sim_lab demo`. No original
engineer affiliation, field commissioning record, or award is attributed to this account.
