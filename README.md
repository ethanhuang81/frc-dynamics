# FRC rigid-arm calculator

Python 3.9+; no third-party packages, account, CAD connection, or server required.
This prototype uses the supplied tube data for a fixed-axis planar arm.

From this directory:

```sh
python3 calculator.py
python3 -m unittest -v
```

Open `results/report.html` in a browser. Edit `arm.json` and rerun to change
the mechanism, motor, reduction candidates, limits, battery, and controller.
Alternative inputs/output: `python3 calculator.py --config arm.json --output results`.
Each run overwrites matching output files. Old trace files for removed candidates
may remain; `summary.json` and the report identify the current run's candidates.

## Continuous ratio optimization

The calculator now searches decimal ratios instead of limiting the answer to
the `ratios` comparison list. In `arm.json`:

```json
"optimization": {
  "enabled": true,
  "ratio_min": 1.0,
  "ratio_max": 100.0,
  "ratio_tolerance": 0.01,
  "coarse_samples": 21
}
```

It starts with a logarithmic scan, then repeatedly refines detected local valleys
to the ratio tolerance. At each ratio, it brackets a feasible profile duration and
refines that transition to 10 microseconds. The objective is forward-simulated
completion time including settling, with profile duration breaking ties.
Controller sampling quantizes completion time; two decimal places specify a
search/display precision, not real-world accuracy. This is not a joint global
optimization of gearing, trajectory and controller tuning. In particular, a
slower reference profile might settle sooner; that tradeoff is not searched here.

`optimization.csv` records evaluations; `optimized_trace.csv` is the selected
decimal ratio's trace. The report and `summary.json` show the best ratio found.
Bounds are inclusive; a boundary winner means the optimum may lie outside the
searched range. Narrow minima or feasible islands can be missed. Increase
`coarse_samples` to check search sensitivity. Disable optimization to run only
the explicit comparison list. Existing configurations without the new block
continue to run the comparison list only.

## What it computes

- Converts pound-mass/inches to SI and shifts COM inertia to the pivot.
- Generates quintic rest-to-rest profiles, bracketing duration on a geometric
  grid and then refining feasibility. Checks voltage and current feasibility.
- Forward integrates angle and velocity with RK4 under sampled computed-torque
  PD tracking and an ideal instantaneous motor current controller.
- Solves motor voltage, PWM supply current, and battery sag consistently.
- Models gravity, motor viscous drag fitted to no-load current, optional output
  viscous drag, and optional reflected rotor/output inertia.
- Applies stator, positive supply, assumed negative supply, and voltage limits.
- Requires position and speed tolerances through a terminal interval covering
  the configured dwell. Completion includes the dwell, after profile completion.
- Writes CSV traces, full input snapshot and summaries, and an HTML plot report.
- Repeats the selected candidate with half the integration timestep while keeping
  controller update times fixed, and checks its profile at 6,000 subdivisions.

The duration grid can miss narrow feasible regions; there is no proof of global
optimality. These are mathematical ratios, not a catalog of buildable gear stages.
No joint hard stops, structural load limits, or collision constraints are imposed.

## Your supplied arm

Mass: 0.2303115 lb. COM radius: 5 in. COM inertia: 1.953149 lb in².
We assume the COM inertia axis is parallel to the pivot axis and that lb means
pound-mass, as normally used for CAD mass properties.

`Jpivot = 1.953149 + 0.2303115 * 5² = 7.7109365 lb in²`.

Angles are counterclockwise from horizontal, with gravity downward. The COM
offset is added to the joint angle. Positive gravitational load torque is
`m*g*r*cos(angle + offset)` and is subtracted from electromagnetic drive torque.

## Important modeling assumptions

The checked WCP X44 table is explicitly preliminary and specifies trapezoidal
commutation: 7530 RPM, 1.4 A no-load, 4.05 Nm stall, 275 A stall at nominal 12 V.
The configured 279 A stator limit is the user's limit, not the motor's stall-current
specification. R, Kt and Ke are fitted from that table. This approximate empirical
equivalent DC fit does not exactly enforce Kt = Ke in SI. It is not a phase-level
BLDC/FOC model, and its fitted torque/energy predictions need hardware validation.

The user's 40 A supply limit is a strict instantaneous model limit, not a modeled
thermal-breaker trip curve. Controller loss, inductance, delays, actual Talon
firmware limit timing, brownout behavior and field weakening are not modeled.
`sample_dt_s` is the command-update period; the convergence rerun independently
subdivides integration within this period. It is not a claimed Talon update rate.

Battery baseline: 12 V and 12 mΩ, with zero added wiring resistance and no other
robot load. The datasheet resistance is specified at 1 kHz and is not a measured
DC sag parameter for this robot. Edit these values from measurements. 13.3 V is
an optional user scenario, not assumed to be a sustained match voltage.

Braking uses signed current and an ideal reversible battery. Negative supply
current charges that ideal battery, raising bus voltage. The 40 A regeneration
cap is an explicit simulation assumption, NOT a battery charging specification.
Real bus/regen handling must be validated before trusting fastest braking results.
Electrical points with impossible power or back-EMF outside supported voltage
behavior are rejected rather than silently clamped.

Rotor inertia is UNKNOWN and provisionally zero. Gearbox inertia and output
friction are also unknown and set to zero; transmission is ideal. These can
strongly affect rankings for this very light arm. The motor's mounting should be
fixed to the chassis for this model; a motor carried on the arm also contributes
to CAD body mass and inertia. Rotor relative motion must be accounted for properly.
No predicted time here is a validated minimum or a hardware gear recommendation.

`output_em_torque_nm` is electromagnetic torque reflected to the output BEFORE
modeled motor/output damping. `draw_j`, `regen_j`, and `net_j` describe the
mechanism's bus energy, excluding background loads. Regeneration is not heat.

## Sources

- [WCP X44 preliminary motor table](https://docs.wcproducts.com/welcome/electronics/kraken-x44/kraken-x44-motor/overview-and-features/motor-performance)
- [MK ES17-12 manufacturer datasheet hosted by REV](https://www.revrobotics.com/content/docs/ES17-12_User_Guide.pdf)
- [User-selected battery](https://andymark.com/products/mk-es17-12-12v-sla-battery-set-of-2)

PLAN.md preserves the original roadmap. The calculator now implements a first
profile search and forward simulation; CAD importing, calibrated losses,
hardware validation and multibody dynamics remain future milestones.
