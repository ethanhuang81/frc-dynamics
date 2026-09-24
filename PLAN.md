# FRC mechanism dynamics: implementation plan

## Product definition

Given CAD-derived mass properties, an actuator/transmission, electrical conditions,
and a required motion, predict motion and loading, then compare physically
buildable designs. The output should identify a useful range of ratios and the
assumptions that change the decision—not imply an exact real-world optimum.

User decisions: Python physics prototype first; use the user's mechanism when
its details arrive. No example motor specifications or mechanism are assumed.

## Corrections to the previous proposal

- A fixed-voltage, constant-parameter DC electrical model produces the familiar
  linear torque-speed curve. Its benefit is exposing current and voltage so that
  limits, battery coupling, and changing commands can be modeled. It does not
  by itself add nonlinear motor fidelity.
- Battery current is supply current, not the sum of motor stator currents.
  For an ideal PWM equivalent, Isupply = duty * Imotor. Signed quantities matter
  during braking. CTRE documents the motoring relationship and separate limits:
  https://pro.docs.ctr-electronics.com/en/stable/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html
- A rigid arm has constant inertia about its fixed pivot axis. Gravity changes
  with orientation. Mesh detail alone does not improve its rigid-body dynamics
  once mass, COM, and inertia are known accurately.
- Gear ratio cannot be optimized independently of motion strategy, braking,
  controller settings, and constraints. First crossing of a position target is
  not completion of a rest-to-rest move.
- MuJoCo currently documents a dcmotor actuator, but its presence does not make
  it a validated Talon FX/Kraken/battery emulator. Validate supported behavior:
  https://mujoco.readthedocs.io/en/latest/XMLreference.html#actuator-dcmotor

## First mechanism and input contract

Start with one rigid assembly on one fixed revolute joint. Keep flexible links,
backlash, contacts, and multibody motion outside the first release. An elevator
and flywheel can follow using the same electrical model.

All computations use SI units and radians. Store source units, CAD revision,
configuration, timestamp, and measured/estimated provenance alongside inputs.

Required mechanism data:

- Mass in kg; COM position in m; full symmetric COM inertia tensor in kg m².
- Tensor coordinate-frame orientation; pivot origin and unit axis in that same
  frame; reference pose corresponding to angle zero; world gravity direction.
- Travel limits, start/target positions, initial velocity, payload configurations.
- Motor model/count, controller operating mode, candidate reductions (defined
  as motor speed / output speed), stator and supply current limits separately.
- Battery open-circuit voltage, effective battery/wiring resistance, other
  robot supply load; estimated friction and transmission losses with ranges.
- Position/velocity settling tolerances and dwell duration; torque, speed,
  acceleration, voltage-floor, and move-time requirements where available.

If CAD exports inertia about a mate connector rather than the COM, record that
explicitly and do not apply the parallel-axis shift a second time. Rotate tensors
into a common frame using Inew = R Iold Rᵀ. Check tensor symmetry, eigenvalues,
principal-moment triangle inequalities, units, and COM/frame consistency.

Manual mass-property entry is the first importer. Add the Onshape API once its
output agrees with manual extraction on known parts. The Onshape mass-property
tool supports selected parts/assemblies and reference mate connectors; unassigned
materials can omit parts from the calculation:
https://cad.onshape.com/help/Content/View/mass_properties_tool.htm

## Physics contract

For a fixed-axis rigid body, with COM offset r and unit pivot axis a:

    Jpivot = aᵀ Icom a + m (r·r - (a·r)²)
    torque_gravity(q) = a · (r(q) × m g)
    Jeff qddot = torque_drive + torque_gravity - torque_losses

All torques are signed along the same axis. Add motor rotor inertia and rotating
transmission components exactly once. Under ideal kinematics, n identical motor
rotors contribute n * Jrotor * G² to output inertia. Unknown rotor inertia should
be an explicit uncertainty rather than silently assumed negligible.

Use an equivalent motor model initially:

    Vterminal = R I + Ke omega_motor
    torque_em = Kt I
    omega_motor = G qdot
    Vbus = Voc - Rbattery (Iother + sum(Isupply))

Specify how motor losses convert electromagnetic torque to shaft torque. Fit
datasheet curves consistently, including no-load current, rather than combining
incompatible loss assumptions. Track FOC/non-FOC curve provenance and units;
do not invent motor constants or treat all current definitions as interchangeable.

The controller adjusts duty/voltage to enforce current limits subject to available
bus voltage. Do not clip calculated current while leaving the terminal voltage
unchanged: that violates the electrical equation. Solve the motor-controller-bus
operating point together at each evaluation, with residual and feasibility checks.
Include supply-limit timing only when the selected controller actually uses it.

For the first baseline, ideal transmission is useful for conservation tests.
Then introduce measured/estimated friction and a power-direction-aware loss model.
Multiplying torque by a single efficiency in every direction can mishandle
backdriving. Avoid counting the same losses in efficiency and friction twice.

Declare braking policy: coast, electrical braking, or active reverse torque.
An ideal reversible battery is only a reference model; production predictions
need bounded regeneration/bus behavior. Thermal transients, controller dynamics,
and structural compliance should be added when data or decisions justify them.

## Architecture

Keep the physics independent of the interface and the dynamics backend:

    Input adapters -> validated mechanism/scenario
    Motion planner/controller -> electrical + transmission model
    Electrical/transmission <-> mechanical dynamics backend
    Time histories -> metrics/constraints -> design sweep -> report

Suggested Python modules:

- models/: immutable bodies, joints, motors, transmissions, scenarios and units.
- cad/: manual JSON input first, Onshape adapter later.
- electrical/: motor equations, controller limits, shared battery solve.
- dynamics/: single-axis backend, later MuJoCo adapter.
- motion/: reference profiles, sampled controller, completion detection.
- analysis/: metrics, feasibility, design enumeration, uncertainty scenarios.
- validation/: analytical cases, convergence, recorded hardware benchmarks.

Use NumPy/SciPy for production numerics and a plotting library for trace reports.
Start with a CLI/notebook; keep any later browser UI a client of this same model.
No dependency installation is required for the included reference kernels.

## Milestones and acceptance criteria

| Stage | Deliverable | Acceptance gate |
|---|---|---|
| 0: reference kernels | Inertia projection, gravity, ideal motor/battery equations | Analytical mechanics and electrical conservation tests |
| 1: forward simulation | Actual user arm, fixed command then sampled closed-loop control; CSV and plots | No-load/stall/hold behavior, pendulum energy, timestep convergence, documented limits |
| 2: motion evaluation | Accelerate, brake, settle, hold; limits and failed-run explanations | Completion requires position AND velocity tolerances continuously for a configured dwell; reject later departures during validation hold |
| 3: design sweep | Candidate ratios, motor counts, profile parameters; feasibility and tradeoff plots | Fair per-design control/profile treatment; no feasible result clearly reported; finalist reruns at tighter numerical tolerance |
| 4: calibration | Log comparison, fitted friction/electrical parameters and residuals | Predict held-out moves in both directions and with different payloads |
| 5: multibody | Pivoting elevator with shared electrical model | Match the single-axis case with extension fixed and an independent two-DOF analytical model when moving |

For stage 1, integrate continuous mechanics with an appropriate SciPy integrator,
but update the robot controller at a separately specified sample period. Handle
limits and events explicitly. Halve timestep/tighten tolerances until changes in
time, peak load, energy, and bus minimum fall below project-selected tolerances.
A small timestep alone is not evidence of accuracy.

Before optimizing, validate against a rod with known inertia, gravity hold
torque, ideal motor operating points, passive pendulum energy, and the applicable
WPILib mechanism simulation. WPILib is a baseline, not independent proof when
it shares the same assumptions:
https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/physics-sim.html

## Optimization that is useful to a mechanical designer

Initially minimize settle time among enumerated feasible candidates. Enumerate
buildable gear/sprocket stages, not just continuous abstract reductions. Record
packaging, per-stage torque/speed ratings, and stock part constraints separately.

Use two comparisons: (1) same deployment control strategy, with stated tuning
rules; (2) best trajectory found within a specified profile family for each ratio.
Neither should be advertised as the global physical minimum. Later, constrained
optimal control can provide a stronger benchmark if it changes decisions.

Show time versus ratio, stator and supply current separately, output torque,
speed, bus voltage, energy, I²R loss, settling, and each binding constraint.
Energy should distinguish draw, regeneration, and net flow. Estimate temperatures
only after adding a parameterized thermal model; I²t alone is not temperature.

Run finalists across payload, friction, battery resistance/charge state, and other
robot-load scenarios. Prefer a broad region of good performance if small input
changes reverse the ranking. Do not imply confidence intervals without a basis
for the parameter distributions. Repeat duty cycles to evaluate sustained use.

## V2: moving elevator on a pivot

Both releases can use rigid bodies: V2 introduces multiple bodies and relative
motion, not necessarily flexible materials. Separate the moving carriage/stages
from the pivot structure. Do not aggregate a moving assembly into one tensor.

Represent each body with inertial properties and joint transforms. Reuse the
validated motor, controller, battery, losses, scenarios and reporting. Let MuJoCo
compute multibody motion from applied joint forces/torques; avoid duplicating
electrical dynamics, damping or reflected inertia across layers. Inspect whether
its built-in motor actuator matches our requirements before choosing it.

Use a two-coordinate analytical pivot/slider model to validate gravity, centrifugal
and velocity-coupling terms. With extension locked, V2 must reproduce V1. This
isolates backend errors from actuator-model errors before adding contacts.

## What exists now

physics.py contains three reference operations: arbitrary-axis COM inertia
projection, orientation-dependent gravity torque, and a self-consistent ideal
fixed-duty motor/battery operating point. test_physics.py checks analytical
limits and conservation. There is no forward integrator, current limiter,
motor database, optimizer, CAD connection, or graphical interface yet.

Run from this folder with Python 3.9+:

    python3 -m unittest -v

Next implementation: encode the user's actual arm and motor data, then build the
limited-voltage/current actuator and stage-1 time-domain model. Keep the ideal
reference kernels as regression oracles.
