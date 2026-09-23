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
