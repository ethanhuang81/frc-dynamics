"""Small reference kernels. SI units; no external dependencies.

This is a foundation, not yet a mechanism simulator or motor-controller emulator.
All vectors and tensors supplied to a function must use the same coordinate frame.
"""
from dataclasses import dataclass
from math import cos, sin, sqrt, isfinite


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2],
            a[0]*b[1]-a[1]*b[0])


def vector(v):
    if len(v) != 3 or not all(isfinite(x) for x in v):
        raise ValueError('Expected a finite 3-vector')
    return tuple(v)


def unit(v):
    v = vector(v)
    n = sqrt(dot(v, v))
    if n == 0:
        raise ValueError('Axis cannot be zero')
    return tuple(x/n for x in v)


def pivot_inertia(mass, inertia_com, com_from_pivot, axis):
    """Project a COM inertia tensor onto a fixed pivot axis.

    Caller must supply a physically valid tensor ABOUT THE COM, expressed in
    the same frame as the COM offset and axis. Do not shift pivot inertia twice.
    Full physical realizability/eigenvalue validation belongs in the importer.
    """
    if not isfinite(mass) or mass <= 0:
        raise ValueError('Mass must be positive')
    a, r = unit(axis), vector(com_from_pivot)
    if len(inertia_com) != 3:
        raise ValueError('Expected a 3x3 tensor')
    rows = [vector(row) for row in inertia_com]
    if any(abs(rows[i][j]-rows[j][i]) > 1e-10 for i in range(3) for j in range(3)):
        raise ValueError('Tensor must be symmetric')
    result = dot(a, [dot(row, a) for row in rows]) + mass * dot(cross(r, a), cross(r, a))
    if result <= 0:
        raise ValueError('Projected inertia must be positive')
    return result


def gravity_torque(mass, com_from_pivot_at_zero, axis, angle,
                   gravity=(0.0, -9.80665, 0.0)):
    """Signed torque along world-fixed axis; angle uses the right-hand rule."""
    if not isfinite(mass) or mass <= 0 or not isfinite(angle):
        raise ValueError('Expected positive mass and finite angle')
    a, r, g = unit(axis), vector(com_from_pivot_at_zero), vector(gravity)
    axr = cross(a, r)
    rotated = tuple(r[i]*cos(angle) + axr[i]*sin(angle)
                    + a[i]*dot(a, r)*(1-cos(angle)) for i in range(3))
    return dot(a, cross(rotated, tuple(mass*x for x in g)))


@dataclass(frozen=True)
class ElectricalPoint:
    bus_voltage: float
    terminal_voltage: float
    current_per_motor: float
    motor_supply_current_total: float
    battery_current_total: float


def fixed_duty_point(*, duty, motor_speed, resistance, ke, count,
                     open_circuit_voltage, battery_resistance, other_supply_current=0.0):
    """Exact algebraic solution for identical motors on one ideal PWM bus.

    Vmotor=duty*Vbus; I=(Vmotor-ke*speed)/R; Isupply=count*duty*I.
    Battery is an ideal reversible Thevenin source, including regeneration.
    No current limiting, inductance, controller loss, battery charge limits,
    FOC saturation/field weakening, brownout behavior, or bus clamping yet.
    Signed current is an equivalent DC quantity, not a phase-current model.
    """
    values = (duty, motor_speed, resistance, ke, open_circuit_voltage,
              battery_resistance, other_supply_current)
    if not all(isfinite(v) for v in values):
        raise ValueError('Inputs must be finite')
    if not -1 <= duty <= 1 or resistance <= 0 or ke <= 0:
        raise ValueError('Invalid duty or motor parameters')
    if type(count) is not int or count < 1:
        raise ValueError('Motor count must be a positive integer')
    if open_circuit_voltage <= 0 or battery_resistance < 0 or other_supply_current < 0:
        raise ValueError('Invalid battery/load parameters')
    bus = (open_circuit_voltage - battery_resistance*other_supply_current
           + battery_resistance*count*duty*ke*motor_speed/resistance) / (
               1 + battery_resistance*count*duty*duty/resistance)
    if bus <= 0:
        raise ValueError('Bus collapsed; this model has no brownout dynamics')
    terminal = duty*bus
    current = (terminal-ke*motor_speed)/resistance
    supply = count*duty*current
    return ElectricalPoint(bus, terminal, current, supply, supply+other_supply_current)
