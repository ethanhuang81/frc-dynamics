"""Analytical fixtures only; these are not a proposed FRC mechanism."""
import math
import unittest
from physics import pivot_inertia, gravity_torque, fixed_duty_point


class PhysicsTests(unittest.TestCase):
    def test_uniform_rod_about_end(self):
        m, length = 3.0, 0.8
        j = m*length**2/12
        self.assertAlmostEqual(pivot_inertia(m, ((0,0,0),(0,j,0),(0,0,j)),
                                            (length/2,0,0), (0,0,1)), m*length**2/3)

    def test_offset_parallel_to_axis_does_not_change_inertia(self):
        self.assertAlmostEqual(pivot_inertia(2, ((1,0,0),(0,1,0),(0,0,1)),
                                            (0,0,100), (0,0,5)), 1)

    def test_off_diagonal_tensor_projection(self):
        self.assertAlmostEqual(pivot_inertia(1, ((2,1,0),(1,2,0),(0,0,2)),
                                            (0,0,0), (1,1,0)), 3)

    def test_gravity_sign_and_vertical_zero(self):
        self.assertAlmostEqual(gravity_torque(2,(0.5,0,0),(0,0,1),0), -9.80665)
        self.assertAlmostEqual(gravity_torque(2,(0.5,0,0),(0,0,1),math.pi/2), 0)

    def point(self, **overrides):
        args = dict(duty=0.5, motor_speed=100, resistance=0.1, ke=0.02,
                    count=2, open_circuit_voltage=12, battery_resistance=0.02,
                    other_supply_current=10)
        args.update(overrides)
        return fixed_duty_point(**args)

    def test_battery_kirchhoff_and_power_balance(self):
        p = self.point()
        self.assertAlmostEqual(p.bus_voltage, 12-0.02*p.battery_current_total)
        self.assertAlmostEqual(p.bus_voltage*p.motor_supply_current_total,
                               2*p.terminal_voltage*p.current_per_motor)
        self.assertAlmostEqual(p.terminal_voltage, 0.1*p.current_per_motor+2)

    def test_ideal_supply(self):
        self.assertEqual(self.point(battery_resistance=0).bus_voltage, 12)

    def test_regeneration_is_signed(self):
        p = self.point(motor_speed=500, other_supply_current=0)
        self.assertLess(p.battery_current_total, 0)
        self.assertGreater(p.bus_voltage, 12)

    def test_invalid_inputs(self):
        with self.assertRaises(ValueError):
            self.point(duty=2)
        with self.assertRaises(ValueError):
            pivot_inertia(1, ((1,0,0),(0,1,0),(0,0,1)), (0,0,0), (0,0,0))


if __name__ == '__main__':
    unittest.main()