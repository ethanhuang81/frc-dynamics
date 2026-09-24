import copy
import json
import math
from pathlib import Path
import unittest
from calculator import Model, profile, simulate, feasible_profile, LB, INCH
from physics import fixed_duty_point


class CalculatorTests(unittest.TestCase):
    def setUp(self):
        self.cfg = json.loads(Path(__file__).with_name('arm.json').read_text())
        self.m = Model(self.cfg)

    def test_user_parallel_axis_conversion(self):
        self.assertAlmostEqual(self.m.j_arm, 7.7109365*LB*INCH**2)
        self.assertAlmostEqual(self.m.mass, 0.104467539123255)
        self.assertAlmostEqual(self.m.gravity_load(math.pi/2), 0)

    def test_motor_stall_and_no_load(self):
        m = self.m
        self.assertAlmostEqual(m.kt*12/m.r, 4.05)
        current = (12-m.ke*m.free_speed)/m.r
        self.assertAlmostEqual(current, 1.4)
        self.assertAlmostEqual(m.kt*current-m.motor_b*m.free_speed, 0)

    def test_electrical_against_independent_fixed_duty_solution(self):
        m = self.m
        for speed in (-400,0,400):
            for current in (-80,-5,0,5,80):
                p = m.electrical(current,speed)
                if m.violations(p): continue
                oracle = fixed_duty_point(duty=p['duty'],motor_speed=speed,
                    resistance=m.r,ke=m.ke,count=m.n,open_circuit_voltage=m.c['battery_v'],
                    battery_resistance=m.rb,other_supply_current=m.c['other_supply_current_a'])
                self.assertAlmostEqual(oracle.bus_voltage,p['bus_v'])
                self.assertAlmostEqual(oracle.current_per_motor,current)
                self.assertAlmostEqual(p['bus_v'],m.c['battery_v']-m.rb*p['battery_a'])

    def test_current_limit_and_voltage_equations(self):
        m = self.m
        for speed in (-500,0,500):
            for request in (-1000,-300,-100,0,100,300,1000):
                current,p = m.limited_current(request,speed)
                self.assertFalse(m.violations(p))
                self.assertAlmostEqual(p['terminal_v'],m.r*current+m.ke*speed)
                self.assertLessEqual(abs(current),abs(request)+1e-6)

    def test_profile_boundary_conditions(self):
        self.assertEqual(profile(0,1,0,math.pi/2),(0,0,0))
        self.assertEqual(profile(1,1,0,math.pi/2),(math.pi/2,0,0))
        self.assertEqual(profile(2,1,0,math.pi/2),(math.pi/2,0,0))

    def test_gravity_hold(self):
        cfg = copy.deepcopy(self.cfg)
        cfg['target_deg'] = 0
        rows,s = simulate(Model(cfg),10,0.1)
        self.assertEqual(s['status'],'completed')
        self.assertLess(max(abs(r['angle_deg']) for r in rows),1e-10)

    def test_forward_tracking_and_integrator_convergence(self):
        self.assertTrue(feasible_profile(self.m,10,0.06,2000))
        rows,a = simulate(self.m,10,0.06)
        _,b = simulate(self.m,10,0.06,self.cfg['sample_dt_s']/2)
        self.assertEqual(a['status'],'completed')
        self.assertEqual(b['status'],'completed')
        self.assertLess(abs(a['completion_s']-b['completion_s']),0.0003)
        self.assertLess(abs(a['net_j']-b['net_j']),0.03)
        self.assertLess(abs(a['final_error_deg']),0.01)
        for r in rows: self.assertFalse(self.m.violations(r))

    def test_reversed_motion(self):
        cfg = copy.deepcopy(self.cfg)
        cfg.update(start_deg=90,target_deg=0)
        m = Model(cfg)
        self.assertTrue(feasible_profile(m,10,0.2))
        _,s = simulate(m,10,0.2)
        self.assertEqual(s['status'],'completed')

    def test_invalid_input(self):
        for key,value in [('mass_lb',-1),('ratios',[0]),('battery_v',float('nan'))]:
            cfg = copy.deepcopy(self.cfg)
            cfg[key] = value
            with self.assertRaises(ValueError): Model(cfg)


if __name__=='__main__': unittest.main()
