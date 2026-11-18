"""Проверки физического баланса топлива, подбора заправки и ограничений бака."""
import copy
import unittest

import numpy as np

from dtwin.config import load_config,scenario,validate
from dtwin.energy import integrate_energy,minimum_fuel_for_route
from dtwin.route import build_route,time_parameterize


class FuelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg=load_config()
        cls.route=time_parameterize(build_route(cls.cfg),cls.cfg)

    def test_ten_kg_passes_fuel_in_both_winds_but_not_stress_power(self):
        for name in ('nominal','strong_wind'):
            with self.subTest(scenario=name):
                c=scenario(self.cfg,name);c['vehicle']['fuel_kg']=10.
                e=integrate_energy(dict(self.route),c)
                self.assertTrue(e['reserve_passed'])
                self.assertEqual(e['power_passed'],name=='nominal')

    def test_shortage_keeps_tank_and_mass_physical(self):
        c=scenario(self.cfg,'low_fuel');r=dict(self.route)
        e=integrate_energy(r,c)
        self.assertFalse(e['reserve_passed'])
        self.assertTrue(np.all(r['fuel_kg']>=0))
        self.assertEqual(e['final_fuel_kg'],0.)
        self.assertEqual(e['fuel_consumed_kg'],1.)
        self.assertGreater(e['fuel_shortfall_kg'],0)
        self.assertAlmostEqual(e['fuel_required_kg'],e['fuel_consumed_kg']+e['fuel_shortfall_kg'])
        np.testing.assert_allclose(r['mass_kg']-r['fuel_kg'],368.)
        self.assertAlmostEqual(e['final_mass_kg'],368.)

    def test_required_fuel_matches_integrated_shaft_energy(self):
        for name in ('nominal','low_fuel','strong_wind'):
            c=scenario(self.cfg,name)
            e=integrate_energy(dict(self.route),c)
            self.assertAlmostEqual(e['fuel_required_kg'],e['shaft_energy_kwh']*c['energy']['bsfc_kg_kwh'],places=10)

    def test_minimum_matches_independent_constant_power_solution(self):
        c=copy.deepcopy(self.cfg)
        # Предельно большой винт устраняет индуктивную составляющую в эталоне.
        # При нулевой скорости и наборе остаётся постоянная профильная мощность.
        c['vehicle']['rotor_radius_m']=1e15
        c['environment']['wind_ms']=[0.,0.,0.]
        c['energy']['transmission_efficiency']=1.
        r=dict(t=np.array([0.,3600.]),z=np.array([100.,100.]),
               velocity=np.zeros((2,3)),euler=np.zeros((2,3)))
        required=10.*(3600.+120.)/3600.*.32
        expected=required/(1.-.03)
        result=minimum_fuel_for_route(r,c)
        self.assertTrue(result['feasible'])
        self.assertLessEqual(abs(result['minimum_initial_fuel_kg']-expected),result['tolerance_kg'])
        self.assertGreaterEqual(result['final_fuel_kg'],result['reserve_kg'])

    def test_minimum_is_a_boundary_and_does_not_change_inputs(self):
        c=copy.deepcopy(self.cfg);r=dict(self.route)
        before=c['vehicle']['fuel_kg'];keys=set(r)
        plan=minimum_fuel_for_route(r,c)
        self.assertEqual(c['vehicle']['fuel_kg'],before)
        self.assertEqual(set(r),keys)
        for offset,passed in ((.001,True),(-.001,False)):
            c['vehicle']['fuel_kg']=plan['minimum_initial_fuel_kg']+offset
            self.assertEqual(integrate_energy(dict(r),c)['reserve_passed'],passed)

    def test_tank_and_density_validation(self):
        for key,value in (('fuel_density_kg_l',0),('fuel_density_kg_l',float('nan')),
                          ('tank_capacity_l',-1),('tank_capacity_l','неизвестно'),
                          ('tank_capacity_l',10),('transmission_efficiency',1.1)):
            with self.subTest(key=key,value=value):
                c=copy.deepcopy(self.cfg);c['energy'][key]=value
                with self.assertRaises(ValueError):validate(c)
        c=copy.deepcopy(self.cfg);c['energy']['tank_capacity_l']=14.
        validate(c)
        e=integrate_energy(dict(self.route),c)
        self.assertTrue(e['tank_capacity_passed'])

    def test_unknown_tank_is_not_reported_as_verified(self):
        e=integrate_energy(dict(self.route),self.cfg)
        self.assertFalse(e['tank_capacity_checked'])
        self.assertIsNone(e['tank_capacity_passed'])
        self.assertAlmostEqual(e['initial_fuel_l'],10./.75)

    def test_minimum_detects_insufficient_tank(self):
        c=copy.deepcopy(self.cfg);c['energy']['tank_capacity_l']=1.
        self.assertFalse(minimum_fuel_for_route(self.route,c)['feasible'])


if __name__=='__main__':unittest.main()
