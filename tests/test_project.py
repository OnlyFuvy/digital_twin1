"""Numerical and end-to-end acceptance tests (stdlib unittest)."""
import copy
import tempfile
import unittest
from pathlib import Path
import numpy as np
from dtwin.config import load_config,load_sensors,validate,scenario
from dtwin.route import build_route,time_parameterize,collision_checks,length
from dtwin.energy import integrate_energy
from dtwin.sensors import simulate_sensors,rotations,body_rates
from dtwin.telemetry import memory_budget,write_stream,read_stream


class MissionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.c=load_config();cls.r=time_parameterize(build_route(cls.c),cls.c)
        cls.energy=integrate_energy(cls.r,cls.c)
        cls.sensors=simulate_sensors(cls.r,cls.c,load_sensors())

    def test_seven_detour_lengths(self):
        self.assertEqual(len(self.r['obstacles']),7)
        for o,p in zip(self.r['obstacles'],[20,4,4,4,4,5,6]):self.assertAlmostEqual(o['actual_percent'],p,places=7)
        self.assertAlmostEqual(self.r['s'][-1]-self.r['baseline_length_m'],9400,places=5)

    def test_vertical_half_current_height(self):
        for o in self.r['obstacles']:
            if o['type']=='over':self.assertEqual(o['peak_agl_m'],90)

    def test_collision_and_canopy(self):
        checks,margin=collision_checks(self.r,self.c)
        self.assertTrue(all(o['passed'] for o in checks));self.assertGreater(margin,25)

    def test_bridge_rejects_insufficient_opening(self):
        c=copy.deepcopy(self.c);c['environment']['bridge_bottom_agl_m']=50
        checks,_=collision_checks(self.r,c)
        self.assertFalse(next(x for x in checks if x['type']=='bridge')['passed'])

    def test_trajectory_geometry_uses_absolute_altitude(self):
        self.assertAlmostEqual(length(np.column_stack([self.r[k] for k in ('x','y','z')])),self.r['s'][-1],places=6)
        self.assertTrue(np.all(np.diff(self.r['t'])>0))
        np.testing.assert_allclose(self.r['z']-self.r['terrain'],self.r['agl'])

    def test_dynamic_limits(self):
        self.assertLessEqual(np.max(np.abs(np.rad2deg(self.r['euler'][:,0]))),25.1)
        self.assertLessEqual(np.abs(self.r['velocity'][:,2]).max(),3.02)
        self.assertLessEqual(self.r['speed'].max(),120/3.6+.001)
        accel=np.gradient(self.r['speed'],self.r['t'])
        self.assertLessEqual(np.max(np.abs(accel)),1.02)

    def test_mass_changes_only_with_fuel(self):
        dry=368
        np.testing.assert_allclose(self.r['mass_kg']-self.r['fuel_kg'],dry)
        self.assertTrue(np.all(np.diff(self.r['fuel_kg'])<=0))

    def test_fuel_energy_conservation(self):
        predicted=self.energy['shaft_energy_kwh']*.32
        self.assertLess(abs(predicted-self.energy['fuel_consumed_kg']),.005)
        self.assertTrue(self.energy['reserve_passed'])

    def test_low_fuel_rejected(self):
        result=integrate_energy(copy.deepcopy(self.r),scenario(self.c,'low_fuel'))
        self.assertFalse(result['reserve_passed'])

    def test_power_limit_violation_detected(self):
        c=copy.deepcopy(self.c);c['vehicle']['max_power_kw']=10
        self.assertFalse(integrate_energy(copy.deepcopy(self.r),c)['power_passed'])

    def test_all_channels_dimensions_and_schedules(self):
        self.assertEqual(len(self.sensors),24)
        for s in self.sensors.values():
            self.assertEqual(s['values'].shape[1],s['spec']['components'])
            self.assertEqual(len(s['t']),int(np.floor(self.r['t'][-1]*s['rate_hz']))+1)
            self.assertTrue(np.all(np.isfinite(s['values'][s['quality']==0])))

    def test_gnss_failure_is_flagged_not_faked(self):
        s=self.sensors['GNSS1'];ix=(s['t']>=300)&(s['t']<360)
        self.assertTrue(np.all(s['quality'][ix]==1));self.assertTrue(np.all(np.isnan(s['values'][ix])))

    def test_derived_baro_uses_primary_measurement(self):
        b=self.sensors['BARO1']['values'][:,0];h=self.sensors['BARO_ALT']['values'][:,0]
        np.testing.assert_allclose(h,44330*(1-(b/101325)**(1/5.255)))

    def test_imu_redundancy_not_identical(self):
        self.assertFalse(np.array_equal(self.sensors['IMU1']['values'],self.sensors['IMU2']['values']))

    def test_rotations_are_proper(self):
        R=rotations(self.r['euler'][::100])
        np.testing.assert_allclose(R@R.transpose(0,2,1),np.broadcast_to(np.eye(3),R.shape),atol=1e-12)
        np.testing.assert_allclose(np.linalg.det(R),1,atol=1e-12)

    def test_exact_memory_includes_redundancy_and_state(self):
        m=memory_budget(10,load_sensors(),self.c)
        self.assertEqual(len(m['streams']),25)
        imu=next(x for x in m['streams'] if x['id']=='IMU2')
        self.assertEqual(imu['total_bytes'],2001*40)
        self.assertEqual(m['binary_bytes'],sum(x['total_bytes'] for x in m['streams']))

    def test_binary_roundtrip_and_truncation(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'s.bin';values=np.array([[1,2,3],[4,5,6.]])
            size=write_stream(path,2,[0,.1],values,[0,1]);self.assertEqual(size,56)
            a=read_stream(path,3);np.testing.assert_allclose(a['values'],values)
            path.write_bytes(path.read_bytes()[:-1])
            with self.assertRaises(ValueError):read_stream(path,3)

    def test_config_validation(self):
        for field,value in [('baseline_m',-1),('spatial_step_m',float('nan')),('half_width_m',4000)]:
            c=copy.deepcopy(self.c);c['route'][field]=value
            with self.assertRaises(ValueError):validate(c)

    def test_overweight_rejected(self):
        c=copy.deepcopy(self.c);c['vehicle']['payload_kg']=1000
        with self.assertRaises(ValueError):validate(c)

    def test_resolution_convergence(self):
        c=copy.deepcopy(self.c);c['route']['spatial_step_m']=1
        fine=time_parameterize(build_route(c),c)
        self.assertLess(abs(fine['s'][-1]-self.r['s'][-1]),.02)
        self.assertLess(abs(fine['t'][-1]-self.r['t'][-1]),1)


if __name__=='__main__':unittest.main(verbosity=2)
