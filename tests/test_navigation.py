import copy
import unittest
import numpy as np
from dtwin.navigation import navigate
from dtwin.config import load_config,load_sensors
from dtwin.route import build_route,time_parameterize
from dtwin.energy import integrate_energy
from dtwin.sensors import simulate_sensors


class NavigationTests(unittest.TestCase):
    def test_stationary_zero_noise_does_not_drift(self):
        c=load_config();t=np.arange(2001)/200
        r=dict(t=t,x=t*0,y=t*0,z=t*0+100,velocity=np.zeros((len(t),3)),euler=np.zeros((len(t),3)))
        imu=dict(t=t,values=np.tile([0,0,9.80665,0,0,0],(len(t),1)))
        gt=np.arange(201)/20
        gnss=dict(t=gt,values=np.tile([0,0,100],(len(gt),1)),quality=np.zeros(len(gt)),spec=dict(x=0,y=0,z=0))
        result=navigate(r,dict(IMU1=imu,GNSS1=gnss),c)
        self.assertLess(result['metrics']['max_error_m'],1e-10)
        self.assertLess(result['metrics']['pure_final_error_m'],1e-10)

    def test_constant_velocity_without_gnss(self):
        c=load_config();t=np.arange(2001)/200
        r=dict(t=t,x=t*10,y=t*0,z=t*0+100,velocity=np.tile([10,0,0],(len(t),1)),euler=np.zeros((len(t),3)))
        imu=dict(t=t,values=np.tile([0,0,9.80665,0,0,0],(len(t),1)))
        gnss=dict(t=np.array([0.]),values=np.array([[0,0,100]]),quality=np.ones(1),spec=dict(x=0,y=0,z=0))
        result=navigate(r,dict(IMU1=imu,GNSS1=gnss),c)
        self.assertLess(result['metrics']['max_error_m'],1e-8)

    def test_sensor_rates_scale_with_speed(self):
        c=load_config();c['vehicle']['cruise_ms']=50
        s=load_sensors(c)
        self.assertEqual(next(x for x in s if x['id']=='GNSS1')['rate_hz'],25)
        self.assertGreaterEqual(next(x for x in s if x['id']=='LIDAR_DOWN')['rate_hz'],67)

    def test_noise_is_reproducible(self):
        c=load_config();r=time_parameterize(build_route(c),c);integrate_energy(r,c)
        s=load_sensors(c)[:1]
        a=simulate_sensors(r,c,s)['IMU1']['values'];b=simulate_sensors(r,c,s)['IMU1']['values']
        np.testing.assert_array_equal(a,b)


if __name__=='__main__':unittest.main()
