import csv
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import matplotlib
matplotlib.use('Agg')
import numpy as np
from matplotlib import pyplot as plt
from dtwin.config import load_config, load_sensors
from dtwin.route import build_route, time_parameterize
from dtwin.energy import integrate_energy
from dtwin.sensors import simulate_sensors
from dtwin.gui import create_dashboard, replay


class GuiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        c = load_config()
        cls.r = time_parameterize(build_route(c), c)
        integrate_energy(cls.r, c)
        cls.streams = simulate_sensors(cls.r, c, load_sensors(c))

    def tearDown(self):
        plt.close('all')

    def test_all_four_signals_are_visible_immediately(self):
        fig = create_dashboard(self.r, self.streams, {})
        axes = fig._twin_axes['signals']
        self.assertEqual(set(axes), {'IMU1', 'FUEL_FLOW', 'LIDAR_DOWN', 'TEMP_BATT'})
        self.assertTrue(all(ax.name != '3d' for ax in fig.axes))
        for sid, ax in axes.items():
            self.assertTrue(ax.get_visible())
            stream = self.streams[sid]
            step = max(1, int(stream['rate_hz'] / 5))
            np.testing.assert_array_equal(
                ax.lines[0].get_ydata(), np.where(stream['quality'][::step] == 0, stream['values'][::step, 0], np.nan)
                * (3600 if sid == 'FUEL_FLOW' else 1))

    def test_timeline_moves_map_and_every_chart(self):
        fig = create_dashboard(self.r, self.streams, {})
        slider = fig._twin_controls[0]
        for t in (325, slider.valmax, 0):
            slider.set_val(t)
            axes = fig._twin_axes
            for ax in (axes['profile'], axes['fuel'], *axes['signals'].values()):
                np.testing.assert_allclose(ax.lines[-1].get_xdata(), [t / 60] * 2)
            index = min(np.searchsorted(self.r['t'], t), len(self.r['t']) - 1)
            marker = axes['route'].lines[-1]
            np.testing.assert_allclose(marker.get_xdata(), [self.r['x'][index] / 1000])
            np.testing.assert_allclose(marker.get_ydata(), [self.r['y'][index] / 1000])

    def test_missing_signals_keep_their_panels(self):
        streams = dict(self.streams)
        streams.pop('IMU1')
        streams['FUEL_FLOW'] = dict(
            t=np.array([]), values=np.empty((0, 1)), rate_hz=.2)
        fig = create_dashboard(self.r, streams, {})
        for sid in ('IMU1', 'FUEL_FLOW'):
            ax = fig._twin_axes['signals'][sid]
            self.assertEqual(ax.texts[0].get_text(), 'Нет записанных данных')

    def test_play_pause_reset_and_end_of_playback(self):
        fig=create_dashboard(self.r,self.streams,{})
        slider,button,reset,timer,_=fig._twin_controls
        tick,args,kwargs=timer.callbacks[0]
        tick(*args,**kwargs)
        self.assertEqual(slider.val,0)
        button._observers.process('clicked',None)
        tick(*args,**kwargs)
        self.assertEqual(slider.val,3)
        button._observers.process('clicked',None)
        tick(*args,**kwargs)
        self.assertEqual(slider.val,3)
        reset._observers.process('clicked',None)
        tick(*args,**kwargs)
        self.assertEqual(slider.val,0)
        slider.set_val(slider.valmax-1)
        button._observers.process('clicked',None)
        tick(*args,**kwargs)
        self.assertEqual(slider.val,slider.valmax)
        slider.set_val(0)
        tick(*args,**kwargs)
        self.assertEqual(slider.val,0)

    def test_summary_distinguishes_route_fuel_from_after_landing(self):
        r=dict(self.r)
        energy=integrate_energy(r,load_config())
        fig=create_dashboard(r,self.streams,dict(energy=energy,passed=True))
        summary=fig.texts[1].get_text()
        self.assertIn(f"после посадки {energy['final_fuel_kg']:.2f} кг",summary)
        self.assertIn('резерв ≥0.30 кг',summary)
        fig.canvas.draw()
        box=fig.texts[1].get_window_extent(fig.canvas.get_renderer())
        self.assertLessEqual(box.x1,fig.bbox.width)

    def test_dashboard_text_stays_inside_window(self):
        fig = create_dashboard(self.r, self.streams, {})
        fig.canvas.draw()
        renderer = fig.canvas.get_renderer()
        for text in [*fig.texts, *[t for ax in fig.axes for t in ax.texts]]:
            box = text.get_window_extent(renderer)
            self.assertGreaterEqual(box.x0, 0)
            self.assertLessEqual(box.x1, fig.bbox.width)
            self.assertGreaterEqual(box.y0, 0)
            self.assertLessEqual(box.y1, fig.bbox.height)

    def test_replay_loads_all_signals_and_handles_absent_samples(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            report = {'route': {'obstacles': self.r['obstacles']}}
            (directory / 'results.json').write_text(json.dumps(report), encoding='utf-8')
            columns = [
                ('t', 't_s'), ('x', 'x_m'), ('y', 'y_m'), ('z', 'z_amsl_m'),
                ('agl', 'agl_m'), ('speed', 'speed_ms'), ('fuel_kg', 'fuel_kg'),
                ('power_kw', 'power_kw')]
            np.savetxt(directory / 'trajectory.csv',
                       np.column_stack([self.r[key] for key, _ in columns]),
                       delimiter=',', header=','.join(col for _, col in columns),
                       comments='')
            samples = directory / 'sensor_samples.csv'
            with samples.open('w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(['channel', 't_s', 'quality', 'v1'])
                for sid in ('IMU1', 'FUEL_FLOW', 'LIDAR_DOWN', 'TEMP_BATT'):
                    writer.writerows([[sid, 0, 0, 1], [sid, 5, 0, 2]])
            with patch('dtwin.gui.plt.show'):
                replay(directory)
                for sid, ax in plt.gcf()._twin_axes['signals'].items():
                    expected = [3600, 7200] if sid == 'FUEL_FLOW' else [1, 2]
                    np.testing.assert_array_equal(ax.lines[0].get_ydata(), expected)
                plt.close('all')
                samples.unlink()
                replay(directory)
                for ax in plt.gcf()._twin_axes['signals'].values():
                    self.assertEqual(ax.texts[0].get_text(), 'Нет записанных данных')


if __name__ == '__main__':
    unittest.main()
