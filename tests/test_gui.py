import unittest
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt
from dtwin.config import load_config,load_sensors
from dtwin.route import build_route,time_parameterize
from dtwin.energy import integrate_energy
from dtwin.sensors import simulate_sensors
from dtwin.gui import create_dashboard


class GuiTests(unittest.TestCase):
    def test_timeline_and_channel_callbacks(self):
        c=load_config();r=time_parameterize(build_route(c),c);integrate_energy(r,c)
        s=simulate_sensors(r,c,load_sensors(c));fig=create_dashboard(r,s,{})
        slider,_,_,radio,_,update,select=fig._twin_controls
        slider.set_val(325);radio.set_active(1);fig.canvas.draw()
        self.assertEqual(slider.val,325)
        self.assertEqual(radio.value_selected,'FUEL_FLOW')
        for text in fig.texts:
            box=text.get_window_extent(fig.canvas.get_renderer())
            self.assertGreaterEqual(box.y0,0)
            self.assertLessEqual(box.y1,fig.bbox.height)
        plt.close(fig)


if __name__=='__main__':unittest.main()
