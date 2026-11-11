import numpy as np
# рельеф
# гауссов холм (моделирования препятствия для облета)
def terrain_height(x_m, params):
    a = params["peak_amplitude_m"]
    c = params["center_x_m"]
    s = params["sigma_m"]
    return a * np.exp(-((x_m - c) / s) ** 2)