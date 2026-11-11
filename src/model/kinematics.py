import numpy as np

def compute_kinematics(x, y, z_agl, params_scn, v_cruise_ms):
    # приращения координат
    dx = np.diff(x)
    dy = np.diff(y)
    dz = np.diff(z_agl)
    # евклидово расстояние
    ds = np.sqrt(dx * dx + dy * dy + dz * dz)
    # накопленный путь
    s = np.concatenate([[0.0], np.cumsum(ds)])
    # массив скоростей
    v = np.full_like(s, v_cruise_ms)
    # определение участков облета
    half_w = params_scn["maneuver_half_width_m"]
    factor = params_scn["detour_speed_factor"]
    in_detour = np.zeros_like(s, dtype=bool)
    for ob in params_scn["obstacles"]:
        center = ob["x_km"] * 1000.0
        in_detour |= (x >= center - half_w) & (x <= center + half_w)
    # снижение скорости на облетах
    v[in_detour] = v_cruise_ms * factor
    # время на сегменте
    dt = np.zeros_like(s)
    dt[1:] = ds / v[1:]
    # накопленное время
    t = np.cumsum(dt)
    return s, v, t, in_detour