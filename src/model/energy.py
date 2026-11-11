import numpy as np


def compute_energy(t, in_detour, z_agl, params_e):
    # отрезок времени
    dt = np.diff(t, prepend = t[0])
    # перевод из КВт в Вт
    p_cruise = params_e["P_cruise_kw"] * 1000.0
    p_detour = params_e["P_detour_kw"] * 1000.0
    p_climb = params_e["P_climb_kw"] * 1000.0
    # определение подъема ЛА
    dz = np.diff(z_agl, prepend=z_agl[0])
    climbing = dz > 0.01
    # заполнение массива мощностей
    P = np.full_like(t, p_cruise)
    P[in_detour] = p_detour
    P[climbing] = p_climb
    # накопленная энергия
    E = np.cumsum(P * dt)
    # емкость
    E_total = params_e["E_total_kwh"] * 3.6e6
    # доля заряда
    soc = np.clip(1.0 - E / E_total, 0.0, 1.0)

    m_empty = params_e["m_empty_kg"]
    m_payload = params_e["m_payload_kg"]
    m_batt = params_e["m_battery_kg"]
    mass_nominal = np.full_like(t, m_empty + m_payload + m_batt)

    drop_time_s = params_e["mass_drop_time_min"] * 60.0
    mass_after_drop = mass_nominal.copy()
    drop_mask = t >= drop_time_s
    mass_after_drop[drop_mask] -= params_e["mass_drop_kg"]
    return P, E, soc, mass_nominal, mass_after_drop, E_total