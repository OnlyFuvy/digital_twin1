import numpy as np
# маршрут
from src.model.terrain import terrain_height

# треугольный маневр
# отклонение вбок и возврат обратно
# облет препятствия со стороны
def triangular(x, center, half_width, amplitude):
    dx = np.abs(x - center)
    return np.where(dx <= half_width, amplitude * (1.0 - dx / half_width), 0.0)

# длина сегмента
# евклидово расстояние между точками
def segment_length(x, y, z):
    dx = np.diff(x)
    dy = np.diff(y)
    dz = np.diff(z)
    return np.sqrt(dx * dx + dy * dy + dz * dz)

# амплитуда при которой длина дуги на участке равна заданной
def solve_amplitude(x_seg, base_z_seg, center, half_width, target_len):
    lo, hi = 0.0, 1.0
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        y_seg = triangular(x_seg, center, half_width, mid)
        L = segment_length(x_seg, y_seg, base_z_seg).sum()
        if L < target_len:
            lo = mid
        else:
            break
        hi *= 2.0
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        y_seg = triangular(x_seg, center, half_width, mid)
        L = segment_length(x_seg, y_seg, base_z_seg).sum()
        if L < target_len:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)

# сборка маршрута в единую траекторию
def build_route(params_scn, params_trn, n_points):
    L0 = params_scn["L0_m"]
    x = np.linspace(0.0, L0, n_points)
    y = np.zeros_like(x)
    z_agl = np.full_like(x, params_scn["agl_base_m"])
    half_w = params_scn["maneuver_half_width_m"]
    records = []
    for ob in params_scn["obstacles"]:
        center = ob["x_km"] * 1000.0
        target_extra = L0 * ob["add_pct"] / 100.0
        mask = (x >= center - half_w) & (x <= center + half_w)
        x_seg = x[mask]
        if x_seg.size < 2:
            continue
        if ob["vertical"]:
            z_seg0 = params_scn["agl_base_m"] + 20.0 * (
                1.0 - np.abs(x_seg - center) / half_w
            )
        else:
            z_seg0 = np.full_like(x_seg, params_scn["agl_base_m"])
        y_seg0 = np.zeros_like(x_seg)
        base_len = segment_length(x_seg, y_seg0, z_seg0).sum()
        target_len = base_len + target_extra
        amp = solve_amplitude(x_seg, z_seg0, center, half_w, target_len)
        y[mask] += triangular(x_seg, center, half_w, amp)
        if ob["vertical"]:
            z_agl[mask] += 20.0 * (1.0 - np.abs(x_seg - center) / half_w)
        records.append({
            "id": ob["id"],
            "type": ob["type"],
            "x_km": ob["x_km"],
            "add_pct": ob["add_pct"],
            "amplitude_m": float(amp),
            "target_extra_m": float(target_extra),
        })
    terrain = terrain_height(x, params_trn)
    z_amsl = terrain + z_agl
    return x, y, z_agl, z_amsl, terrain, records