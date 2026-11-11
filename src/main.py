import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from src.inout.read_data import load_json
from src.inout.write_data import write_trajectory_csv, write_results_json
from src.model.route import build_route
from src.model.kinematics import compute_kinematics
from src.model.energy import compute_energy
from src.model.telemetry import compute_memory_budget


def main():
    params = load_json(ROOT / "src" / "config" / "params.json")
    coordinates = load_json(ROOT / "src" / "config" / "sensors_coordinates.json")
    scn = params["scenario"]
    trn = params["terrain"]
    eng = params["energy"]
    uav = params["uav"]
    n = params["sampling"]["n_points"]

    x, y, z_agl, z_amsl, terrain, records = build_route(scn, trn, n)

    v_cruise_ms = uav["cruise_speed_kmh"] / 3.6
    s, v, t, in_detour = compute_kinematics(x, y, z_agl, scn, v_cruise_ms)

    P, E, soc, mass_nominal, mass_after_drop, E_total = compute_energy(
        t, in_detour, z_agl, eng
    )

    duration_s = float(t[-1])
    budget = compute_memory_budget(duration_s)
    budget90 = compute_memory_budget(90.0 * 60.0)

    header = ["t_s", "x_m", "y_m", "z_agl_m", "z_amsl_m", "terrain_m",
              "v_ms", "s_m", "in_detour", "soc", "mass_kg", "P_w"]
    rows = []
    for i in range(x.size):
        rows.append([
            float(t[i]), float(x[i]), float(y[i]), float(z_agl[i]),
            float(z_amsl[i]), float(terrain[i]), float(v[i]), float(s[i]),
            int(in_detour[i]), float(soc[i]), float(mass_nominal[i]), float(P[i]),
        ])
    write_trajectory_csv(ROOT / "data" / "trajectory.csv", rows, header)

    report = {
        "uav": uav,
        "route": {
            "L0_km": scn["L0_m"] / 1000.0,
            "total_length_km": float(s[-1]) / 1000.0,
            "detour_length_km": float(s[-1] - scn["L0_m"]) / 1000.0,
            "detour_pct": float((s[-1] - scn["L0_m"]) / scn["L0_m"] * 100.0),
            "duration_min": duration_s / 60.0,
            "avg_ground_speed_kmh": float(s[-1] / duration_s * 3.6),
            "n_points": int(x.size),
            "obstacles": records,
        },
        "energy": {
            "E_total_kwh": eng["E_total_kwh"],
            "E_consumed_kwh": float(E[-1]) / 3.6e6,
            "soc_final_pct": float(soc[-1] * 100.0),
            "mass_nominal_kg": float(mass_nominal[0]),
            "mass_after_drop_kg": float(mass_after_drop[-1]),
            "mass_drop_kg": eng["mass_drop_kg"],
            "mass_drop_time_min": eng["mass_drop_time_min"],
        },
        "memory": {
            "duration_min": duration_s / 60.0,
            "useful_mib": budget["useful_mib"],
            "full_mib": budget["full_mib"],
            "overhead_factor": budget["overhead_factor"],
            "useful_mib_90min": budget90["useful_mib"],
            "full_mib_90min": budget90["full_mib"],
        },
        "sensors": {
            "total": len(coordinates["sensors"]),
            "active": sum(1 for s in coordinates["sensors"] if s["status"] == "active"),
            "stub": sum(1 for s in coordinates["sensors"] if s["status"] == "stub"),
            "list": coordinates["sensors"],
        },
    }
    write_results_json(ROOT / "output" / "results.json", report)

    print(f"Датчиков: {report['sensors']['total']} "
          f"(активных = {report['sensors']['active']}, "
          f"отключенных = {report['sensors']['stub']})")
    print(f"Длина маршрута: {report['route']['total_length_km']:.3f} км")
    print(f"Время полёта: {report['route']['duration_min']:.2f} мин")
    print(f"Масса (номинальная): {report['energy']['mass_nominal_kg']:.1f} кг")
    print(f"Объём записи ({report['route']['duration_min']:.2f} мин): "
          f"{budget['full_mib']:.2f} МиБ")
    print(f"Объём записи (90 мин): {budget90['full_mib']:.2f} МиБ")


if __name__ == "__main__":
    main()