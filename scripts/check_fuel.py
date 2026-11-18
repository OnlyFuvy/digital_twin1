"""Повторяемая проверка заправок 1, 5, 10 и 60 кг без расчёта датчиков и БИНС."""
import csv
import json
import sys
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from dtwin.config import load_config,scenario
from dtwin.route import build_route,time_parameterize
from dtwin.energy import integrate_energy,minimum_fuel_for_route


def main():
    base=load_config()
    route=time_parameterize(build_route(base),base)
    rows=[];minimum={}
    for name in ('nominal','strong_wind'):
        cfg=scenario(base,name)
        minimum[name]=minimum_fuel_for_route(route,cfg)
        for initial in (1.,5.,10.,60.):
            cfg['vehicle']['fuel_kg']=initial
            e=integrate_energy(dict(route),cfg)
            row=dict(scenario=name,initial_fuel_kg=initial,
                     initial_fuel_l=e['initial_fuel_l'],fuel_required_kg=e['fuel_required_kg'],
                     final_fuel_kg=e['final_fuel_kg'],fuel_shortfall_kg=e['fuel_shortfall_kg'],
                     reserve_kg=e['reserve_kg'],reserve_passed=e['reserve_passed'],
                     peak_power_kw=e['peak_power_kw'],power_passed=e['power_passed'],
                     initial_mass_kg=e['initial_mass_kg'])
            rows.append(row)
            print(f"{name:12s} · заправка {initial:5.1f} кг · потребность {e['fuel_required_kg']:.3f} кг · "
                  f"остаток {e['final_fuel_kg']:.3f} кг · резерв: {e['reserve_passed']} · мощность: {e['power_passed']}")
        print(f"Минимальная заправка по топливу: {minimum[name]['minimum_initial_fuel_kg']:.4f} кг")
    out=Path(__file__).resolve().parents[1]/'results'
    report=dict(configuration=base,minimum=minimum,comparisons=rows,
                tank_capacity_status='Паспортная вместимость не найдена в проверенных открытых источниках',
                note='Сильный ветер: оценка топлива для заданного плана, который отклоняется по мощности двигателя')
    (out/'fuel_audit.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    with (out/'fuel_audit.csv').open('w',newline='',encoding='utf-8') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]),lineterminator='\n')
        writer.writeheader();writer.writerows(rows)


if __name__=='__main__':main()
