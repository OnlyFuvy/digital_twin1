"""Воспроизведение контрольного и стрессовых сценариев из корня проекта."""
import csv
import json
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import matplotlib
matplotlib.use('Agg')
from dtwin.config import load_config,scenario
from dtwin.simulation import simulate
from dtwin.plots import save_figures
from dtwin.gui import create_dashboard
from matplotlib import pyplot as plt


def experiment_row(name,rep):
    """Свести готовый отчёт к строке сравнения, сохранив причины отказа."""
    e=rep['energy'];nav=rep['navigation']
    return dict(scenario=name,duration_s=e['total_duration_s'],
                initial_fuel_kg=e['initial_fuel_kg'],fuel_required_kg=e['fuel_required_kg'],
                fuel_consumed_kg=e['fuel_consumed_kg'],fuel_shortfall_kg=e['fuel_shortfall_kg'],
                fuel_final_kg=e['final_fuel_kg'],reserve_kg=e['reserve_kg'],
                reserve_passed=e['reserve_passed'],peak_power_kw=e['peak_power_kw'],
                power_passed=e['power_passed'],passed=rep['passed'],
                failed_checks=';'.join(k for k,v in rep['checks'].items() if not v),
                navigation_rmse_m=nav['rmse_m'] if nav else '',outage_peak_m=nav['outage_peak_m'] if nav else '')


def save_summary(root,rows):
    """Сохранить одну и ту же сводку в CSV и JSON."""
    with (root/'experiments.csv').open('w',newline='',encoding='utf-8') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]),lineterminator='\n')
        writer.writeheader();writer.writerows(rows)
    (root/'experiments.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2),encoding='utf-8')


def main():
    root=Path(__file__).resolve().parents[1]/'results';rows=[]
    for name in ('nominal','gnss_dropout','low_fuel','strong_wind'):
        c=scenario(load_config(),name)
        out=root/('reference' if name=='gnss_dropout' else name)
        r,s,n,rep=simulate(c,out,with_navigation=name in ('nominal','gnss_dropout'),record=name=='gnss_dropout')
        save_figures(r,s,n,rep,out/'figures')
        if name=='gnss_dropout':
            fig=create_dashboard(r,s,rep)
            fig._twin_controls[0].set_val(325)
            fig.savefig(out/'figures/dashboard_2d.png',dpi=150);plt.close(fig)
        row=experiment_row(name,rep)
        rows.append(row);print(row,flush=True)
    save_summary(root,rows)


if __name__=='__main__':main()
