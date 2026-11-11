"""Reproduce the reference scenario and stress experiments from project root."""
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


def main():
    root=Path('results');rows=[]
    for name in ('nominal','gnss_dropout','low_fuel','strong_wind'):
        c=scenario(load_config(),name)
        out=root/('reference' if name=='gnss_dropout' else name)
        r,s,n,rep=simulate(c,out,with_navigation=name in ('nominal','gnss_dropout'),record=name=='gnss_dropout')
        save_figures(r,s,n,rep,out/'figures')
        if name=='gnss_dropout':
            fig=create_dashboard(r,s,rep)
            fig._twin_controls[0].set_val(325)
            fig.savefig(out/'figures/dashboard.png',dpi=150);plt.close(fig)
        row=dict(scenario=name,duration_s=rep['energy']['total_duration_s'],fuel_final_kg=rep['energy']['final_fuel_kg'],
                 peak_power_kw=rep['energy']['peak_power_kw'],passed=rep['passed'],failed_checks=';'.join(k for k,v in rep['checks'].items() if not v),
                 navigation_rmse_m=rep['navigation']['rmse_m'] if n else '',outage_peak_m=rep['navigation']['outage_peak_m'] if n else '')
        rows.append(row);print(row,flush=True)
    with (root/'experiments.csv').open('w',newline='',encoding='utf-8') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]),lineterminator="\n");writer.writeheader();writer.writerows(rows)
    (root/'experiments.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2),encoding='utf-8')


if __name__=='__main__':main()
