"""End-to-end simulation and traceable requirement checks."""
import json
from pathlib import Path
import numpy as np
from .config import load_sensors
from .route import build_route,time_parameterize,collision_checks
from .energy import integrate_energy
from .sensors import simulate_sensors,interpolate
from .navigation import navigate
from .telemetry import memory_budget,save_streams


def simulate(c,output=None,with_navigation=True,record=False):
    r=time_parameterize(build_route(c),c);energy=integrate_energy(r,c)
    specs=load_sensors(c);streams=simulate_sensors(r,c,specs)
    nav=navigate(r,streams,c) if with_navigation else None
    collision,canopy_margin=collision_checks(r,c)
    bank=float(np.max(np.abs(np.rad2deg(r['euler'][:,0]))))
    max_climb=float(np.max(np.abs(r['velocity'][:,2])))
    checks={
        'seven_obstacles':len(r['obstacles'])==7,
        'detour_percentages':all(abs(o['actual_percent']-o['target_percent'])<1e-6 for o in r['obstacles']),
        'vertical_half_height':all(abs(o['peak_agl_m']/c['route']['agl_m']-1.5)<1e-6 for o in r['obstacles'] if o['type']=='over'),
        'no_collision':all(o['passed'] for o in collision),
        'canopy_clearance':canopy_margin>=c['environment']['min_canopy_clearance_m'],
        'fuel_reserve_3_percent':energy['reserve_passed'],
        'engine_power':energy['power_passed'],
        'battery_remaining':energy['final_soc']>=0,
        'bank_limit':bank<=c['vehicle']['max_bank_deg']+.1,
        'climb_limit':max_climb<=c['vehicle']['max_climb_ms']+.02,
        'ceiling':float(r['z'].max())<=c['vehicle']['ceiling_m'],
        'monotonic_time':bool(np.all(np.diff(r['t'])>0)),
        'all_24_channels':len(streams)==24 and all(len(s['values'])>0 for s in streams.values())}
    budget=memory_budget(float(r['t'][-1]),specs,c)
    report=dict(configuration=c,scope='prescribed airborne trajectory plus 120 s energy allowance for departure and landing',
        route=dict(baseline_horizontal_m=c['route']['baseline_m'],baseline_3d_m=r['baseline_length_m'],
            total_3d_m=float(r['s'][-1]),extra_m=float(r['s'][-1]-r['baseline_length_m']),
            extra_percent=float((r['s'][-1]-r['baseline_length_m'])/c['route']['baseline_m']*100),
            airborne_duration_s=float(r['t'][-1]),max_bank_deg=bank,max_climb_ms=max_climb,
            min_canopy_clearance_m=canopy_margin,obstacles=r['obstacles'],collision_checks=collision),
        energy=energy,memory=budget,memory_90_min=memory_budget(5400,specs,c),memory_5_hours=memory_budget(18000,specs,c),
        navigation=nav['metrics'] if nav else None,checks=checks,passed=all(checks.values()),
        sensors=[dict(id=k,rate_hz=s['rate_hz'],samples=len(s['t']),units=s['units'],invalid=int(np.count_nonzero(s['quality']))) for k,s in streams.items()])
    if output:
        output=Path(output);output.mkdir(parents=True,exist_ok=True)
        if record:
            rate=c['telemetry']['state_hz'];t=np.arange(int(np.floor(r['t'][-1]*rate))+1)/rate
            values=np.column_stack([r['x'],r['y'],r['z'],r['velocity'],r['euler'],r['fuel_kg'],r['power_kw'],r['soc'],r['agl'],r['mass_kg'],r['airspeed'],r['speed']])
            state=dict(t=t,values=interpolate(t,r['t'],values),units=['m']*3+['m/s']*3+['rad']*3+['kg','kW','fraction','m','kg','m/s','m/s'])
            manifest=save_streams(output/'telemetry',streams,state,c)
            actual=sum(s['bytes'] for s in manifest);report['memory']['actual_binary_bytes']=actual
            report['checks']['recording_size_matches']=actual==budget['binary_bytes']
            report['passed']=all(report['checks'].values())
        (output/'results.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        columns=['t_s','x_m','y_m','z_amsl_m','agl_m','terrain_m','speed_ms','fuel_kg','mass_kg','power_kw','soc','roll_rad','pitch_rad','yaw_rad']
        data=np.column_stack([r['t'],r['x'],r['y'],r['z'],r['agl'],r['terrain'],r['speed'],r['fuel_kg'],r['mass_kg'],r['power_kw'],r['soc'],r['euler']])
        np.savetxt(output/'trajectory.csv',data,delimiter=',',header=','.join(columns),comments='',fmt='%.7f')
        if nav:
            stride=int(c['sensors']['imu_hz']/10)
            np.savetxt(output/'navigation.csv',np.column_stack([nav['t'][::stride],nav['error'][::stride],nav['pure_error'][::stride]]),delimiter=',',header='t_s,fused_error_m,pure_ins_error_m',comments='',fmt='%.6f')
        # Compact evidence at exactly 5 s for all channels, not a second raw recording.
        rows=[]
        for sid,s in streams.items():
            for i in np.arange(0,len(s['t']),max(1,int(5*s['rate_hz']))):
                rows.append([sid,float(s['t'][i]),int(s['quality'][i]),*s['values'][i].tolist()])
        import csv
        with (output/'sensor_samples.csv').open('w',newline='',encoding='utf-8') as f:
            w=csv.writer(f,lineterminator="\n");w.writerow(['channel','t_s','quality','v1','v2','v3','v4','v5','v6']);w.writerows(rows)
    return r,streams,nav,report
