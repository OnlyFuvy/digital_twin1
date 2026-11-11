"""Reduced-order rotorcraft shaft power, fuel and auxiliary battery model."""
import numpy as np


def integrate_energy(r,c):
    v,e,env=c['vehicle'],c['energy'],c['environment']
    t=r['t']; dt=np.diff(t,prepend=0)
    air=r['velocity']-np.array(env['wind_ms'])
    airspeed=np.linalg.norm(air,axis=1)
    # ISA pressure with configured near-surface temperature, constant gravity.
    temp=env['temperature_c']+273.15-.0065*r['z']
    rho=101325*(1-2.25577e-5*r['z'])**5.25588/(287.05*temp)
    n=len(t); fuel=np.empty(n); power=np.empty(n); flow=np.empty(n); mass=np.empty(n)
    dry=v['empty_mass_kg']+v['payload_kg']+v['battery_mass_kg']
    def shaft(m,i):
        thrust=m*9.80665/np.maximum(np.cos(r['euler'][i,0]),.6)
        vh=np.sqrt(thrust/(2*rho[i]*np.pi*v['rotor_radius_m']**2))
        u=airspeed[i]
        induced=1.15*thrust*vh*np.sqrt(np.sqrt(1+(u/(2*vh))**4)-(u/(2*vh))**2)
        profile=e['profile_power_kw']*1000*(1+3*(u/180)**2)
        parasite=.5*rho[i]*e['drag_area_m2']*u**3
        climb=max(0,m*9.80665*r['velocity'][i,2])
        return (induced+profile+parasite+climb)/e['transmission_efficiency']/1000
    # Dedicated reserve for vertical departure/arrival: 2 min at hover power.
    hover_mass=dry+v['fuel_kg']; thrust=hover_mass*9.80665
    hover_kw=(1.15*thrust**1.5/np.sqrt(2*rho[0]*np.pi*v['rotor_radius_m']**2)/1000+e['profile_power_kw'])/e['transmission_efficiency']
    ground_phase_s=120.
    phase_fuel=hover_kw*ground_phase_s/3600*e['bsfc_kg_kwh']
    fuel[0]=v['fuel_kg']-phase_fuel/2
    for i in range(n):
        if i: fuel[i]=fuel[i-1]-flow[i-1]*dt[i]
        mass[i]=dry+max(0,fuel[i]);power[i]=shaft(mass[i],i)
        flow[i]=power[i]*e['bsfc_kg_kwh']/3600
    final_fuel=fuel[-1]-phase_fuel/2
    shaft_kwh=float(np.trapezoid(power,t)/3600+hover_kw*ground_phase_s/3600)
    reserve_kg=v['fuel_kg']*e['reserve_fraction']
    battery_load=e['hotel_power_w']-e['generator_w']
    soc=e['initial_soc']-battery_load*(t+ground_phase_s/2)/3600/e['battery_wh']
    voltage=24+4*np.clip(soc,0,1)
    current=battery_load/voltage
    temp_batt=env['temperature_c']+15+current**2*.08*(1-np.exp(-t/500))
    r.update(fuel_kg=fuel,mass_kg=mass,power_kw=power,fuel_flow_kgs=flow,
             soc=soc,voltage=voltage,current=current,temp_batt=temp_batt,airspeed=airspeed,rho=rho)
    return dict(initial_fuel_kg=v['fuel_kg'],final_fuel_kg=float(final_fuel),
        final_fuel_percent=float(100*final_fuel/v['fuel_kg']),reserve_kg=reserve_kg,
        reserve_passed=bool(final_fuel>=reserve_kg),fuel_consumed_kg=float(v['fuel_kg']-final_fuel),
        shaft_energy_kwh=shaft_kwh,peak_power_kw=float(power.max()),hover_power_kw=float(hover_kw),
        max_power_kw=v['max_power_kw'],power_passed=bool(max(power.max(),hover_kw)<=v['max_power_kw']),
        phase_allowance_s=ground_phase_s,phase_allowance_fuel_kg=float(phase_fuel),
        final_soc=float(soc[-1]-battery_load*ground_phase_s/2/3600/e['battery_wh']),
        initial_mass_kg=dry+v['fuel_kg'],final_mass_kg=dry+final_fuel,
        total_duration_s=float(t[-1]+ground_phase_s),model='quasi-steady momentum inspired, uncalibrated')
