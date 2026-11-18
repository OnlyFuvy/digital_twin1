"""Расчёт потребной мощности, топлива ДВС и заряда бортовой батареи."""
import copy
import numpy as np


def integrate_energy(r,c):
    """Оценить ресурсы заданной траектории; допустимость проверяется отдельно."""
    v,e,env=c['vehicle'],c['energy'],c['environment']
    t=r['t']; dt=np.diff(t,prepend=t[0])
    air=r['velocity']-np.array(env['wind_ms'])
    airspeed=np.linalg.norm(air,axis=1)
    # Давление по стандартной атмосфере; приземная температура задаётся сценарием.
    temp=env['temperature_c']+273.15-.0065*r['z']
    rho=101325*(1-2.25577e-5*r['z'])**5.25588/(287.05*temp)
    n=len(t); fuel=np.empty(n); power=np.empty(n); flow=np.empty(n); mass=np.empty(n)
    dry=v['empty_mass_kg']+v['payload_kg']+v['battery_mass_kg']
    def shaft(m,i):
        thrust=m*9.80665/np.maximum(np.cos(r['euler'][i,0]),.6)
        vh=np.sqrt(thrust/(2*rho[i]*np.pi*v['rotor_radius_m']**2))
        u=airspeed[i]
        # Равносильная форма без вычитания близких чисел на большой скорости.
        q=(u/(2*vh))**2
        induced=1.15*thrust*vh/np.sqrt(np.hypot(1.,q)+q)
        profile=e['profile_power_kw']*1000*(1+3*(u/180)**2)
        parasite=.5*rho[i]*e['drag_area_m2']*u**3
        climb=max(0,m*9.80665*r['velocity'][i,2])
        return (induced+profile+parasite+climb)/e['transmission_efficiency']/1000
    # По 60 с на вылет и посадку при мощности висения с начальной массой.
    # Это энергетическая надбавка, отдельная от неприкосновенного остатка 3%.
    hover_mass=dry+v['fuel_kg']; thrust=hover_mass*9.80665
    hover_kw=(1.15*thrust**1.5/np.sqrt(2*rho[0]*np.pi*v['rotor_radius_m']**2)/1000+e['profile_power_kw'])/e['transmission_efficiency']
    ground_phase_s=120.
    phase_fuel=hover_kw*ground_phase_s/3600*e['bsfc_kg_kwh']
    required=phase_fuel/2
    for i in range(n):
        if i: required+=flow[i-1]*dt[i]
        # При нехватке топлива продолжается только прогноз заданного плана.
        # Содержимое бака неотрицательно; дефицит учитывается отдельно.
        fuel[i]=max(0.,v['fuel_kg']-required)
        mass[i]=dry+fuel[i];power[i]=shaft(mass[i],i)
        flow[i]=power[i]*e['bsfc_kg_kwh']/3600
    required+=phase_fuel/2
    final_fuel=max(0.,v['fuel_kg']-required)
    # Та же квадратура левых прямоугольников, что и в балансе топлива.
    shaft_kwh=float(np.dot(power[:-1],np.diff(t))/3600+hover_kw*ground_phase_s/3600)
    reserve_kg=v['fuel_kg']*e['reserve_fraction']
    battery_load=e['hotel_power_w']-e['generator_w']
    soc=e['initial_soc']-battery_load*(t+ground_phase_s/2)/3600/e['battery_wh']
    voltage=24+4*np.clip(soc,0,1)
    current=battery_load/voltage
    temp_batt=env['temperature_c']+15+current**2*.08*(1-np.exp(-t/500))
    density=e.get('fuel_density_kg_l',.75);capacity=e.get('tank_capacity_l')
    initial_l=v['fuel_kg']/density
    peak_kw=float(max(power.max(),hover_kw))
    r.update(fuel_kg=fuel,mass_kg=mass,power_kw=power,fuel_flow_kgs=flow,
             soc=soc,voltage=voltage,current=current,temp_batt=temp_batt,airspeed=airspeed,rho=rho)
    return dict(initial_fuel_kg=v['fuel_kg'],initial_fuel_l=initial_l,
        final_fuel_kg=float(final_fuel),final_fuel_l=float(final_fuel/density),
        final_fuel_percent=float(100*final_fuel/v['fuel_kg']),reserve_kg=reserve_kg,
        reserve_passed=bool(required+reserve_kg<=v['fuel_kg']),fuel_consumed_kg=float(v['fuel_kg']-final_fuel),
        fuel_required_kg=float(required),fuel_shortfall_kg=max(0.,float(required-v['fuel_kg'])),
        reserve_shortfall_kg=max(0.,float(required+reserve_kg-v['fuel_kg'])),
        fuel_density_kg_l=density,tank_capacity_l=capacity,tank_capacity_checked=capacity is not None,
        tank_capacity_passed=bool(initial_l<=capacity) if capacity is not None else None,
        shaft_energy_kwh=shaft_kwh,peak_power_kw=peak_kw,hover_power_kw=float(hover_kw),
        max_power_kw=v['max_power_kw'],power_passed=bool(peak_kw<=v['max_power_kw']),
        phase_allowance_s=ground_phase_s,phase_allowance_fuel_kg=float(phase_fuel),
        final_soc=float(soc[-1]-battery_load*ground_phase_s/2/3600/e['battery_wh']),
        initial_mass_kg=dry+v['fuel_kg'],final_mass_kg=dry+final_fuel,
        total_duration_s=float(t[-1]-t[0]+ground_phase_s),
        model='Учебная квазистационарная оценка; параметры не откалиброваны по полётам')


def minimum_fuel_for_route(r,c,tolerance_kg=1e-4):
    """Найти заправку с остатком ≥3%, пересчитывая расход при каждой массе.

    Поиск ограничен взлётной массой и, если он известен, объёмом бака.
    Превышение мощности остаётся отдельным основанием для отказа миссии.
    Исходные конфигурация и траектория не изменяются.
    """
    if not np.isfinite(tolerance_kg) or tolerance_kg<=0:
        raise ValueError('Точность поиска топлива должна быть конечной и положительной')
    cfg=copy.deepcopy(c);v,e=cfg['vehicle'],cfg['energy']
    dry=sum(v[k] for k in ('empty_mass_kg','payload_kg','battery_mass_kg'))
    upper=v['max_mass_kg']-dry
    if e.get('tank_capacity_l') is not None:
        upper=min(upper,e['tank_capacity_l']*e.get('fuel_density_kg_l',.75))
    if upper<=0:raise ValueError('Нет допустимого запаса массы для топлива')
    def evaluate(initial):
        v['fuel_kg']=initial
        return integrate_energy(dict(r),cfg)
    if not evaluate(upper)['reserve_passed']:
        return dict(feasible=False,minimum_initial_fuel_kg=None,
                    reason='Топлива в допустимых пределах массы и бака недостаточно')
    lower=0.
    while upper-lower>tolerance_kg:
        middle=(lower+upper)/2
        if evaluate(middle)['reserve_passed']:upper=middle
        else:lower=middle
    result=evaluate(upper)
    return dict(feasible=True,minimum_initial_fuel_kg=upper,
                minimum_initial_fuel_l=upper/e.get('fuel_density_kg_l',.75),
                final_fuel_kg=result['final_fuel_kg'],reserve_kg=result['reserve_kg'],
                fuel_required_kg=result['fuel_required_kg'],
                power_passed=result['power_passed'],tolerance_kg=tolerance_kg)
