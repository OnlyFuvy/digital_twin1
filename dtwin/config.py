"""Configuration loading and fail-fast validation."""
import copy
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def load_config(path=None):
    data = json.loads(Path(path or ROOT / 'config/mission.json').read_text(encoding='utf-8'))
    validate(data)
    return data


def load_sensors(c=None):
    specs=json.loads((ROOT / 'config/sensors.json').read_text(encoding='utf-8'))['sensors']
    if c is not None:
        for s in specs:
            if s['type']=='IMU':s['rate_hz']=c['sensors']['imu_hz']
            elif s['type']=='GNSS' or s['id']=='GNSS_VEL':
                s['rate_hz']=max(s['rate_hz'],math.ceil(c['vehicle']['cruise_ms']/2.))
            elif s['type'].startswith('LIDAR') or s['id']=='RADAR_ALT':
                s['rate_hz']=max(s['rate_hz'],math.ceil(c['vehicle']['cruise_ms']/.75))
    return specs


def validate(c):
    def finite(value):
        if isinstance(value, dict):
            for v in value.values(): finite(v)
        elif isinstance(value, list):
            for v in value: finite(v)
        elif isinstance(value, (int, float)) and not math.isfinite(value):
            raise ValueError('Конфигурация содержит NaN или бесконечность')
    finite(c)
    r, v, e = c['route'], c['vehicle'], c['energy']
    for key in ('baseline_m', 'agl_m', 'half_width_m', 'spatial_step_m'):
        if r[key] <= 0: raise ValueError(f'route.{key} должен быть положительным')
    if 2*r['half_width_m'] >= r['baseline_m']/8:
        raise ValueError('Участки манёвров пересекаются')
    if r['spatial_step_m'] > r['half_width_m']/20:
        raise ValueError('Слишком крупный пространственный шаг')
    if len(r['extra_percent']) != 7 or sorted(r['types']) != sorted(['no_fly','over','over','over','bridge','lateral','lateral']):
        raise ValueError('Нужны семь препятствий: запретная зона, три верхних, мост и два боковых')
    if len(r['sides']) != 7 or any(s not in (-1, 1) for s in r['sides']):
        raise ValueError('Стороны облёта задаются семью значениями -1 или 1')
    if any(p <= 0 for p in r['extra_percent']): raise ValueError('Добавки пути должны быть положительными')
    for key, value in v.items():
        if value <= 0: raise ValueError(f'vehicle.{key} должен быть положительным')
    if sum(v[k] for k in ('empty_mass_kg','payload_kg','battery_mass_kg','fuel_kg')) > v['max_mass_kg']:
        raise ValueError('Превышена максимальная взлётная масса')
    if not 0 < e['reserve_fraction'] < 1 or not 0 <= e['initial_soc'] <= 1:
        raise ValueError('Некорректный запас топлива или SOC')
    for key in ('bsfc_kg_kwh','battery_wh','transmission_efficiency'):
        if e[key] <= 0: raise ValueError(f'energy.{key} должен быть положительным')
    if c['sensors']['imu_hz'] < 100: raise ValueError('Частота IMU должна быть не менее 100 Гц')
    if c['sensors']['dropout_end_s'] < c['sensors']['dropout_start_s']:
        raise ValueError('Конец отказа GNSS раньше начала')
    if not 0<v['max_bank_deg']<70:raise ValueError('Допустимый крен должен находиться между 0 и 70 градусами')
    if not -100<c['environment']['temperature_c']<60:raise ValueError('Температура вне области применимости модели')
    if len(c['environment']['wind_ms'])!=3:raise ValueError('Вектор ветра должен содержать три компоненты')
    return c


def scenario(base, name):
    c = copy.deepcopy(base)
    if name == 'gnss_dropout': pass
    elif name == 'nominal':
        c['sensors']['dropout_start_s'] = 0
        c['sensors']['dropout_end_s'] = 0
    elif name == 'low_fuel': c['vehicle']['fuel_kg'] = 1.
    elif name == 'strong_wind': c['environment']['wind_ms'] = [-18., 8., 0.]
    else: raise ValueError(f'Неизвестный сценарий {name}')
    validate(c)
    return c
