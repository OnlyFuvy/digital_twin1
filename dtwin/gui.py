"""Автономный двумерный проигрыватель с одновременным показом всех графиков."""
import json

import numpy as np
from matplotlib import pyplot as plt
from matplotlib.widgets import Slider, Button

from .plots import style, route_map, NAVY, TEAL, ORANGE

from .plots import DASHBOARD_SIGNALS, signal_plot


def create_dashboard(r, streams, report):
    style()
    fig = plt.figure(figsize=(16, 9.5), facecolor='#edf3f6')
    fig.canvas.manager.set_window_title('ВТ-440 | Цифровой двойник')
    fig.text(.055, .95, 'ВТ-440  /  ЦИФРОВОЙ ДВОЙНИК',
             fontsize=23, weight='bold', color=NAVY)
    energy=report.get('energy')
    summary='Маршрут и все графики отображаются одновременно'
    if energy:
        summary=(f"Заправка {energy['initial_fuel_kg']:.2f} кг · "
                 f"потребность {energy['fuel_required_kg']:.2f} кг · "
                 f"после посадки {energy['final_fuel_kg']:.2f} кг · "
                 f"резерв ≥{energy['reserve_kg']:.2f} кг")
        if not report.get('passed',True):summary+=' · ПЛАН ОТКЛОНЁН'
    fig.text(.055, .91, summary, fontsize=12, color='#647583')
    grid = fig.add_gridspec(3, 4, left=.065, right=.95, bottom=.20, top=.84,
                           wspace=.95, hspace=1.05, height_ratios=[1.15, 1, 1])

    route = fig.add_subplot(grid[0, :2])
    route_map(route, r)
    route.set_title('Маршрут БПЛА и препятствия', fontsize=12)
    route.legend(loc='upper right', fontsize=8)
    vehicle, = route.plot([], [], marker='o', color=ORANGE,
                          markersize=8, linestyle='', zorder=5)

    status = fig.add_subplot(grid[0, 2:])
    status.set_axis_off()
    info = status.text(0, 1, '', fontsize=12, linespacing=1.5,
                       fontfamily='DejaVu Sans Mono', va='top',
                       transform=status.transAxes)

    minutes = r['t'] / 60
    profile = fig.add_subplot(grid[1, :2])
    height_line, = profile.plot(minutes, r['agl'], color=TEAL, lw=1.6,
                                label='Высота AGL')
    speed = profile.twinx()
    speed_line, = speed.plot(minutes, r['speed'] * 3.6, color=ORANGE,
                             lw=1.2, label='Вдоль траектории')
    speed.grid(False)
    profile.set(xlabel='Время, мин', ylabel='Высота AGL, м',
                title='Маршрут после взлёта')
    speed.set_ylabel('Скорость, км/ч')
    profile.set_ylim(0, 1.1 * max(r['agl']))
    profile.legend(handles=[height_line, speed_line], fontsize=8, loc='upper right')

    fuel = fig.add_subplot(grid[2, :2])
    fuel_line, = fuel.plot(minutes, r['fuel_kg'], color=TEAL, lw=1.6,
                           label='Остаток в баке')
    power = fuel.twinx()
    power_line, = power.plot(minutes, r['power_kw'], color=ORANGE,
                             lw=1.2, label='Мощность')
    power.grid(False)
    fuel.set(xlabel='Время, мин', ylabel='Топливо, кг', title='Топливо и мощность')
    power.set_ylabel('Мощность, кВт')
    fuel.legend(handles=[fuel_line, power_line], fontsize=8, loc='upper right')

    cursors = [ax.axvline(0, color=ORANGE, lw=1) for ax in (profile, fuel)]
    signal_axes = {}
    for index, (sid, label, unit) in enumerate(DASHBOARD_SIGNALS):
        ax = fig.add_subplot(grid[1 + index // 2, 2 + index % 2])
        signal_axes[sid] = ax
        stream = streams.get(sid)
        signal_plot(ax, sid, stream, report.get('configuration', {}))
        ax.set_title(ax.get_title(), fontsize=10)
        ax.tick_params(labelsize=8)
        cursors.append(ax.axvline(0, color=ORANGE, lw=1))

    for ax in (profile, fuel):
        ax.set_title(ax.get_title(), fontsize=12)

    slider = Slider(fig.add_axes([.12, .085, .50, .025]), 'Маршрут, с',
                    0, float(r['t'][-1]), valinit=0, valfmt='%0.0f', color=TEAL)
    button = Button(fig.add_axes([.70, .075, .14, .045]), 'Пуск / пауза',
                    color='#dbe7ec', hovercolor='#bfd7df')
    reset = Button(fig.add_axes([.86, .075, .10, .045]), 'В начало',
                   color='#dbe7ec', hovercolor='#bfd7df')
    active = [False]

    vertical = np.gradient(r['z'], r['t'], edge_order=2 if len(minutes)>2 else 1) if len(minutes)>1 else np.zeros_like(minutes)

    def update(t):
        i = min(np.searchsorted(r['t'], t), len(r['t']) - 1)
        vehicle.set_data([r['x'][i] / 1000], [r['y'][i] / 1000])
        for cursor in cursors:
            cursor.set_xdata([t / 60, t / 60])
        info.set_text(
            f"НА МАРШРУТЕ {t / 60:7.2f} мин\n"
            f"ПО ПУТИ     {r['speed'][i] * 3.6:7.1f} км/ч\n"
            f"ВЕРТИКАЛЬНО {vertical[i]:+7.2f} м/с\n"
            f"ВЫСОТА AGL  {r['agl'][i]:7.1f} м\n"
            f"ТОПЛИВО     {r['fuel_kg'][i]:7.2f} кг\n"
            f"МОЩНОСТЬ    {r['power_kw'][i]:7.2f} кВт")
        fig.canvas.draw_idle()

    def rewind(_):
        active[0] = False
        slider.set_val(0)

    slider.on_changed(update)
    button.on_clicked(lambda _: active.__setitem__(0, not active[0]))
    reset.on_clicked(rewind)
    timer = fig.canvas.new_timer(interval=100)

    def tick():
        if active[0]:
            slider.set_val(min(slider.val + 3., slider.valmax))
            if slider.val >= slider.valmax:
                active[0] = False

    timer.add_callback(tick)
    timer.start()
    fig.canvas.mpl_connect('close_event', lambda _: timer.stop())
    fig.text(.70, .035, 'Воспроизведение ×30   ·   шаг 0,1 с',
             fontsize=10, color='#647583')
    # Сохраняем элементы управления и даём скриптам доступ к шкале времени.
    fig._twin_controls = (slider, button, reset, timer, update)
    fig._twin_axes = dict(route=route, profile=profile, fuel=fuel,
                          signals=signal_axes)
    update(0)
    return fig


def launch(c, out, with_navigation=True, record=False):
    from .simulation import simulate
    print('Расчёт модели. Обычно требуется до минуты…')
    r, s, n, report = simulate(c, out, with_navigation, record)
    create_dashboard(r, s, report)
    plt.show()
    return report


def replay(directory):
    report = json.loads((directory / 'results.json').read_text(encoding='utf-8'))
    a = np.genfromtxt(directory / 'trajectory.csv', delimiter=',', names=True)
    r = {key: a[col] for key, col in [
        ('t', 't_s'), ('x', 'x_m'), ('y', 'y_m'), ('z', 'z_amsl_m'),
        ('agl', 'agl_m'), ('speed', 'speed_ms'), ('fuel_kg', 'fuel_kg'),
        ('power_kw', 'power_kw')]}
    r['obstacles'] = report['route']['obstacles']
    streams = {}
    samples = directory / 'sensor_samples.csv'
    if samples.exists():
        import csv
        with samples.open(encoding='utf-8') as f:
            rows = list(csv.DictReader(f))
        for sid, _, unit in DASHBOARD_SIGNALS:
            selected = [row for row in rows if row['channel'] == sid]
            streams[sid] = dict(
                t=np.array([float(z['t_s']) for z in selected]),
                values=np.array([[float(z['v1'])] for z in selected]).reshape(-1, 1),
                quality=np.array([int(z['quality']) for z in selected]),
                rate_hz=.2, units=['kg/s' if sid=='FUEL_FLOW' else unit])
    create_dashboard(r, streams, report)
    plt.show()
