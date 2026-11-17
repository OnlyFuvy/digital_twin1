"""Offline 2D player showing every dashboard signal at once."""
import json

import numpy as np
from matplotlib import pyplot as plt
from matplotlib.widgets import Slider, Button

from .plots import style, route_map, NAVY, TEAL, ORANGE


DASHBOARD_SIGNALS = (
    ('IMU1', 'Ускорение X', 'м/с²'),
    ('FUEL_FLOW', 'Расход топлива', 'кг/с'),
    ('LIDAR_DOWN', 'Дальность до крон', 'м'),
    ('TEMP_BATT', 'Температура батареи', '°C'),
)


def create_dashboard(r, streams, report):
    style()
    fig = plt.figure(figsize=(16, 9.5), facecolor='#edf3f6')
    fig.canvas.manager.set_window_title('ВТ-440 | Цифровой двойник')
    fig.text(.055, .95, 'ВТ-440  /  ЦИФРОВОЙ ДВОЙНИК',
             fontsize=23, weight='bold', color=NAVY)
    fig.text(.055, .91, 'Маршрут и все графики отображаются одновременно',
             fontsize=12, color='#647583')
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
                             lw=1.2, label='Скорость')
    speed.grid(False)
    profile.set(xlabel='Время, мин', ylabel='Высота AGL, м',
                title='Высота и скорость')
    speed.set_ylabel('Скорость, км/ч')
    profile.legend(handles=[height_line, speed_line], fontsize=8, loc='upper right')

    fuel = fig.add_subplot(grid[2, :2])
    fuel_line, = fuel.plot(minutes, r['fuel_kg'], color=TEAL, lw=1.6,
                           label='Топливо')
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
        if stream is not None and len(stream['t']):
            step = max(1, int(stream['rate_hz'] / 5))
            ax.plot(stream['t'][::step] / 60, stream['values'][::step, 0],
                    color=TEAL, lw=.8)
        else:
            ax.text(.5, .5, 'Нет записанных данных', ha='center', va='center',
                    fontsize=9, transform=ax.transAxes)
        ax.set(xlabel='Время, мин', ylabel=unit)
        ax.set_title(f'{sid} · {label}', fontsize=10)
        ax.tick_params(labelsize=8)
        cursors.append(ax.axvline(0, color=ORANGE, lw=1))

    for ax in (profile, fuel):
        ax.set_title(ax.get_title(), fontsize=12)

    slider = Slider(fig.add_axes([.12, .085, .50, .025]), 'Полёт, с',
                    0, float(r['t'][-1]), valinit=0, valfmt='%0.0f', color=TEAL)
    button = Button(fig.add_axes([.70, .075, .14, .045]), 'Пуск / пауза',
                    color='#dbe7ec', hovercolor='#bfd7df')
    reset = Button(fig.add_axes([.86, .075, .10, .045]), 'В начало',
                   color='#dbe7ec', hovercolor='#bfd7df')
    active = [False]

    def update(t):
        i = min(np.searchsorted(r['t'], t), len(r['t']) - 1)
        vehicle.set_data([r['x'][i] / 1000], [r['y'][i] / 1000])
        for cursor in cursors:
            cursor.set_xdata([t / 60, t / 60])
        info.set_text(
            f"ВРЕМЯ       {t / 60:7.2f} мин\n"
            f"СКОРОСТЬ    {r['speed'][i] * 3.6:7.1f} км/ч\n"
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
    # Keep controls alive for Matplotlib and expose the timeline to export scripts.
    fig._twin_controls = (slider, button, reset, timer, update)
    fig._twin_axes = dict(route=route, profile=profile, fuel=fuel,
                          signals=signal_axes)
    update(0)
    return fig


def launch(c, out):
    from .simulation import simulate
    print('Расчёт модели. Обычно требуется до минуты…')
    r, s, n, report = simulate(c, out, True, False)
    create_dashboard(r, s, report)
    plt.show()


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
                rate_hz=.2, units=[unit])
    create_dashboard(r, streams, report)
    plt.show()
