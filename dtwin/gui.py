"""Offline Python desktop player with a timeline and sensor selection."""
import json
from pathlib import Path
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.widgets import Slider,Button,RadioButtons
from .plots import style,draw_3d,NAVY,TEAL,ORANGE


def create_dashboard(r,streams,report):
    style();fig=plt.figure(figsize=(15.5,9),facecolor='#edf3f6')
    fig.canvas.manager.set_window_title('ВТ-440 | Цифровой двойник')
    fig.text(.045,.95,'ВТ-440  /  ЦИФРОВОЙ ДВОЙНИК',fontsize=23,weight='bold',color=NAVY)
    fig.text(.045,.915,'Учебная модель · маршрут, рельеф, измерения и топливо',fontsize=12,color='#647583')
    ax3=fig.add_axes([.035,.39,.61,.49],projection='3d');draw_3d(ax3,r)
    vehicle,=ax3.plot([],[],[],marker='o',color=ORANGE,markersize=9,linestyle='')
    profile=fig.add_axes([.08,.15,.55,.19]);profile.plot(r['t']/60,r['agl'],color=TEAL,lw=2)
    profile.set(xlabel='Время маршрутного участка, мин',ylabel='Высота AGL, м')
    cursor=profile.axvline(0,color=ORANGE)
    info=fig.text(.69,.86,'',fontsize=13,linespacing=1.8,fontfamily='DejaVu Sans Mono',va='top')
    ax_sensor=fig.add_axes([.71,.22,.25,.24]);names=['IMU1','FUEL_FLOW','LIDAR_DOWN','TEMP_BATT']
    radio=RadioButtons(fig.add_axes([.71,.51,.25,.16],facecolor='#edf3f6'),names,activecolor=TEAL)
    sensor_line,=ax_sensor.plot([],[],color=TEAL,lw=.8);sensor_cursor=ax_sensor.axvline(0,color=ORANGE)
    ax_sensor.set_xlabel('Время, мин')
    selected=[names[0]]
    def select(name):
        selected[0]=name
        if name in streams:
            stream=streams[name];step=max(1,int(stream['rate_hz']/5))
            sensor_line.set_data(stream['t'][::step]/60,stream['values'][::step,0]);ax_sensor.relim();ax_sensor.autoscale_view()
            ax_sensor.set_ylabel(stream['units'][0]);ax_sensor.set_title(name)
        fig.canvas.draw_idle()
    radio.on_clicked(select)
    slider=Slider(fig.add_axes([.12,.065,.54,.025]),'Полёт, с',0,float(r['t'][-1]),valinit=0,color=TEAL)
    button=Button(fig.add_axes([.71,.06,.12,.045]),'Пуск / пауза',color='#dbe7ec',hovercolor='#bfd7df')
    reset=Button(fig.add_axes([.85,.06,.10,.045]),'В начало',color='#dbe7ec',hovercolor='#bfd7df')
    active=[False]
    def update(t):
        i=min(np.searchsorted(r['t'],t),len(r['t'])-1)
        vehicle.set_data_3d([r['x'][i]/1000],[r['y'][i]/1000],[r['z'][i]])
        cursor.set_xdata([t/60,t/60]);sensor_cursor.set_xdata([t/60,t/60])
        info.set_text(f"ВРЕМЯ       {t/60:7.2f} мин\nСКОРОСТЬ    {r['speed'][i]*3.6:7.1f} км/ч\nВЫСОТА AGL  {r['agl'][i]:7.1f} м\nТОПЛИВО     {r['fuel_kg'][i]:7.2f} кг\nМОЩНОСТЬ    {r['power_kw'][i]:7.2f} кВт")
        fig.canvas.draw_idle()
    slider.on_changed(update)
    button.on_clicked(lambda _:active.__setitem__(0,not active[0]))
    reset.on_clicked(lambda _:slider.set_val(0))
    timer=fig.canvas.new_timer(interval=100)
    def tick():
        if active[0]:
            value=min(slider.val+3.,slider.valmax);slider.set_val(value)
            if value>=slider.valmax:active[0]=False
    timer.add_callback(tick);timer.start();select(names[0]);update(0)
    fig.text(.71,.12,'Воспроизведение ×30   ·   шаг 0,1 с',fontsize=10,color='#647583')
    fig._twin_controls=(slider,button,reset,radio,timer,update,select)
    return fig


def launch(c,out):
    from .simulation import simulate
    print('Расчёт модели. Обычно требуется до минуты…')
    r,s,n,report=simulate(c,out,True,False)
    create_dashboard(r,s,report);plt.show()


def replay(directory):
    report=json.loads((directory/'results.json').read_text(encoding='utf-8'))
    a=np.genfromtxt(directory/'trajectory.csv',delimiter=',',names=True)
    r={key:a[col] for key,col in [('t','t_s'),('x','x_m'),('y','y_m'),('z','z_amsl_m'),('agl','agl_m'),('speed','speed_ms'),('fuel_kg','fuel_kg'),('power_kw','power_kw')]}
    r['obstacles']=report['route']['obstacles'];streams={}
    samples=directory/'sensor_samples.csv'
    if samples.exists():
        import csv
        with samples.open(encoding='utf-8') as f:
            rows=list(csv.DictReader(f))
        for sid in ('IMU1','FUEL_FLOW','LIDAR_DOWN','TEMP_BATT'):
            selected=[row for row in rows if row['channel']==sid]
            spec=next(s for s in report['sensors'] if s['id']==sid)
            streams[sid]=dict(t=np.array([float(z['t_s']) for z in selected]),values=np.array([[float(z['v1'])] for z in selected]),rate_hz=.2,units=spec['units'])
    create_dashboard(r,streams,report);plt.show()
