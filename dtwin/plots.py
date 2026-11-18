"""Построение графиков по результатам численного расчёта."""
from pathlib import Path
from matplotlib import pyplot as plt
from matplotlib.patches import Circle

NAVY='#17334b';TEAL='#087f8c';ORANGE='#e2853b';RED='#b84c4c'


def style():
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':11,'axes.titlesize':15,
      'axes.labelcolor':NAVY,'text.color':NAVY,'axes.spines.top':False,'axes.spines.right':False,
      'axes.grid':True,'grid.alpha':.18,'figure.facecolor':'white','axes.facecolor':'#f7fafc','savefig.dpi':180})


def route_map(ax,r):
    ax.plot(r['x']/1000,r['y']/1000,color=TEAL,lw=2.2,label='Расчётный маршрут')
    ax.plot([r['x'][0]/1000,r['x'][-1]/1000],[0,0],color='#8997a3',ls='--',lw=1,label='Базовое направление')
    colors={'no_fly':RED,'over':ORANGE,'bridge':'#6d65a9','lateral':'#527d4b'}
    for o in r['obstacles']:
        ax.add_patch(Circle((o['x_m']/1000,o['y_m']/1000),o['radius_m']/1000,color=colors[o['type']],alpha=.65))
        ax.annotate(str(o['id']),(o['x_m']/1000,o['y_m']/1000),xytext=(0,13),textcoords='offset points',ha='center',weight='bold')
    ax.set(xlabel='Восток X, км',ylabel='Север Y, км',title='Семь препятствий на маршруте')
    ax.set_aspect('equal',adjustable='datalim');ax.legend(loc='lower left',fontsize=9)


def save_figures(r,streams,nav,report,out):
    style();out=Path(out);out.mkdir(parents=True,exist_ok=True)
    paths={}
    def save(fig,name):
        fig.savefig(out/f'{name}.png',bbox_inches='tight');plt.close(fig);paths[name]=str(out/f'{name}.png')
    fig,ax=plt.subplots(figsize=(12,5));route_map(ax,r);fig.tight_layout();save(fig,'route_map')
    fig,axes=plt.subplots(2,1,figsize=(12,7),sharex=True)
    km=r['s']/1000
    axes[0].fill_between(km,r['terrain'],color='#c9d6c5',label='Рельеф')
    axes[0].plot(km,r['terrain']+r['canopy'],color='#527d4b',lw=1,label='Верхушки деревьев')
    axes[0].plot(km,r['z'],color=TEAL,lw=2,label='Абсолютная высота БПЛА')
    axes[0].set(ylabel='AMSL, м',title='Огибание рельефа и три подъёма до 90 м AGL');axes[0].legend(ncol=3,fontsize=9)
    axes[1].plot(km,r['agl'],color=TEAL,label='Высота AGL, м');axes[1].plot(km,r['speed']*3.6,color=ORANGE,label='Скорость, км/ч')
    axes[1].set(xlabel='Пройденный путь, км',ylabel='Значение');axes[1].legend(ncol=2,fontsize=9)
    fig.tight_layout();save(fig,'flight_profile')
    fig,axes=plt.subplots(2,1,figsize=(11,6),sharex=True);minutes=r['t']/60
    axes[0].plot(minutes,r['fuel_kg'],color=TEAL,lw=2,label='Топливо на маршрутном участке')
    axes[0].axhline(report['energy']['reserve_kg'],color=RED,ls='--',label='Резерв 3% начального топлива')
    e=report['energy']
    axes[0].set(ylabel='Топливо, кг',title=f"Заправка {e['initial_fuel_kg']:.2f} кг; после посадки {e['final_fuel_kg']:.2f} кг")
    axes[0].legend(fontsize=9)
    axes[1].plot(minutes,r['power_kw'],color=ORANGE,lw=2,label='Потребная мощность')
    axes[1].axhline(report['energy']['max_power_kw'],color=RED,ls='--',label='Допущение о пределе двигателя')
    axes[1].set(xlabel='Время маршрутного участка, мин',ylabel='Мощность, кВт');axes[1].legend(fontsize=9)
    fig.tight_layout();save(fig,'energy')
    if nav:
        fig,axes=plt.subplots(2,1,figsize=(11,6),sharex=True)
        axes[0].plot(nav['t'][::20]/60,nav['error'][::20],color=TEAL,lw=1.5,label='БИНС с коррекцией GNSS')
        axes[1].plot(nav['t'][::20]/60,nav['pure_error'][::20],color=ORANGE,lw=1.5,label='БИНС без коррекции')
        for ax in axes:
            ax.set(ylabel='Ошибка позиции, м');ax.legend(fontsize=9)
            cfg=report['configuration']['sensors']
            if cfg['dropout_end_s']>cfg['dropout_start_s']:ax.axvspan(cfg['dropout_start_s']/60,cfg['dropout_end_s']/60,color=RED,alpha=.15)
        axes[0].set_title('Рост ошибки при потере спутниковой коррекции');axes[1].set_xlabel('Время, мин')
        fig.tight_layout();save(fig,'navigation')
    fig,axes=plt.subplots(2,2,figsize=(12,6))
    for ax,sid,col,label in zip(axes.flat,['IMU1','FUEL_FLOW','LIDAR_DOWN','TEMP_BATT'],[0,0,0,0],['Ускорение X, м/с²','Расход, кг/с','Дальность до крон, м','Температура батареи, °C']):
        a=streams[sid];step=max(1,int(a['rate_hz']/5));ax.plot(a['t'][::step]/60,a['values'][::step,col],color=TEAL,lw=.8)
        ax.set(title=sid,xlabel='Время, мин',ylabel=label)
    fig.tight_layout();save(fig,'sensor_signals')
    return paths
