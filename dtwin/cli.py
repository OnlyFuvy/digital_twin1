"""Командный интерфейс для Windows, Linux и macOS: python -m dtwin."""
import argparse
import json
from pathlib import Path
from .config import load_config,scenario,validate


def main(argv=None):
    parser=argparse.ArgumentParser(description='Учебный цифровой двойник БПЛА ВТ-440')
    sub=parser.add_subparsers(dest='command',required=True)
    for name in ('simulate','gui'):
        p=sub.add_parser(name);p.add_argument('--config');p.add_argument('--scenario',choices=['nominal','gnss_dropout','low_fuel','strong_wind'],default='nominal')
        p.add_argument('--output',default='results/run');p.add_argument('--record',action='store_true',help='Записать полную двоичную телеметрию')
        p.add_argument('--no-navigation',action='store_true',help='Быстрый расчёт без интегрирования БИНС')
        p.add_argument('--fuel-kg',type=float,help='Начальная заправка в кг; по умолчанию 10 кг, в low_fuel — 1 кг')
    p=sub.add_parser('memory');p.add_argument('--hours',type=float,default=5)
    p=sub.add_parser('replay');p.add_argument('directory',nargs='?',default='results/reference')
    args=parser.parse_args(argv)
    try:
        if args.command=='memory':
            from .config import load_sensors
            from .telemetry import memory_budget
            print(json.dumps(memory_budget(args.hours*3600,load_sensors(),load_config()),ensure_ascii=False,indent=2));return 0
        if args.command=='replay':
            from .gui import replay
            replay(Path(args.directory));return 0
        c=scenario(load_config(args.config),args.scenario)
        if args.fuel_kg is not None:
            c['vehicle']['fuel_kg']=args.fuel_kg
            validate(c)
        if args.command=='gui':
            from .gui import launch
            report=launch(c,Path(args.output),not args.no_navigation,args.record)
            return 0 if report['passed'] else 2
        from .simulation import simulate
        from .plots import save_figures
        r,s,n,report=simulate(c,args.output,not args.no_navigation,args.record)
        save_figures(r,s,n,report,Path(args.output)/'figures')
        print(f"Маршрут: {report['route']['total_3d_m']/1000:.3f} км; полное время: {report['energy']['total_duration_s']/60:.2f} мин")
        e=report['energy']
        print(f"Заправка: {e['initial_fuel_kg']:.3f} кг (около {e['initial_fuel_l']:.2f} л); потребность плана: {e['fuel_required_kg']:.3f} кг")
        print(f"Остаток после посадки: {e['final_fuel_kg']:.3f} кг ({e['final_fuel_percent']:.2f}%); минимум: {e['reserve_kg']:.3f} кг")
        if e['fuel_shortfall_kg']>0:print(f"Дефицит топлива для плана: {e['fuel_shortfall_kg']:.3f} кг")
        if not e['tank_capacity_checked']:print('Паспортный объём бака не подтверждён; проверка вместимости не выполнена')
        print(f"Каналы: {len(s)}; двоичная запись: {report['memory']['binary_mib']:.2f} МиБ")
        print('Проверки: '+', '.join(k for k,v in report['checks'].items() if not v) if not report['passed'] else 'Все проверки сценария пройдены')
        print(f'Результаты: {Path(args.output).resolve()}')
        return 0 if report['passed'] else 2
    except (ValueError,KeyError,FileNotFoundError) as exc:
        parser.error(str(exc))


if __name__=='__main__':raise SystemExit(main())
