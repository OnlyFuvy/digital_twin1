"""Проверка команд запуска, отказов, записи и проигрывателя без экрана."""
import json
import os
import subprocess
import sys
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from dtwin.config import load_config


def main():
    root=Path(__file__).resolve().parents[1]
    work=Path('results/run/program_check')
    (root/work).mkdir(parents=True,exist_ok=True)
    cfg=load_config();cfg['energy']['tank_capacity_l']=1.
    (root/work/'small_tank.json').write_text(json.dumps(cfg,ensure_ascii=False),encoding='utf-8')
    cases=[]
    def mission(name,entry=('-m','dtwin'),command='simulate',extra=(),expected=0):
        args=[*entry,command,'--no-navigation','--output',str(work/name),*extra]
        cases.append((name,args,expected,work/name))
    mission('nominal_record',extra=('--record',))
    mission('low_fuel',extra=('--scenario','low_fuel'),expected=2)
    mission('strong_wind',extra=('--scenario','strong_wind'),expected=2)
    mission('fuel_60',extra=('--fuel-kg','60'))
    mission('root_entry',entry=('main.py',))
    mission('legacy_entry',entry=('src/main.py',))
    mission('gui_nominal',command='gui')
    mission('gui_stress_record',command='gui',extra=('--scenario','strong_wind','--record'),expected=2)
    for name,args,expected in (
        ('help',['-m','dtwin','--help'],0),
        ('replay',['-m','dtwin','replay','results/reference'],0),
        ('memory_5_hours',['-m','dtwin','memory','--hours','5'],0),
        ('invalid_duration',['-m','dtwin','memory','--hours','-1'],2),
        ('invalid_fuel',['-m','dtwin','simulate','--fuel-kg','-1'],2),
        ('nan_fuel',['-m','dtwin','simulate','--fuel-kg','nan'],2),
        ('overweight',['-m','dtwin','simulate','--fuel-kg','1000'],2),
        ('tank_overflow',['-m','dtwin','simulate','--config',str(work/'small_tank.json')],2),
    ):
        cases.append((name,args,expected,None))
    rows=[]
    for name,args,expected,directory in cases:
        process=subprocess.run([sys.executable,*args],cwd=root,
            env={**os.environ,'MPLBACKEND':'Agg','PYTHONIOENCODING':'utf-8'},
            capture_output=True,text=True,encoding='utf-8',timeout=120)
        passed=process.returncode==expected
        row=dict(name=name,command='python '+' '.join(args),
                 expected_exit_code=expected,actual_exit_code=process.returncode,passed=passed)
        if directory is not None and passed:
            report=json.loads((root/directory/'results.json').read_text(encoding='utf-8'))
            assert report['navigation'] is None,'Флаг --no-navigation не применён'
            assert report['energy']['final_fuel_kg']>=0,'Отрицательный остаток топлива'
            assert report['passed']==(expected==0),'Код завершения не соответствует проверкам миссии'
            row['failed_checks']=[key for key,value in report['checks'].items() if not value]
            if name=='low_fuel':
                assert row['failed_checks']==['fuel_reserve_3_percent']
                assert report['energy']['fuel_shortfall_kg']>0
            if name in ('strong_wind','gui_stress_record'):
                assert row['failed_checks']==['engine_power']
                assert report['energy']['reserve_passed']
            if '--record' in args:
                manifest=json.loads((root/directory/'telemetry/manifest.json').read_text(encoding='utf-8'))
                actual=sum((root/directory/'telemetry'/f"{item['id']}.bin").stat().st_size for item in manifest['streams'])
                assert len(manifest['streams'])==25
                assert actual==manifest['total_bytes']==report['memory']['binary_bytes']
                assert report['checks']['recording_size_matches']
                row['verified_binary_bytes']=actual
        if name=='memory_5_hours' and passed:
            budget=json.loads(process.stdout)
            assert len(budget['streams'])==25
            assert budget['binary_bytes']==sum(item['total_bytes'] for item in budget['streams'])
            row['binary_bytes']=budget['binary_bytes']
        if not passed:row['output']=process.stdout+process.stderr
        rows.append(row)
        print(f"{'OK' if passed else 'ОШИБКА'}: {name}, код {process.returncode}",flush=True)
    result=dict(python=sys.version.split()[0],backend='Agg',checks=rows,passed=all(row['passed'] for row in rows))
    (root/'results/program_checks.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
