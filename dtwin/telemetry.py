"""Deterministic binary stream format and exact sample-based memory budget."""
import json
from pathlib import Path
import numpy as np


def record_dtype(components):
    return np.dtype([('t','<f8'),('channel','<u2'),('components','<u2'),('quality','<u4'),('values','<f4',(components,))])


def memory_budget(duration,specs,c):
    if duration<0 or not np.isfinite(duration):raise ValueError('Длительность должна быть конечной и неотрицательной')
    rows=[]
    for s in specs+[dict(id='STATE',rate_hz=c['telemetry']['state_hz'],components=c['telemetry']['state_components'],type='STATE')]:
        rate=c['sensors']['imu_hz'] if s['type']=='IMU' else s['rate_hz']
        count=int(np.floor(duration*rate))+1;payload=s['components']*4;packet=16+payload
        rows.append(dict(id=s['id'],rate_hz=rate,components=s['components'],payload_bytes=payload,
            packet_bytes=packet,sample_count=count,total_bytes=count*packet,bytes_per_second=rate*packet))
    total=sum(x['total_bytes'] for x in rows)
    return dict(streams=rows,bytes_per_second=sum(x['bytes_per_second'] for x in rows),binary_bytes=total,
        binary_mib=total/2**20,reserved_mib=total*c['telemetry']['storage_margin']/2**20,
        margin=c['telemetry']['storage_margin'],duration_s=duration)


def write_stream(path,channel,t,values,quality):
    values=np.asarray(values);a=np.empty(len(t),dtype=record_dtype(values.shape[1]))
    a['t']=t;a['channel']=channel;a['components']=values.shape[1];a['quality']=quality;a['values']=values
    a.tofile(path)
    return a.nbytes


def read_stream(path,components):
    path=Path(path);dtype=record_dtype(components)
    if path.stat().st_size%dtype.itemsize:raise ValueError('Оборванная запись телеметрии')
    result=np.fromfile(path,dtype=dtype)
    if np.any(result['components']!=components):raise ValueError('Неверная размерность записи')
    if np.any(np.diff(result['t'])<0):raise ValueError('Время записи не монотонно')
    return result


def save_streams(directory,streams,state,c):
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True);manifest=[]
    for channel,(sid,s) in enumerate(streams.items(),1):
        size=write_stream(directory/f'{sid}.bin',channel,s['t'],s['values'],s['quality'])
        manifest.append(dict(id=sid,channel=channel,components=s['values'].shape[1],rate_hz=s['rate_hz'],
            samples=len(s['t']),bytes=size,units=s['units'],invalid_samples=int(np.count_nonzero(s['quality']))))
    size=write_stream(directory/'STATE.bin',25,state['t'],state['values'],np.zeros(len(state['t']),dtype=np.uint32))
    manifest.append(dict(id='STATE',channel=25,components=16,rate_hz=c['telemetry']['state_hz'],samples=len(state['t']),bytes=size,units=state['units'],invalid_samples=0))
    (directory/'manifest.json').write_text(json.dumps(dict(format='little endian: float64 t, uint16 channel, uint16 components, uint32 quality, float32 values[N]',
        streams=manifest,total_bytes=sum(x['bytes'] for x in manifest)),ensure_ascii=False,indent=2),encoding='utf-8')
    return manifest
