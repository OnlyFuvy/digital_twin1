"""24 multirate channels with noise, body lever arms and explicit validity."""
import numpy as np
from .route import terrain, canopy


def interpolate(t, source_t, values):
    values=np.asarray(values)
    if values.ndim==1: return np.interp(t,source_t,values)
    return np.column_stack([np.interp(t,source_t,values[:,j]) for j in range(values.shape[1])])


def rotations(euler):
    roll,pitch,yaw=np.asarray(euler).T
    cr,sr=np.cos(roll),np.sin(roll);cp,sp=np.cos(pitch),np.sin(pitch);cy,sy=np.cos(yaw),np.sin(yaw)
    out=np.empty((len(roll),3,3))
    out[:,0,0]=cy*cp;out[:,0,1]=cy*sp*sr-sy*cr;out[:,0,2]=cy*sp*cr+sy*sr
    out[:,1,0]=sy*cp;out[:,1,1]=sy*sp*sr+cy*cr;out[:,1,2]=sy*sp*cr-cy*sr
    out[:,2,0]=-sp;out[:,2,1]=cp*sr;out[:,2,2]=cp*cr
    return out


def body_rates(euler,t):
    dr,dp,dy=np.gradient(euler,t,axis=0,edge_order=2).T
    roll,pitch,_=euler.T
    return np.column_stack((dr-dy*np.sin(pitch),dp*np.cos(roll)+dy*np.sin(roll)*np.cos(pitch),
                            -dp*np.sin(roll)+dy*np.cos(roll)*np.cos(pitch)))


def front_lidar(pos,direction,r,c):
    max_range=200.
    ranges=np.full(len(pos),max_range)
    # Ray march against the synthetic surface, 2.5 m resolution.
    for distance in np.arange(2.5,max_range+.1,2.5):
        q=pos+direction*distance
        hit=q[:,2] <= terrain(q[:,0],q[:,1])+canopy(q[:,0],q[:,1])
        for ob in r['obstacles']:
            if ob['type'] in ('over','lateral'):
                hit |= ((q[:,0]-ob['x_m'])**2+(q[:,1]-ob['y_m'])**2 < ob['radius_m']**2) & (q[:,2]<terrain(ob['x_m'],ob['y_m'])+ob['height_m'])
        ranges=np.where(hit & (ranges==max_range),distance,ranges)
    return ranges


def simulate_sensors(r,c,specs):
    streams={}; cfg=c['sensors']
    rt=r['t']; pos=np.column_stack([r[k] for k in ('x','y','z')])
    angular=body_rates(r['euler'],rt);angular_acc=np.gradient(angular,rt,axis=0,edge_order=2)
    for idx,spec in enumerate(specs):
        sid=spec['id'];rate=spec['rate_hz']
        if spec['type']=='IMU':rate=cfg['imu_hz']
        t=np.arange(int(np.floor(rt[-1]*rate))+1)/rate
        rng=np.random.default_rng(np.random.SeedSequence([c['seed'],idx]))
        eul=interpolate(t,rt,r['euler']);R=rotations(eul)
        arm=np.array([spec[k] for k in ('x','y','z')]);offset=np.einsum('nij,j->ni',R,arm)
        p=interpolate(t,rt,pos)+offset
        valid=np.zeros(len(t),dtype=np.uint32)  # 0=valid, 1=dropout, 2=out-of-range
        def sample(key):return interpolate(t,rt,r[key])
        if sid.startswith('IMU'):
            omega=interpolate(t,rt,angular);alpha=interpolate(t,rt,angular_acc)
            accel=sample('acceleration')-np.array([0,0,-9.80665])
            fb=np.einsum('nji,nj->ni',R,accel)+np.cross(alpha,arm)+np.cross(omega,np.cross(omega,arm))
            bias=np.array(cfg['bias_accel_ms2'])*(1 if sid=='IMU1' else -.8)
            gbias=np.array(cfg['bias_gyro_rads'])*(1 if sid=='IMU1' else -.8)
            values=np.column_stack((fb+bias+rng.normal(0,cfg['sigma_accel_ms2'],fb.shape),
                omega+gbias+rng.normal(0,cfg['sigma_gyro_rads'],omega.shape)))
            units=['m/s2']*3+['rad/s']*3
        elif sid in ('GNSS1','GNSS2'):
            values=p+rng.normal(0,cfg['gnss_sigma_m'],p.shape)
            fail=(t>=cfg['dropout_start_s'])&(t<cfg['dropout_end_s'])
            values[fail]=np.nan;valid[fail]=1;units=['m']*3
        elif sid in ('BARO1','BARO2'):
            values=101325*np.maximum(1-p[:,2]/44330,1e-5)**5.255+rng.normal(0,2.,len(t));units=['Pa']
        elif sid in ('MAG1','MAG2'):
            values=np.einsum('nji,j->ni',R,np.array([18.,2.,-48.]))+rng.normal(0,.15,(len(t),3));units=['uT']*3
        elif sid in ('LIDAR_DOWN','RADAR_ALT'):
            surface=terrain(p[:,0],p[:,1])+(canopy(p[:,0],p[:,1]) if sid=='LIDAR_DOWN' else 0.)
            values=(p[:,2]-surface)/np.maximum(R[:,2,2],.1)+rng.normal(0,.05 if sid=='LIDAR_DOWN' else .3,len(t))
            valid[values>(120 if sid=='LIDAR_DOWN' else 1500)]=2;units=['m']
        elif sid=='LIDAR_FRONT':
            values=front_lidar(p,R[:,:,0],r,c);valid[values>=200]=2
            values+=rng.normal(0,.05,len(t));units=['m']
        elif sid=='RPM':values=5200+6*sample('power_kw')+rng.normal(0,8,len(t));units=['rpm']
        elif sid=='PWR':values=np.column_stack((sample('power_kw')*1000,sample('voltage'),sample('current')));units=['W','V','A']
        elif sid=='ADC_BATT':values=np.rint(np.clip(sample('voltage')/36,0,1)*4095);units=['count']
        elif sid=='TEMP_BATT':values=sample('temp_batt')+rng.normal(0,.2,len(t));units=['degC']
        elif sid=='CURRENT_MOTOR':
            # Legacy identifier retained: actuator current, not ICE shaft current.
            values=(80+20*np.abs(eul[:,0]))/sample('voltage')+rng.normal(0,.02,len(t));units=['A']
        elif sid=='VOLTAGE_BUS':values=sample('voltage')+rng.normal(0,.02,len(t));units=['V']
        elif sid=='FUEL_FLOW':values=np.maximum(0,sample('fuel_flow_kgs')+rng.normal(0,.00001,len(t)));units=['kg/s']
        elif sid=='SOC_EST':
            # Coulomb counter derived from the measured POWER current channel.
            power=streams['PWR'];pt=power['t'];amps=power['values'][:,2]
            charge=np.r_[0,np.cumsum((amps[1:]+amps[:-1])/2*np.diff(pt))]
            values=np.clip(r['soc'][0]-np.interp(t,pt,charge)/(c['energy']['battery_wh']/28*3600),0,1);units=['fraction']
        elif sid=='AIRSPEED':values=np.maximum(0,sample('airspeed')+rng.normal(0,.2,len(t)));units=['m/s']
        elif sid=='VIBRO':values=.12+.002*sample('power_kw')+.015*np.sin(2*np.pi*18*t)+rng.normal(0,.008,len(t));units=['g_rms']
        elif sid=='TEMP_IMU':values=c['environment']['temperature_c']+25+8*(1-np.exp(-t/400))+rng.normal(0,.1,len(t));units=['degC']
        elif sid=='GNSS_VEL':
            gnss=streams['GNSS1']; values=np.gradient(gnss['values'],gnss['t'],axis=0)
            valid=np.where(np.isfinite(values).all(axis=1),0,1).astype(np.uint32);units=['m/s']*3
        elif sid=='BARO_ALT':
            baro=streams['BARO1']; values=44330*(1-(baro['values'][:,0]/101325)**(1/5.255));units=['m']
        else:raise ValueError(f'Нереализованный канал {sid}')
        values=np.asarray(values)
        if values.ndim==1:values=values[:,None]
        streams[sid]=dict(t=t,values=values,quality=valid,rate_hz=rate,units=units,spec=spec)
    return streams
