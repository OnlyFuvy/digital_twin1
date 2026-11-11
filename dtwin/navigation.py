"""Strapdown propagation with independent GNSS alpha-beta correction.

Attitude starts aligned and is propagated from gyro measurements only.
No true positions/velocities are injected after initialization.
This is a complementary filter demonstration, not a certified EKF.
"""
import numpy as np
from .sensors import interpolate,rotations


def navigate(r,streams,c):
    imu=streams['IMU1'];gnss=streams['GNSS1'];t=imu['t'];n=len(t)
    pos=np.empty((n,3));vel=np.empty((n,3));pure=np.empty((n,3))
    true_pos=interpolate(t,r['t'],np.column_stack((r['x'],r['y'],r['z'])))
    pos[0]=pure[0]=true_pos[0];vel[0]=r['velocity'][0]
    pv=vel[0].copy();R=rotations(r['euler'][:1])[0]
    # Matrix propagation with Rodrigues increments and periodic orthonormalization.
    next_fix=1;last_fix=0.;accepted=0;rejected=0
    positions=gnss['values'];arm=np.array([gnss['spec'][k] for k in ('x','y','z')])
    for i in range(1,n):
        dt=t[i]-t[i-1];measure=(imu['values'][i]+imu['values'][i-1])/2
        a=R@measure[:3]+np.array([0.,0.,-9.80665])
        pos[i]=pos[i-1]+vel[i-1]*dt+.5*a*dt*dt;vel[i]=vel[i-1]+a*dt
        pure[i]=pure[i-1]+pv*dt+.5*a*dt*dt;pv+=a*dt
        wx,wy,wz=measure[3:]*dt
        K=np.array([[0,-wz,wy],[wz,0,-wx],[-wy,wx,0]])
        angle=np.linalg.norm([wx,wy,wz]);a1=np.sinc(angle/np.pi);a2=.5*np.sinc(angle/(2*np.pi))**2
        R=R@(np.eye(3)+a1*K+a2*K@K)
        if i%1000==0:
            u,_,vt=np.linalg.svd(R);R=u@vt
        while next_fix<len(gnss['t']) and gnss['t'][next_fix]<=t[i]+1e-9:
            if gnss['quality'][next_fix]==0:
                residual=positions[next_fix]-R@arm-pos[i]
                gap=gnss['t'][next_fix]-last_fix
                if np.linalg.norm(residual)<5000:
                    # No velocity kick after an outage: reacquire position first.
                    alpha=.12 if gap<1 else .8
                    beta=.002 if gap<1 else 0.
                    pos[i]+=alpha*residual;vel[i]+=beta/max(gap,.05)*residual
                    accepted+=1;last_fix=gnss['t'][next_fix]
                else: rejected+=1
            next_fix+=1
    error=np.linalg.norm(pos-true_pos,axis=1);pure_error=np.linalg.norm(pure-true_pos,axis=1)
    fail=(t>=c['sensors']['dropout_start_s'])&(t<c['sensors']['dropout_end_s'])
    healthy=~fail
    return dict(t=t,estimated=pos,true=true_pos,error=error,pure_error=pure_error,
        metrics=dict(rmse_m=float(np.sqrt(np.mean(error**2))),max_error_m=float(error.max()),
        healthy_rmse_m=float(np.sqrt(np.mean(error[healthy]**2))),
        outage_peak_m=float(error[fail].max()) if fail.any() else 0.,
        final_error_m=float(error[-1]),pure_final_error_m=float(pure_error[-1]),
        accepted_gnss_fixes=accepted,rejected_gnss_fixes=rejected))
