"""Smooth terrain-following prescribed trajectory, not a flight controller."""
import numpy as np


def terrain(x, y):
    x, y = np.asarray(x), np.asarray(y)
    return (120 + 65*np.exp(-((x-10500)/2200)**2-((y+300)/2500)**2)
            + 14*np.sin(x/1600)*np.cos(y/2100))


def canopy(x, y):
    # Synthetic river corridors; tree crowns elsewhere, no real geodata.
    x, y = np.asarray(x), np.asarray(y)
    river = np.exp(-((x-12500)/230)**4)
    return (19 + 3*np.cos(x/900)*np.cos(y/700))*(1-river)


def bump(x, center, width):
    u = np.clip((np.asarray(x)-center)/width, -1, 1)
    return np.cos(np.pi*u/2)**4


def length(points):
    return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())


def build_route(c):
    r, env = c['route'], c['environment']
    L, w, h = r['baseline_m'], r['half_width_m'], r['agl_m']
    centers = np.arange(1,8)*L/8
    n = int(np.ceil(L/r['spatial_step_m']))+1
    x = np.unique(np.r_[np.linspace(0,L,n),centers,centers-w,centers+w])
    y = np.zeros_like(x); agl = np.full_like(x,h)
    base = np.column_stack((x,y,terrain(x,y)+h))
    obstacles = []
    for i, (cx, pct, kind, side) in enumerate(zip(centers,r['extra_percent'],r['types'],r['sides'])):
        ix = np.where((x >= cx-w)&(x <= cx+w))[0]
        xx = x[ix]; b = bump(xx,cx,w)
        hh = h*(1+.5*b) if kind == 'over' else np.full_like(xx,h)
        baseline = length(base[ix]); target = baseline+L*pct/100
        def points(a):
            yy = side*a*b
            return np.column_stack((xx,yy,terrain(xx,yy)+hh))
        lo, hi = 0., L
        if length(points(lo)) > target: raise ValueError('Вертикальный подъём превышает заданную добавку пути')
        for _ in range(60):
            mid = (lo+hi)/2
            if length(points(mid)) < target: lo = mid
            else: hi = mid
        amp = (lo+hi)/2
        y[ix] = side*amp*b; agl[ix] = hh
        cy = side*amp if kind in ('over','bridge') else 0.
        # Obstacles are explicit synthetic scenario objects, configured once
        # after solving the prescribed manoeuvre geometry.
        obstacles.append(dict(id=i+1,type=kind,x_m=float(cx),y_m=float(cy),
            half_width_m=w,side=side,amplitude_m=amp,target_percent=pct,
            baseline_length_m=baseline,length_m=length(points(amp)),
            extra_m=length(points(amp))-baseline,actual_percent=(length(points(amp))-baseline)/L*100,
            radius_m=env['nfz_radius_m'] if kind=='no_fly' else env['obstacle_radius_m'],
            height_m=env['obstacle_height_m'],peak_agl_m=float(hh.max())))
    z = terrain(x,y)+agl
    return dict(x=x,y=y,z=z,agl=agl,terrain=terrain(x,y),canopy=canopy(x,y),
                obstacles=obstacles,baseline_length_m=length(base))


def time_parameterize(route,c):
    p = np.column_stack([route[k] for k in ('x','y','z')])
    ds = np.linalg.norm(np.diff(p,axis=0),axis=1)
    s = np.r_[0,np.cumsum(ds)]
    tangent = np.gradient(p,s,axis=0,edge_order=2)
    curvature = np.linalg.norm(np.gradient(tangent,s,axis=0,edge_order=2),axis=1)
    vcfg = c['vehicle']
    # 1% speed margin absorbs finite-difference curvature error on the grid.
    limit = np.minimum(vcfg['cruise_ms'],.99*np.sqrt(9.80665*np.tan(np.deg2rad(vcfg['max_bank_deg']))/np.maximum(curvature,1e-9)))
    limit = np.minimum(limit,vcfg['max_climb_ms']/np.maximum(np.abs(tangent[:,2]),1e-9))
    for ob in route['obstacles']:
        mask = np.abs(route['x']-ob['x_m']) <= ob['half_width_m']
        limit[mask] = np.minimum(limit[mask],12. if ob['type']=='bridge' else 23.)
    # The route is an airborne mission segment. Vertical takeoff/landing
    # and their energy are separate holds in the energy model.
    speed = limit.copy(); speed[0]=speed[-1]=5.
    a = vcfg['max_accel_ms2']
    for i in range(1,len(speed)): speed[i]=min(speed[i],np.sqrt(speed[i-1]**2+2*a*ds[i-1]))
    for i in range(len(speed)-2,-1,-1): speed[i]=min(speed[i],np.sqrt(speed[i+1]**2+2*a*ds[i]))
    t = np.r_[0,np.cumsum(2*ds/(speed[1:]+speed[:-1]))]
    vel = np.gradient(p,t,axis=0,edge_order=2)
    acc = np.gradient(vel,t,axis=0,edge_order=2)
    yaw=np.unwrap(np.arctan2(vel[:,1],vel[:,0]))
    pitch=-np.arctan2(vel[:,2],np.linalg.norm(vel[:,:2],axis=1))
    yaw_rate=np.gradient(yaw,t,edge_order=2)
    roll=-np.arctan2(np.linalg.norm(vel[:,:2],axis=1)*yaw_rate,9.80665)
    route.update(t=t,s=s,speed=speed,velocity=vel,acceleration=acc,
                 euler=np.column_stack((roll,pitch,yaw)),curvature=curvature)
    return route


def collision_checks(r,c):
    env,v=c['environment'],c['vehicle']; radius=v['rotor_radius_m']; half_h=v['body_half_height_m']
    checks=[]
    for ob in r['obstacles']:
        d=np.hypot(r['x']-ob['x_m'],r['y']-ob['y_m'])
        if ob['type']=='no_fly':
            margin=float(d.min()-ob['radius_m']-radius); ok=margin>0
        elif ob['type'] in ('over','lateral'):
            overlap=d<=ob['radius_m']+radius
            top=terrain(ob['x_m'],ob['y_m'])+ob['height_m']
            margin=float(np.min(r['z'][overlap]-half_h-top)) if overlap.any() else float(d.min()-ob['radius_m']-radius)
            ok=margin>0
        else:
            mask=np.abs(r['x']-ob['x_m'])<=env['bridge_half_thickness_m']+radius
            deck=terrain(ob['x_m'],ob['y_m'])+env['bridge_bottom_agl_m']
            vertical=float(np.min(deck-r['z'][mask]-half_h))
            lateral=float(np.min(env['bridge_opening_width_m']/2-np.abs(r['y'][mask]-ob['y_m'])-radius))
            margin=min(vertical,lateral);ok=margin>0 and ob['actual_percent']<=4.00001
        checks.append(dict(id=ob['id'],type=ob['type'],passed=bool(ok),clearance_m=margin))
    clearance=r['agl']-r['canopy']-half_h
    return checks, float(clearance.min())
