"""Continuous trajectory builder; no ROS or AI dependencies."""
import math
DT=.04

def build_motion(model, targets, motion, velocity_limit=1.5, acceleration_limit=4.):
    xyz = {p['name']:p['xyz'] for p in targets['points']}
    rows=[]
    def add(t,q,label):
        if any(not lo+.01 <= x <= hi-.01 for x,(lo,hi) in zip(q,model.limits)):
            raise ValueError(f'{label}: joint limit exceeded')
        rows.append({'t':t,'q':q,'label':label})
    t=0.
    add(t,model.ik(xyz['A'],math.radians(motion['points']['A']['pitch_deg'])),'A')
    def hold(name):
        nonlocal t
        seconds=motion['points'][name]['hold_s']
        if seconds:
            t+=seconds
            add(t,list(rows[-1]['q']),f'hold {name}')
    hold('A')
    endpoint_indices={'A':0}
    for edge,spec in motion['segments'].items():
        a,b=edge
        duration=spec['duration_s']
        count=max(20,math.ceil(duration/DT),spec['oscillation_cycles']*40)
        p0,p1=[motion['points'][n]['pitch_deg'] for n in edge]
        control=2*spec['midpoint_pitch_deg']-(p0+p1)/2
        for k in range(1,count+1):
            u=k/count
            progress=10*u**3-15*u**4+6*u**5
            p=[x+(y-x)*progress for x,y in zip(xyz[a],xyz[b])]
            pitch=(1-progress)**2*p0+2*progress*(1-progress)*control+progress**2*p1
            q=model.ik(p,math.radians(pitch))
            q[3]+=math.radians(spec['joint4_amplitude_deg'])*math.sin(2*math.pi*spec['oscillation_cycles']*progress)
            add(t+duration*u,q,edge)
        t+=duration
        endpoint_indices[b]=len(rows)-1
        hold(b)
    # Derivatives for controller interpolation, estimated from the smooth samples.
    stops=set(endpoint_indices.values())|{0,len(rows)-1}
    stops.update(i for i,r in enumerate(rows) if r['label'].startswith('hold '))
    for i,r in enumerate(rows):
        if i in stops:
            r['v']=[0.]*4
        else:
            before,after=rows[i-1],rows[i+1]
            r['v']=[(b-a)/(after['t']-before['t']) for a,b in zip(before['q'],after['q'])]
    max_v=[0.]*4; max_a=[0.]*4
    for i,r in enumerate(rows):
        if i in stops:r['a']=[0.]*4
        else:
            before,after=rows[i-1],rows[i+1]
            r['a']=[(b-a)/(after['t']-before['t']) for a,b in zip(before['v'],after['v'])]
        for j in range(4):
            max_v[j]=max(max_v[j],abs(r['v'][j]))
            max_a[j]=max(max_a[j],abs(r['a'][j]))
        if i:
            delta=r['t']-rows[i-1]['t']
            if delta<=0: raise ValueError('Non-increasing trajectory time')
            for j in range(4):
                max_v[j]=max(max_v[j],abs(r['q'][j]-rows[i-1]['q'][j])/delta)
                max_a[j]=max(max_a[j],abs(r['v'][j]-rows[i-1]['v'][j])/delta)
    # Position+velocity trajectory points use cubic Hermite interpolation.
    # Check its position extrema and exact velocity/acceleration extrema per interval.
    for before,after in zip(rows,rows[1:]):
        h=after['t']-before['t']
        for j in range(4):
            p0,p1,v0,v1=before['q'][j],after['q'][j],before['v'][j],after['v'][j]
            a=2*p0-2*p1+h*(v0+v1)
            b=-3*p0+3*p1-h*(2*v0+v1)
            c=h*v0
            probes=[0.,1.]
            if abs(a)>1e-15:
                probes.append(-b/(3*a))
                disc=4*b*b-12*a*c
                if disc>=0:
                    probes.extend([(-2*b+math.sqrt(disc))/(6*a),(-2*b-math.sqrt(disc))/(6*a)])
            elif abs(b)>1e-15:
                probes.append(-c/(2*b))
            for u in probes:
                if 0<=u<=1:
                    pos=((a*u+b)*u+c)*u+p0
                    lo,hi=model.limits[j]
                    if not lo+.01<=pos<=hi-.01:
                        raise ValueError('Controller interpolation exceeds joint limits')
                    max_v[j]=max(max_v[j],abs((3*a*u*u+2*b*u+c)/h))
                    max_a[j]=max(max_a[j],abs((6*a*u+2*b)/(h*h)))
    if max(max_v)>velocity_limit or max(max_a)>acceleration_limit:
        raise ValueError(f'Dynamics check failed: max velocity={max(max_v):.3f} rad/s '
                         f'(limit {velocity_limit}), max acceleration={max(max_a):.3f} rad/s^2 '
                         f'(limit {acceleration_limit}). Increase duration explicitly; parameters were not changed.')
    return rows,{'duration_s':t,'samples':len(rows),'max_joint_velocity_rad_s':max_v,
                 'max_joint_acceleration_rad_s2':max_a,
                 'velocity_check_limit_rad_s':velocity_limit,
                 'acceleration_check_limit_rad_s2':acceleration_limit,
                 'note':'Sampled derivatives; engineering caps, not confirmed actuator acceleration ratings.'}
