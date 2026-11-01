"""Parse live URDF/SRDF; generic FK and a supported CRANE chain IK backend."""
import math
import xml.etree.ElementTree as ET
import numpy as np


def vector(text, default='0 0 0'):
    return np.array([float(x) for x in (text or default).split()])


def rot(axis,angle):
    axis=np.asarray(axis,dtype=float);axis=axis/np.linalg.norm(axis)
    x,y,z=axis;K=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    return np.eye(3)+math.sin(angle)*K+(1-math.cos(angle))*(K@K)


def quaternion(R):
    # eigenvector method, scalar component last
    K=np.array([[R[0,0]-R[1,1]-R[2,2],R[1,0]+R[0,1],R[2,0]+R[0,2],R[2,1]-R[1,2]],
                [R[1,0]+R[0,1],R[1,1]-R[0,0]-R[2,2],R[2,1]+R[1,2],R[0,2]-R[2,0]],
                [R[2,0]+R[0,2],R[2,1]+R[1,2],R[2,2]-R[0,0]-R[1,1],R[1,0]-R[0,1]],
                [R[2,1]-R[1,2],R[0,2]-R[2,0],R[1,0]-R[0,1],R.trace()]])/3
    _,V=np.linalg.eigh(K);q=V[:,-1]
    return (q if q[3]>=0 else -q).tolist()


class RobotModel:
    def __init__(self,urdf,srdf,parameters,config):
        root=ET.fromstring(urdf);semantic=ET.fromstring(srdf)
        chain=semantic.find(f"group[@name='{config['group']}']/chain")
        if chain is None:raise ValueError('Expected SRDF chain group')
        self.base=chain.get('base_link');self.tip=chain.get('tip_link')
        self.joints={};self.child={}
        for element in root.findall('joint'):
            origin=element.find('origin');axis=element.find('axis');limit=element.find('limit')
            j={'name':element.get('name'),'type':element.get('type'),
               'parent':element.find('parent').get('link'),'child':element.find('child').get('link'),
               'xyz':vector(None if origin is None else origin.get('xyz')).tolist(),
               'rpy':vector(None if origin is None else origin.get('rpy')).tolist(),
               'axis':vector('1 0 0' if axis is None else axis.get('xyz')).tolist(),
               'lower':None if limit is None else float(limit.get('lower','-inf')),
               'upper':None if limit is None else float(limit.get('upper','inf')),
               'velocity':None if limit is None or limit.get('velocity') is None else float(limit.get('velocity'))}
            self.joints[j['name']]=j;self.child[j['child']]=j
        link=self.tip;ordered=[]
        while link!=self.base:
            j=self.child[link];ordered.append(j);link=j['parent']
        self.chain=list(reversed(ordered));self.arm=[j for j in self.chain if j['type']!='fixed']
        self.names=[j['name'] for j in self.arm]
        self.limits=[(j['lower'],j['upper']) for j in self.arm]
        self.velocity_caps=[];self.acceleration_caps=[];self.limit_sources=[]
        for j in self.arm:
            prefix='robot_description_planning.joint_limits.'+j['name']+'.'
            v=parameters.get(prefix+'max_velocity') if parameters.get(prefix+'has_velocity_limits',True) else None
            a=parameters.get(prefix+'max_acceleration') if parameters.get(prefix+'has_acceleration_limits',True) else None
            self.velocity_caps.append(min([x for x in [j['velocity'],v,config['velocity_cap']] if x is not None and x>0]))
            self.acceleration_caps.append(min(a,config['acceleration_cap']) if a is not None and a>0 else config['acceleration_cap'])
            lo=parameters.get(prefix+'min_position');hi=parameters.get(prefix+'max_position')
            idx=len(self.velocity_caps)-1
            self.limits[idx]=(max(j['lower'],lo) if lo is not None else j['lower'],min(j['upper'],hi) if hi is not None else j['upper'])
            self.limit_sources.append({'joint':j['name'],'moveit_velocity':v,'moveit_acceleration':a,
                                      'fallback_acceleration_used':a is None or a<=0})
        # Reject other geometries instead of silently applying CRANE-specific IK.
        if len(self.arm)!=4:raise ValueError('This backend supports four arm joints only')
        for j,axis in zip(self.arm,[[0,0,1],[0,1,0],[0,-1,0],[0,-1,0]]):
            if not np.allclose(j['axis'],axis) or not np.allclose(j['rpy'],0) or not np.allclose(j['xyz'][:2],0):
                raise ValueError('Unsupported chain geometry; implement another IK backend')
        if any(not np.allclose(j['xyz'],0) or not np.allclose(j['rpy'],0) for j in self.chain if j['type']=='fixed'):
            raise ValueError('Unsupported fixed tool offset')
        self.height=self.arm[0]['xyz'][2]+self.arm[1]['xyz'][2]
        self.L1=self.arm[2]['xyz'][2];self.L2=self.arm[3]['xyz'][2]
        self.disabled=[dict(x.attrib) for x in semantic.findall('disable_collisions')]

    def check(self,q):
        if len(q)!=4 or any(type(v) not in (float,int) or not math.isfinite(v) for v in q):raise ValueError('Expected four finite joint angles')
        if any(not lo+.001<=v<=hi-.001 for v,(lo,hi) in zip(q,self.limits)):raise ValueError('Joint range exceeded')

    def ik(self,xyz,pitch):
        x,y,z=xyz;r=math.hypot(x,y);h=z-self.height
        c=(r*r+h*h-self.L1**2-self.L2**2)/(2*self.L1*self.L2)
        if not -1<=c<=1:raise ValueError('Unreachable target')
        q3=-math.acos(c)
        q2=math.atan2(r,h)+math.atan2(self.L2*math.sin(q3),self.L1+self.L2*c)
        q=[math.atan2(y,x),q2,q3,q2-q3-pitch];self.check(q);return q

    def poses(self,positions):
        transforms={self.base:np.eye(4)};pending=list(self.joints.values())
        for _ in range(len(pending)+1):
            remaining=[]
            for j in pending:
                if j['parent'] not in transforms:remaining.append(j);continue
                if j['type']!='fixed' and j['name'] not in positions:continue
                T=np.eye(4);r,p,y=j['rpy'];T[:3,3]=j['xyz']
                T[:3,:3]=rot([0,0,1],y)@rot([0,1,0],p)@rot([1,0,0],r)
                if j['type'] in ('revolute','continuous'):T[:3,:3]=T[:3,:3]@rot(j['axis'],positions[j['name']])
                transforms[j['child']]=transforms[j['parent']]@T
            if len(remaining)==len(pending):break
            pending=remaining
        return {name:{'xyz':T[:3,3].tolist(),'quaternion':quaternion(T[:3,:3])} for name,T in transforms.items()}

    def pose(self,q):return self.poses(dict(zip(self.names,q)))[self.tip]
    def context(self):
        return {'base':self.base,'tip':self.tip,'joint_order':self.names,'joints':self.arm,
                'effective_position_limits':self.limits,'velocity_caps':self.velocity_caps,
                'acceleration_caps':self.acceleration_caps,'limit_sources':self.limit_sources,
                'disabled_collision_pairs':self.disabled,'ik_backend':'negative joint3 branch; four-joint CRANE geometry'}
