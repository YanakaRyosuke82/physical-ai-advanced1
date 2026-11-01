"""Mandatory checks are code-owned; AI receives actual results and revises."""
import copy
import json
import math
from .trajectory import build_motion
from ai.schema import validate_parameters
from rosidl_runtime_py.convert import message_to_ordereddict


class Validator:
    def __init__(self,bridge,targets,logs):self.bridge=bridge;self.targets=targets;self.logs=logs;self.accepted=None;self.count=0
    def validate(self,candidate):
        self.accepted=None;self.count+=1;model=self.bridge.model
        try:
            motion=candidate['motion'];validate_parameters(motion,self.targets)
            if motion['status']!='ready':raise ValueError('Motion status is not ready')
            xyz={p['name']:p['xyz'] for p in self.targets['points']}
            checks=[]
            for collection,desired in [('anchors_rad',xyz),('midpoints_rad',{e:[(a+b)/2 for a,b in zip(xyz[e[0]],xyz[e[1]])] for e in ['AB','BC','CD','DE']})]:
                for label,q in candidate[collection].items():
                    model.check(q);pose=model.pose(q)
                    error=math.dist(pose['xyz'],desired[label]);pitch=math.degrees(q[1]-q[2]-q[3])
                    expected=motion['points'][label]['pitch_deg'] if collection=='anchors_rad' else motion['segments'][label]['midpoint_pitch_deg']
                    if error>.001:raise ValueError(f'{collection}.{label}: FK error {error:.6f} m; actual {pose["xyz"]}; target {desired[label]}')
                    if abs(pitch-expected)>.1:raise ValueError(f'{label}: joints imply pitch {pitch:.4f} deg but motion requests {expected}')
                    # Ensure the generated pose belongs to the implemented IK branch.
                    solved=model.ik(desired[label],math.radians(expected))
                    if max(abs(a-b) for a,b in zip(solved,q))>.03:raise ValueError(f'{label}: unsupported IK branch or inconsistent joint pose')
                    state=self.bridge.check_pose(q)
                    if not state['valid']:raise ValueError(f'{label}: collision {state["contacts"]}')
                    checks.append({'label':label,'position_error_m':error,'pitch_deg':pitch})
            rows,stats=build_motion(model,self.targets,motion,min(model.velocity_caps),min(model.acceleration_caps))
            start=self.bridge.snapshot()['joint_positions']
            approach=self.bridge.approach(rows[0]['q'])
            self.bridge.check_model(rows[0]['q'])
            print(f'候補{self.count}: {len(rows)}点の軌道を検証中...',flush=True)
            self.validate_rows(rows)
            plan_id=f'candidate_{self.count:03d}'
            self.accepted={'plan_id':plan_id,'candidate':copy.deepcopy(candidate),'rows':rows,'statistics':stats,
                           'start':start,'approach':approach,'drawing':self.bridge.make_trajectory(rows)}
            serial={k:v for k,v in self.accepted.items() if k not in ('approach','drawing')}
            serial['approach']=message_to_ordereddict(approach);serial['drawing']=message_to_ordereddict(self.accepted['drawing'])
            (self.logs.folder/(plan_id+'.json')).write_text(json.dumps(serial,ensure_ascii=False,indent=2))
            result={'valid':True,'plan_id':plan_id,'anchor_checks':checks,'statistics':stats,
                    'limitations':'Sampled collision checks; acceleration caps may be engineering fallback; fixed scene required.'}
        except (ValueError,RuntimeError,KeyError,TypeError) as exc:
            result={'valid':False,'error':f'{type(exc).__name__}: {exc}'}
        self.logs.row('validation',{'candidate':self.count,'input':candidate,'result':result})
        return result
    def validate_rows(self,rows):
        for i,row in enumerate(rows):
            for q in ([row['q']] if i==0 else [row['q'],[(a+b)/2+(row['t']-rows[i-1]['t'])*(va-vb)/8 for a,b,va,vb in zip(rows[i-1]['q'],row['q'],rows[i-1]['v'],row['v'])]]):
                result=self.bridge.check_pose(q)
                if not result['valid']:raise ValueError(f'Trajectory {row["label"]} t={row["t"]:.3f}: collision {result["contacts"]}')
            if i%100==0:print(f'検証 {i+1}/{len(rows)}',flush=True)
