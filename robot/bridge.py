"""ROS adapter: live state, parameters, services and execution; background spinning."""
import copy
import json
import math
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import GetParameters,ListParameters
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetStateValidity,GetPositionFK,GetPlanningScene
from moveit_msgs.action import MoveGroup,ExecuteTrajectory
from moveit_msgs.msg import RobotState,Constraints,JointConstraint,RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message
from action_msgs.msg import GoalStatus


class Bridge(Node):
    def __init__(self,logs,config):
        super().__init__('ai_motion_demo',parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.logs=logs;self.config=config;self.model=None;self.positions={};self.last_state=0.;self.lock=threading.Lock();self.last_links=0.;self.active=None;self.controller_error=None;self.controller_error_at=0.
        self.subscription=self.create_subscription(JointState,'/joint_states',self.on_state,qos_profile_sensor_data)
        self.planner=ActionClient(self,MoveGroup,'/move_action');self.execute_client=ActionClient(self,ExecuteTrajectory,'/execute_trajectory')
        self.validity=self.create_client(GetStateValidity,'/check_state_validity');self.fk_client=self.create_client(GetPositionFK,'/compute_fk')
        self.executor_thread=SingleThreadedExecutor();self.executor_thread.add_node(self)
        self.worker=threading.Thread(target=self.executor_thread.spin,daemon=True);self.worker.start()
        logs.clock=lambda:self.get_clock().now().nanoseconds/1e9
        self.extra_subs=[]
    def on_state(self,msg):
        with self.lock:self.positions=dict(zip(msg.name,msg.position));self.last_state=time.monotonic()
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        for i,n in enumerate(msg.name):
            self.logs.row('joints',{'topic':'/joint_states','message_stamp_s':stamp,'joint':n,
                'position_rad':msg.position[i] if i<len(msg.position) else None,
                'velocity_rad_s':msg.velocity[i] if i<len(msg.velocity) else None,
                'effort':msg.effort[i] if i<len(msg.effort) else None})
        if self.model and time.monotonic()-self.last_links>=.1:
            self.last_links=time.monotonic()
            for name,p in self.model.poses(self.positions).items():
                self.logs.row('links',{'message_stamp_s':stamp,'frame':self.model.base,'link':name,
                    'source':'URDF_FK_from_joint_states','x_m':p['xyz'][0],'y_m':p['xyz'][1],'z_m':p['xyz'][2],
                    'quaternion_xyzw':p['quaternion']})
    def wait(self,future,timeout=15):
        ready=threading.Event();future.add_done_callback(lambda _:ready.set())
        if not ready.wait(timeout):raise RuntimeError('ROS response timeout')
        return future.result()
    def call(self,client,request):
        if not client.wait_for_service(timeout_sec=10):raise RuntimeError('ROS service unavailable: '+client.srv_name)
        return self.wait(client.call_async(request))
    def snapshot(self):
        deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            with self.lock:p=copy.deepcopy(self.positions);age=time.monotonic()-self.last_state
            if self.model and all(j in p for j in self.model.names) and age<1:
                return {'joint_positions':p,'age_s':age,'sim_s':self.get_clock().now().nanoseconds/1e9,
                        'link_poses':self.model.poses(p)}
            time.sleep(.02)
        raise RuntimeError('Fresh complete joint state unavailable; check Gazebo is running')
    def parameters(self):
        remote=self.config['parameter_node'].rstrip('/')
        get=self.create_client(GetParameters,remote+'/get_parameters')
        listing=self.create_client(ListParameters,remote+'/list_parameters')
        req=ListParameters.Request();req.depth=0
        names=self.call(listing,req).result.names
        names=list(dict.fromkeys(['robot_description','robot_description_semantic']+[n for n in names if 'joint_limits' in n]))
        request=GetParameters.Request();request.names=names
        values=self.call(get,request).values
        result={}
        attrs={1:'bool_value',2:'integer_value',3:'double_value',4:'string_value',5:'byte_array_value',6:'bool_array_value',7:'integer_array_value',8:'double_array_value',9:'string_array_value'}
        for name,value in zip(names,values):result[name]=getattr(value,attrs[value.type]) if value.type in attrs else None
        if not result.get('robot_description') or not result.get('robot_description_semantic'):raise RuntimeError('URDF/SRDF unavailable on '+remote)
        return result
    def discover(self):
        topics=dict(self.get_topic_names_and_types());self.logs.event('topic_inventory',topics)
        (self.logs.folder/'topics.json').write_text(json.dumps(topics,indent=2))
        selected=[n for n in topics if n in self.config['bag_topics'] or any(t.endswith('/JointTrajectoryControllerState') for t in topics[n])]
        required=['/joint_states','/clock','/tf','/tf_static']
        selected=sorted(set(selected+[t for t in required if t in topics]))
        for name,types in topics.items():
            if any(t.endswith('/JointTrajectoryControllerState') for t in types):
                def callback(msg,topic=name):
                    self.logs.row('controller_messages',{'topic':topic,'message':message_to_ordereddict(msg)})
                    names=list(msg.joint_names)
                    desired=getattr(msg,'desired',getattr(msg,'reference',None))
                    actual=getattr(msg,'actual',getattr(msg,'feedback',None))
                    error=getattr(msg,'error',None)
                    def item(part,field,i):
                        values=getattr(part,field,[]) if part is not None else []
                        return values[i] if i<len(values) else None
                    errors=[]
                    for i,j in enumerate(names):
                        data={'topic':topic,'joint':j}
                        for prefix,part in [('desired',desired),('actual',actual),('error',error)]:
                            for field in ('positions','velocities','accelerations'):
                                data[prefix+'_'+field]=item(part,field,i)
                        self.logs.row('controller_state',data)
                        if j in self.model.names and data['error_positions'] is not None:errors.append(abs(data['error_positions']))
                    if errors:self.controller_error=max(errors);self.controller_error_at=time.monotonic()
                self.extra_subs.append(self.create_subscription(get_message(types[0]),name,callback,qos_profile_sensor_data))
            elif any(t in ('gazebo_msgs/msg/LinkStates','gazebo_msgs/msg/ModelStates') for t in types):
                selected.append(name)
                def sim_callback(msg,topic=name):
                    self.logs.row('simulator_state',{'topic':topic,'source':'Gazebo','message':message_to_ordereddict(msg)})
                self.extra_subs.append(self.create_subscription(get_message(types[0]),name,sim_callback,qos_profile_sensor_data))
        return selected
    def state(self,q):
        vals=self.snapshot()['joint_positions'];vals.update(zip(self.model.names,q))
        rs=RobotState();rs.is_diff=True;rs.joint_state.name=list(vals);rs.joint_state.position=[float(vals[j]) for j in vals]
        return rs
    def check_pose(self,q):
        self.model.check(q)
        req=GetStateValidity.Request();req.group_name=self.config['group'];req.robot_state=self.state(q)
        res=self.call(self.validity,req)
        return {'valid':res.valid,'contacts':[(c.contact_body_1,c.contact_body_2) for c in res.contacts],
                'pose':self.model.pose(q)}
    def check_model(self,q):
        req=GetPositionFK.Request();req.header.frame_id=self.model.base;req.fk_link_names=[self.model.tip];req.robot_state=self.state(q)
        res=self.call(self.fk_client,req)
        if res.error_code.val!=1:raise RuntimeError('MoveIt FK failed')
        p=res.pose_stamped[0].pose.position
        if math.dist([p.x,p.y,p.z],self.model.pose(q)['xyz'])>.0001:raise RuntimeError('Local model and MoveIt FK disagree')
    def scene(self):
        client=self.create_client(GetPlanningScene,'/get_planning_scene')
        req=GetPlanningScene.Request();req.components.components=1023
        scene=self.call(client,req).scene
        data=message_to_ordereddict(scene)
        (self.logs.folder/'planning_scene.json').write_text(json.dumps(data,indent=2))
        return data
    def action(self,client,goal,timeout=90):
        if not client.wait_for_server(timeout_sec=10):raise RuntimeError('Action server unavailable')
        handle=self.wait(client.send_goal_async(goal))
        if handle is None or not handle.accepted:raise RuntimeError('Action rejected')
        self.active=handle
        try:
            future=handle.get_result_async()
            deadline=time.monotonic()+timeout;bad_since=None
            while not future.done():
                if time.monotonic()>deadline:raise RuntimeError('Action timed out')
                if client is self.execute_client:
                    if time.monotonic()-self.last_state>2:raise RuntimeError('Joint-state stream stale during execution')
                    fresh=time.monotonic()-self.controller_error_at<1
                    if fresh and self.controller_error is not None and self.controller_error>.3:
                        bad_since=bad_since or time.monotonic()
                        if time.monotonic()-bad_since>.5:raise RuntimeError('Controller tracking error > .3 rad for .5 seconds')
                    else:bad_since=None
                time.sleep(.05)
            result=future.result()
            if result.status!=GoalStatus.STATUS_SUCCEEDED or result.result.error_code.val!=1:
                raise RuntimeError('MoveIt action failed: '+str(result.result.error_code.val))
            return result.result
        except BaseException:
            try:self.wait(handle.cancel_goal_async(),5)
            except BaseException:pass
            raise
        finally:self.active=None
    def approach(self,q):
        goal=MoveGroup.Goal();req=goal.request;req.group_name=self.config['group']
        current=self.snapshot()['joint_positions'];req.start_state=self.state([current[n] for n in self.model.names])
        req.num_planning_attempts=5;req.allowed_planning_time=5.;req.max_velocity_scaling_factor=.2;req.max_acceleration_scaling_factor=.2
        cons=Constraints()
        for j,v in zip(self.model.names,q):cons.joint_constraints.append(JointConstraint(joint_name=j,position=float(v),tolerance_above=.0005,tolerance_below=.0005,weight=1.))
        req.goal_constraints=[cons];goal.planning_options.plan_only=True
        goal.planning_options.planning_scene_diff.is_diff=True;goal.planning_options.planning_scene_diff.robot_state.is_diff=True
        traj=self.action(self.planner,goal).planned_trajectory
        if not traj.joint_trajectory.points:raise RuntimeError('Empty approach')
        end=dict(zip(traj.joint_trajectory.joint_names,traj.joint_trajectory.points[-1].positions))
        if any(abs(end[j]-v)>.001 for j,v in zip(self.model.names,q)):raise RuntimeError('Approach endpoint inaccurate')
        return traj
    def make_trajectory(self,rows):
        traj=RobotTrajectory();traj.joint_trajectory.joint_names=self.model.names
        for row in rows:
            ns=round(row['t']*1e9)
            traj.joint_trajectory.points.append(JointTrajectoryPoint(positions=[float(v) for v in row['q']],velocities=[float(v) for v in row['v']],time_from_start=Duration(sec=ns//10**9,nanosec=ns%10**9)))
        return traj
    def execute(self,traj,phase):
        jt=traj.joint_trajectory;current=self.snapshot()['joint_positions']
        if any(abs(current[n]-jt.points[0].positions[i])>.03 for i,n in enumerate(jt.joint_names)):raise RuntimeError('Start changed; replan')
        self.logs.phase=phase
        for p in jt.points:self.logs.row('commands',{'trajectory_time_s':p.time_from_start.sec+p.time_from_start.nanosec/1e9,'joint_names':jt.joint_names,'positions_rad':list(p.positions),'velocities_rad_s':list(p.velocities) if p.velocities else None})
        self.logs.event('execute_start',{'phase':phase})
        goal=ExecuteTrajectory.Goal();goal.trajectory=traj
        duration=jt.points[-1].time_from_start.sec+jt.points[-1].time_from_start.nanosec/1e9
        self.action(self.execute_client,goal,max(90,duration*10+30))
        deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            p=self.snapshot()['joint_positions']
            error=max(abs(p[n]-jt.points[-1].positions[i]) for i,n in enumerate(jt.joint_names))
            if error<.03:self.logs.event('execute_success',{'max_joint_error_rad':error});return
            time.sleep(.05)
        raise RuntimeError('Final joint error exceeds .03 rad')
    def close(self):
        self.executor_thread.shutdown(timeout_sec=5);self.worker.join(timeout=5);self.destroy_node()
