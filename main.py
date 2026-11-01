#!/usr/bin/env python3
"""One entry point. Reads instruction.txt; generates, validates, confirms, executes."""
import argparse
import json
import math
import os
from pathlib import Path
import shutil
from datetime import datetime,timezone
import uuid

ROOT=Path(__file__).resolve().parent




def confirm_execution():
    """Return True/False, or None when no interactive input is available."""
    prompt='\nこの計画を実行しますか？ [y / Enterで終了]: '
    try:
        answer=input(prompt)
    except EOFError:
        # Some launchers close stdin while leaving a controlling terminal.
        try:
            with open('/dev/tty','r+',encoding='utf-8') as terminal:
                terminal.write(prompt)
                terminal.flush()
                answer=terminal.readline()
                if answer=='':
                    return None
        except OSError:
            return None
    return answer.strip().lower()=='y'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan-only',action='store_true',help='Generate and validate without asking to execute')
    parser.add_argument('--inspect-only',action='store_true',help='Collect ROS model/state/topic information without API calls or motion')
    args=parser.parse_args()
    config=json.loads((ROOT/'config/settings.json').read_text())
    instruction=(ROOT/'scenarios/instruction.txt').read_text(encoding='utf-8-sig').strip()
    targets=json.loads((ROOT/'scenarios/targets.json').read_text(encoding='utf-8-sig'))
    if not instruction:parser.error('instruction.txt is empty')
    if [p['name'] for p in targets['points']]!=list('ABCDE') or targets['units']!='m':parser.error('Expected A-E targets in metres')
    if not args.inspect_only and not os.getenv('OPENAI_API_KEY'):parser.error('OPENAI_API_KEY is not set in this terminal')
    import rclpy
    from recording.logger import Logs
    from robot.bridge import Bridge
    from robot.model import RobotModel
    from robot.validation import Validator
    from ai.planner import plan
    run_id=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+uuid.uuid4().hex[:6]
    folder=ROOT/'runs'/run_id;folder.mkdir(parents=True)
    import sys,platform,importlib.metadata
    versions={}
    for package in ('openai','numpy'):
        try:versions[package]=importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError:versions[package]=None
    (folder/'environment.json').write_text(json.dumps({'python':sys.version,'platform':platform.platform(),
        'ROS_DISTRO':os.getenv('ROS_DISTRO'),'packages':versions},indent=2))
    for source in [ROOT/'scenarios/instruction.txt',ROOT/'scenarios/targets.json',ROOT/'config/settings.json']:
        shutil.copyfile(source,folder/source.name)
    logs=Logs(folder);bridge=None;result={'run_id':run_id,'status':'starting','executed':False}
    rclpy.init()
    try:
        bridge=Bridge(logs,config)
        parameters=bridge.parameters()
        (folder/'robot_description.urdf').write_text(parameters['robot_description'])
        (folder/'robot_description.srdf').write_text(parameters['robot_description_semantic'])
        (folder/'parameters.json').write_text(json.dumps(parameters,indent=2))
        bridge.model=RobotModel(parameters['robot_description'],parameters['robot_description_semantic'],parameters,config)
        if targets['frame_id']!=bridge.model.base:raise ValueError('Target frame does not match SRDF base')
        context=bridge.model.context();state=bridge.snapshot()
        (folder/'robot_context.json').write_text(json.dumps(context,indent=2))
        (folder/'initial_state.json').write_text(json.dumps(state,indent=2))
        topics=bridge.discover()
        if config['record_rosbag']:
            if not topics:raise RuntimeError('No topics available for rosbag2')
            logs.start_bag(topics)
        bridge.scene();bridge.check_model([state['joint_positions'][j] for j in bridge.model.names])
        logs.event('context_ready',{'base':bridge.model.base,'tip':bridge.model.tip})
        print('実行フォルダ:',folder,flush=True)
        print('取得した関節:',', '.join(bridge.model.names),flush=True)
        if args.inspect_only:
            result['status']='inspection_completed';return 0
        logs.phase='ai_planning';validator=Validator(bridge,targets,logs)
        accepted,explanation=plan(bridge,validator,targets,instruction,config,logs)
        print('\nAIの計画: '+explanation,flush=True)
        result['plan_id']=accepted['plan_id'];result['status']='validated'
        if args.plan_only:
            print('計画のみ完了しました。アームは動かしていません。',flush=True)
            return 0
        confirmation=confirm_execution()
        if confirmation is None:
            result['status']='cancelled'
            result['cancellation_reason']='interactive_input_unavailable'
            print('対話入力を取得できないため、実行せず終了しました。',flush=True)
            logs.event('execution_confirmation_unavailable')
            return 0
        if not confirmation:
            result['status']='cancelled'
            result['cancellation_reason']='operator_declined'
            print('実行せず終了しました。',flush=True)
            return 0
        # Final independent gate: never rely on the AI's success statement.
        logs.phase='final_validation'
        current=bridge.snapshot()['joint_positions']
        if any(abs(current[j]-accepted['start'][j])>.03 for j in bridge.model.names):raise RuntimeError('Start changed during planning; run again')
        validator.validate_rows(accepted['rows'])
        result['executed']=True
        bridge.execute(accepted['approach'],'approach')
        at_a=bridge.model.pose([bridge.snapshot()['joint_positions'][j] for j in bridge.model.names])['xyz']
        if math.dist(at_a,targets['points'][0]['xyz'])>targets['endpoint_tolerance_m']:raise RuntimeError('A endpoint position error')
        bridge.execute(accepted['drawing'],'drawing')
        final=bridge.snapshot();q=[final['joint_positions'][j] for j in bridge.model.names]
        error=math.dist(bridge.model.pose(q)['xyz'],targets['points'][-1]['xyz'])
        result['final_position_error_m']=error
        if error>targets['endpoint_tolerance_m']:raise RuntimeError('Final position error exceeds tolerance')
        result['status']='completed';print(f'MOTION COMPLETED: final error={error*1000:.3f} mm',flush=True)
        return 0
    except BaseException as exc:
        result['status']='failed';result['error']=f'{type(exc).__name__}: {exc}'
        print('STOPPED:',result['error'],flush=True);logs.event('failed',result)
        return 130 if isinstance(exc,KeyboardInterrupt) else 1
    finally:
        logs.event('run_finished',result)
        if bridge:bridge.close()
        logs.close();rclpy.shutdown()
        (folder/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
        print('保存先:',folder,flush=True)


if __name__=='__main__':raise SystemExit(main())
