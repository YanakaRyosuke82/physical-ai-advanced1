"""Mock API protocol only; never calls the actual API."""
import json
import sys
from types import SimpleNamespace
import tempfile
from pathlib import Path
from unittest.mock import patch
import unittest
from ai.planner import plan


class LoopTests(unittest.TestCase):
    def test_context_validate_finish_protocol(self):
        calls=[('get_robot_context',{}),('get_current_state',{}),('validate_plan',{}),('finish_plan',{'plan_id':'p1','explanation':'検証成功'})]
        class API:
            def __init__(self,**kwargs):self.responses=self;self.i=0
            def __enter__(self):return self
            def __exit__(self,*args):pass
            def create(self,**kwargs):
                name,args=calls[self.i];self.i+=1
                return SimpleNamespace(status='completed',output=[SimpleNamespace(type='function_call',name=name,arguments=json.dumps(args),call_id=str(self.i))])
        class Validator:
            accepted=None
            def validate(self,candidate):self.accepted={'plan_id':'p1'};return {'valid':True,'plan_id':'p1'}
        class Logs:
            def row(self,*args):pass
            def event(self,*args):pass
        with tempfile.TemporaryDirectory() as d:
            logs=Logs();logs.folder=Path(d)
            bridge=SimpleNamespace(model=SimpleNamespace(context=lambda:{}),snapshot=lambda:{'joint_positions':{}})
            with patch.dict(sys.modules,{'openai':SimpleNamespace(OpenAI=API)}):
                result,_=plan(bridge,Validator(),{},'test',{'max_api_turns':5,'max_candidates':3,'model':'test'},logs)
            self.assertEqual(result['plan_id'],'p1')
    def test_unvalidated_finish_rejected(self):
        class API:
            def __init__(self,**kwargs):self.responses=self
            def __enter__(self):return self
            def __exit__(self,*args):pass
            def create(self,**kwargs):return SimpleNamespace(status='completed',output=[SimpleNamespace(type='function_call',name='finish_plan',arguments='{"plan_id":"invented","explanation":"done"}',call_id='1')])
        logs=SimpleNamespace(row=lambda *a:None,event=lambda *a:None)
        bridge=SimpleNamespace(model=SimpleNamespace())
        with patch.dict(sys.modules,{'openai':SimpleNamespace(OpenAI=API)}):
            with self.assertRaises(RuntimeError):
                plan(bridge,SimpleNamespace(accepted=None),{},'test',{'max_api_turns':1,'max_candidates':3,'model':'test'},logs)
