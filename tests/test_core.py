import csv
import json
import math
from pathlib import Path
import tempfile
import unittest
from robot.model import RobotModel
from robot.trajectory import build_motion
from recording.logger import Logs
from ai.schema import defaults
from ai.planner import tools

ROOT=Path(__file__).resolve().parents[1]


class CoreTests(unittest.TestCase):
    def setUp(self):
        fixture=ROOT/'tests/fixtures'
        self.config=json.loads((ROOT/'config/settings.json').read_text())
        self.targets=json.loads((ROOT/'scenarios/targets.json').read_text())
        self.model=RobotModel((fixture/'robot.urdf').read_text(),(fixture/'robot.srdf').read_text(),{},self.config)
    def test_live_geometry(self):
        p=self.model.pose([0,0,0,0]);self.assertLess(math.dist(p['xyz'],[0,0,.2479]),1e-10)
        self.assertLess(math.dist(p['quaternion'],[0,0,0,1]),1e-10)
    def test_ik_roundtrip(self):
        for p in self.targets['points']:
            q=self.model.ik(p['xyz'],.4)
            self.assertLess(math.dist(self.model.pose(q)['xyz'],p['xyz']),1e-10)
    def test_joint4_does_not_move_tip(self):
        a=self.model.pose([.2,.3,-.5,.1]);b=self.model.pose([.2,.3,-.5,.6])
        self.assertLess(math.dist(a['xyz'],b['xyz']),1e-10)
        self.assertGreater(math.dist(a['quaternion'],b['quaternion']),.1)
    def test_missing_joint_is_not_zero(self):
        poses=self.model.poses(dict(zip(self.model.names,[0,0,0,0])))
        self.assertNotIn('crane_plus_link_hand',poses)
    def test_none_csv(self):
        with tempfile.TemporaryDirectory() as d:
            logs=Logs(Path(d));logs.row('sample',{'missing':None,'zero':0});logs.close()
            with open(Path(d)/'sample.csv') as f:row=next(csv.DictReader(f))
            self.assertEqual(row['missing'],'None');self.assertEqual(row['zero'],'0')
    def test_trajectory(self):
        m=defaults(self.targets);m['points']['B']['hold_s']=1
        m['segments']['CD'].update(joint4_amplitude_deg=15,oscillation_cycles=2)
        m['segments']['AB']['midpoint_pitch_deg']=30;m['segments']['BC']['midpoint_pitch_deg']=30
        rows,stats=build_motion(self.model,self.targets,m)
        self.assertEqual(stats['duration_s'],25)
        self.assertTrue(all(b['t']>a['t'] for a,b in zip(rows,rows[1:])))
        self.assertEqual(rows[0]['v'],[0.]*4);self.assertEqual(rows[-1]['v'],[0.]*4)
    def test_fast_motion_rejected(self):
        m=defaults(self.targets);m['segments']['CD'].update(duration_s=1,joint4_amplitude_deg=15,oscillation_cycles=2)
        with self.assertRaises(ValueError):build_motion(self.model,self.targets,m)
    def test_tool_boundaries(self):
        names={t['name'] for t in tools()}
        self.assertIn('validate_plan',names);self.assertNotIn('execute',names)
        for t in tools():self.assertTrue(t['strict'])


if __name__=='__main__':unittest.main()
