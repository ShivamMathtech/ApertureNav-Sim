import copy
from pathlib import Path
import unittest
import numpy as np
from aperturenav.config import load
from aperturenav.geometry import Aperture,clearance,swept_collision,minimum_jerk,projected_width
from aperturenav.simulation import Simulation
from aperturenav.control import PID
from aperturenav.perception import localize_frame

ROOT=Path(__file__).resolve().parents[1]
class Tests(unittest.TestCase):
    def setUp(self): self.c=load(ROOT/'config/default.yaml')
    def test_nominal_traversal_and_reset(self):
        s=Simulation(self.c);r=s.run()
        self.assertTrue(r['mission_success']);self.assertEqual(r['collision_count'],0)
        states=[e['to'] for e in s.mission.events]
        self.assertIn('VERIFY_CLEARANCE',states);self.assertIn('PASS_THROUGH',states)
        first=s.position.copy();s.reset();self.assertEqual(s.mission.state,'IDLE')
        self.assertEqual(s.history,[]);self.assertTrue(s.run()['mission_success'])
        np.testing.assert_allclose(first,s.position)
    def test_too_small_aborts_before_frame(self):
        self.c['aperture']['width']=.45
        s=Simulation(self.c);r=s.run();self.assertEqual(r['state'],'ABORT')
        self.assertLess(s.position[0],5);self.assertEqual(r['collision_count'],0)
    def test_threshold_rejected(self):
        s=Simulation(self.c);d=self.c['drone'];safe=self.c['safety']
        a=Aperture(np.zeros(3),projected_width(d['width'],d['length'],0,safe['yaw_uncertainty'])+2*(safe['margin']+safe['position_uncertainty']),.6)
        self.assertFalse(clearance(a,d,safe,np.zeros(3),0).safe)
    def test_swept_collision_prevents_tunnelling(self):
        s=Simulation(self.c);a=s.aperture
        self.assertTrue(swept_collision(a,self.c['drone'],[0,2,1.5],[10,2,1.5],0))
        self.assertFalse(swept_collision(a,self.c['drone'],[0,0,1.5],[10,0,1.5],0))
    def test_rotated_aperture(self):
        self.c['aperture']['yaw']=.3
        s=Simulation(self.c);self.assertTrue(s.run()['mission_success'])
    def test_sensor_timeout_and_invalid_transition(self):
        s=Simulation(self.c);s.mission.start(0)
        s.mission.update(s.position,0,2,0)
        self.assertEqual(s.mission.state,'EMERGENCY_HOVER')
        with self.assertRaises(ValueError):s.mission.transition('COMPLETE','invalid',3)
    def test_pid_limits(self):
        p=PID(self.c['controller']);v=p.step([10,10,10],.02)
        self.assertLessEqual(np.linalg.norm(v),.012+1e-10)
        for _ in range(1000):v=p.step([10,10,10],.02)
        self.assertLessEqual(np.linalg.norm(v),.4+1e-10)
        self.assertLessEqual(np.linalg.norm(p.step([1,1,1],.02,.15)),.15+1e-10)
    def test_minimum_jerk_endpoints(self):
        p=minimum_jerk([0,0,0],[1,2,3],2,.02)
        np.testing.assert_allclose(p[0],[0,0,0]);np.testing.assert_allclose(p[-1],[1,2,3])
    def test_localization_valid_and_invalid_depth(self):
        corners=np.array([[240,180],[400,180],[400,300],[240,300]])
        depth=np.full((480,640),2.0)
        r=localize_frame(corners,depth,[400,400,320,240])
        np.testing.assert_allclose(r['center'],[0,0,2],atol=1e-8)
        self.assertAlmostEqual(r['width'],.8);self.assertAlmostEqual(r['height'],.6)
        with self.assertRaises(ValueError):localize_frame(corners,depth*0,[400,400,320,240])
if __name__=='__main__':unittest.main()
