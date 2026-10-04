import unittest
from pathlib import Path
import numpy as np
from aperturenav.perception import detect_rectangles

class VisionTests(unittest.TestCase):
    def test_blank_has_no_candidates(self):
        self.assertEqual(detect_rectangles(np.zeros((480,640,3),dtype=np.uint8)),[])
    def test_rectangle_under_noise_blur_and_dim_light(self):
        import cv2
        image=np.full((480,640,3),160,dtype=np.uint8)
        cv2.rectangle(image,(220,160),(420,320),(20,20,20),-1)
        for variant in (image,cv2.GaussianBlur(image,(9,9),2),(image*.45).astype(np.uint8),np.clip(image.astype(float)+np.random.default_rng(42).normal(0,3,image.shape),0,255).astype(np.uint8)):
            candidates=detect_rectangles(variant)
            self.assertTrue(candidates)
            self.assertLess(np.linalg.norm(candidates[0].corners.mean(0)-[320,240]),5)
    def test_multiple_openings(self):
        import cv2
        image=np.full((480,640,3),180,dtype=np.uint8)
        cv2.rectangle(image,(50,120),(200,320),(20,20,20),-1)
        cv2.rectangle(image,(350,150),(530,350),(20,20,20),-1)
        self.assertGreaterEqual(len(detect_rectangles(image)),2)

class ApiTests(unittest.TestCase):
    def test_scenario_validation_commands_and_stream(self):
        from fastapi.testclient import TestClient
        from dashboard.backend.app import app
        with TestClient(app) as client:
            self.assertEqual(client.get('/api/health').status_code,200)
            self.assertEqual(client.post('/api/scenario',json={'width':-1,'height':.6}).status_code,422)
            self.assertEqual(client.post('/api/command',json={'action':'start'}).status_code,200)
            self.assertEqual(client.post('/api/command',json={'action':'start'}).status_code,409)
            self.assertEqual(client.post('/api/scenario',json={'width':.8,'height':.6}).status_code,409)
            with client.websocket_connect('/ws') as ws:
                telemetry=ws.receive_json();self.assertIn('position',telemetry)
            self.assertEqual(client.post('/api/command',json={'action':'estop'}).status_code,200)
            self.assertEqual(client.get('/api/telemetry').json()['state'],'EMERGENCY_HOVER')
            client.post('/api/command',json={'action':'reset'})
            self.assertEqual(client.get('/api/telemetry').json()['state'],'IDLE')
            self.assertEqual(client.get('/api/results/not_a_run/telemetry.csv').status_code,404)
if __name__=='__main__':unittest.main()
