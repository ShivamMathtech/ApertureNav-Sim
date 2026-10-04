"""Deterministic kinematic test simulator; not Gazebo or a flight dynamics model."""
import csv
from datetime import datetime, timezone
import json
from pathlib import Path
import numpy as np
import yaml
from .control import PID
from .geometry import Aperture, clearance, swept_collision, wrap_angle
from .mission import Mission

class Simulation:
    def __init__(self, config):
        self.c = config
        a = config['aperture']
        self.aperture = Aperture(np.array(a['center'],dtype=float), a['width'], a['height'],a['yaw'],a['thickness'])
        self.pid, self.mission = PID(config['controller']), Mission(config,self.aperture)
        self.reset()

    def reset(self):
        self.rng = np.random.default_rng(self.c['seed'])
        self.position = np.array(self.c['drone']['spawn'],dtype=float)
        self.velocity = np.zeros(3)
        self.yaw = 0.
        self.time = 0.
        self.collisions = 0
        self.path_length = 0.
        self.history = []
        self.paused = False
        self.pid.reset()
        self.mission.reset()

    def snapshot(self):
        cl = clearance(self.aperture,self.c['drone'],self.c['safety'],self.position,self.yaw)
        return {'time':self.time,'position':self.position.tolist(),'velocity':self.velocity.tolist(),
                'yaw':self.yaw,'state':self.mission.state,'clearance':cl.minimum,'safe':cl.safe,
                'distance':float(np.linalg.norm(self.position-self.aperture.center)),
                'collisions':self.collisions,'aperture':self.c['aperture'],
                'source':'idealized kinematic simulation', 'seed':self.c['seed']}

    def step(self):
        if self.paused: return self.snapshot()
        dt = self.c['dt']
        target,yaw_target,speed = self.mission.update(self.position,self.yaw,self.time,self.time)
        command = self.pid.step(target-self.position,dt,speed) if speed else np.zeros(3)
        disturbance = self.rng.normal(0,self.c['environment']['wind_std'],3) if speed else np.zeros(3)
        end = self.position + (command+disturbance)*dt
        if swept_collision(self.aperture,self.c['drone'],self.position,end,self.yaw):
            self.collisions += 1
            self.mission.update(self.position,self.yaw,self.time,self.time,collision=True)
            command = np.zeros(3)
            end = self.position.copy()
        self.path_length += float(np.linalg.norm(end-self.position))
        self.position, self.velocity = end, command+disturbance if speed and not self.collisions else command
        if speed:
            self.yaw += np.clip(wrap_angle(yaw_target-self.yaw),-self.c['controller']['max_yaw_rate']*dt,self.c['controller']['max_yaw_rate']*dt)
        self.time += dt
        row = self.snapshot()
        self.history.append(row)
        return row

    def run(self):
        self.mission.start(self.time)
        while self.time < self.c['max_duration'] and self.mission.state not in ('COMPLETE','ABORT','EMERGENCY_HOVER'):
            self.step()
        if self.mission.state not in ('COMPLETE','ABORT','EMERGENCY_HOVER'):
            self.mission.transition('ABORT','experiment duration limit',self.time)
        return self.summary()

    def summary(self):
        near = [r['clearance'] for r in self.history if abs(self.aperture.coordinates(r['position'])[0]) < 0.5]
        return {'mission_success':self.mission.state=='COMPLETE','state':self.mission.state,
                'mission_duration':self.time,'path_length':self.path_length,'collision_count':self.collisions,
                'minimum_clearance_near_frame':min(near) if near else None,
                'maximum_speed':max((float(np.linalg.norm(r['velocity'])) for r in self.history),default=0.),
                'samples':len(self.history),'seed':self.c['seed'],
                'source':'idealized kinematic simulation; no image perception or physical flight dynamics'}

    def save(self, parent):
        stamp = datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S_%f')
        out = Path(parent)/stamp
        out.mkdir(parents=True)
        (out/'config.yaml').write_text(yaml.safe_dump(self.c))
        (out/'summary.json').write_text(json.dumps(self.summary(),indent=2))
        with (out/'telemetry.csv').open('w',newline='') as f:
            writer=csv.writer(f);writer.writerow(['time','x','y','z','vx','vy','vz','yaw','state','clearance','distance','collisions'])
            for r in self.history: writer.writerow([r['time'],*r['position'],*r['velocity'],r['yaw'],r['state'],r['clearance'],r['distance'],r['collisions']])
        with (out/'events.csv').open('w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=['timestamp','from','to','reason']);w.writeheader();w.writerows(self.mission.events)
        return out
