"""Mission sequencing shared by the idealized simulator and ROS node."""
import numpy as np
from .geometry import clearance, waypoints, wrap_angle

STATES = ('IDLE','TAKEOFF','SEARCH','TARGET_DETECTED','APPROACH','ALIGN_POSITION',
          'ALIGN_YAW','VERIFY_CLEARANCE','READY_TO_ENTER','ENTER','PASS_THROUGH',
          'CLEAR_APERTURE','HOVER','COMPLETE','ABORT','EMERGENCY_HOVER')
NORMAL = dict(zip(STATES[:13], STATES[1:14]))
NORMAL['HOVER'] = 'COMPLETE'

class Mission:
    def __init__(self, config, aperture):
        self.c, self.aperture = config, aperture
        self.reset()

    def reset(self):
        self.state = 'IDLE'
        self.events = []
        self.started = None

    def transition(self, new, reason, timestamp):
        if new not in STATES or new == self.state:
            raise ValueError(f'Invalid transition {self.state} -> {new}')
        allowed = new == NORMAL.get(self.state) or (new in ('ABORT','EMERGENCY_HOVER') and self.state not in ('COMPLETE','ABORT'))
        if self.state == 'EMERGENCY_HOVER' and new == 'ABORT': allowed = True
        if not allowed: raise ValueError(f'Invalid transition {self.state} -> {new}')
        self.events.append({'timestamp': timestamp, 'from': self.state, 'to': new, 'reason': reason})
        self.state = new

    def start(self, timestamp):
        if self.state != 'IDLE': raise ValueError('Reset before starting another mission')
        self.started = timestamp
        self.transition('TAKEOFF', 'mission requested', timestamp)

    def update(self, position, yaw, now, sensor_stamp, detected=True, collision=False):
        a, c = self.aperture, self.c
        hold = np.asarray(position).copy()
        if self.state in ('IDLE','COMPLETE','ABORT','EMERGENCY_HOVER'):
            return hold, yaw, 0.
        if collision or now-sensor_stamp > c['safety']['stale_sensor_timeout']:
            self.transition('EMERGENCY_HOVER', 'collision' if collision else 'stale sensors', now)
            return hold, yaw, 0.
        if now-self.started > c['mission']['timeout']:
            self.transition('ABORT', 'mission timeout', now)
            return hold, yaw, 0.
        if not detected and self.state not in ('TAKEOFF','SEARCH'):
            self.transition('EMERGENCY_HOVER', 'target lost', now)
            return hold, yaw, 0.
        pre, center, post = waypoints(a,c['mission']['approach_distance'],c['mission']['exit_distance'])
        target = pre
        speed = c['controller']['max_speed']
        if self.state == 'TAKEOFF':
            target = np.array([position[0], position[1], c['drone']['spawn'][2]])
            if abs(position[2]-target[2]) < c['mission']['position_tolerance']:
                self.transition('SEARCH','takeoff altitude reached',now)
        elif self.state == 'SEARCH':
            target = hold
            if detected: self.transition('TARGET_DETECTED','aperture detected',now)
        elif self.state == 'TARGET_DETECTED':
            if a.coordinates(position)[0] >= -c['mission']['approach_distance']:
                self.transition('ABORT','start must be on approach side',now)
                return hold,yaw,0.
            self.transition('APPROACH','approach waypoint selected',now)
        elif self.state == 'APPROACH':
            if np.linalg.norm(position-pre) < c['mission']['position_tolerance']:
                self.transition('ALIGN_POSITION','approach point reached',now)
        elif self.state == 'ALIGN_POSITION':
            if np.linalg.norm(position-pre) < c['mission']['position_tolerance']:
                self.transition('ALIGN_YAW','centerline aligned',now)
        elif self.state == 'ALIGN_YAW':
            if abs(wrap_angle(yaw-a.yaw)) < c['mission']['yaw_tolerance']:
                self.transition('VERIFY_CLEARANCE','heading aligned',now)
        elif self.state == 'VERIFY_CLEARANCE':
            if clearance(a,c['drone'],c['safety'],position,yaw).safe:
                self.transition('READY_TO_ENTER','clearance valid',now)
            else:
                self.transition('ABORT','opening too small or uncertain',now)
                return hold,yaw,0.
        elif self.state == 'READY_TO_ENTER':
            self.transition('ENTER','entry authorized',now)
        elif self.state in ('ENTER','PASS_THROUGH'):
            target, speed = post,c['controller']['entry_speed']
            if not clearance(a,c['drone'],c['safety'],position,yaw).safe:
                self.transition('EMERGENCY_HOVER','clearance lost',now)
                return hold,yaw,0.
            along = a.coordinates(position)[0]
            if self.state == 'ENTER' and along >= 0:
                self.transition('PASS_THROUGH','center crossed',now)
            elif self.state == 'PASS_THROUGH' and along > (a.thickness+c['drone']['length'])/2+c['safety']['margin']:
                self.transition('CLEAR_APERTURE','vehicle clear of frame',now)
        elif self.state == 'CLEAR_APERTURE':
            target,speed = post,c['controller']['exit_speed']
            if np.linalg.norm(position-post) < c['mission']['position_tolerance']:
                self.transition('HOVER','exit point reached',now)
        elif self.state == 'HOVER':
            target = post
            self.transition('COMPLETE','traversal complete',now)
        return target,a.yaw,speed
