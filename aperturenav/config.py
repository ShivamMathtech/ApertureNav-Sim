"""Validated configuration; distances are metres, angles radians."""
from pathlib import Path
import math
import yaml

def load(path):
    c = yaml.safe_load(Path(path).read_text())
    for group in ('drone', 'aperture', 'safety', 'controller', 'mission', 'environment'):
        if not isinstance(c.get(group), dict):
            raise ValueError(f'Missing configuration group {group}')
    def positive(value, label, zero=False):
        if not isinstance(value, (int, float)) or not math.isfinite(value) or (value < 0 if zero else value <= 0):
            raise ValueError(f'Invalid {label}')
    for k in ('dt', 'max_duration'): positive(c[k], k)
    for k in ('width', 'length', 'height'): positive(c['drone'][k], k)
    for k in ('width', 'height', 'thickness'): positive(c['aperture'][k], k)
    for k in ('margin', 'position_uncertainty', 'yaw_uncertainty'): positive(c['safety'][k], k, True)
    for k in ('stale_sensor_timeout', 'target_lost_timeout'): positive(c['safety'][k], k)
    for k in ('max_speed', 'entry_speed', 'exit_speed', 'max_acceleration', 'max_yaw_rate', 'integral_limit'): positive(c['controller'][k], k)
    for k in ('kp', 'ki', 'kd'): positive(c['controller'][k], k, True)
    for k in ('approach_distance', 'exit_distance', 'position_tolerance', 'yaw_tolerance', 'timeout'): positive(c['mission'][k], k)
    positive(c['environment']['wind_std'], 'wind_std', True)
    for group, key in (('drone','spawn'), ('aperture','center')):
        if len(c[group][key]) != 3 or not all(math.isfinite(v) for v in c[group][key]):
            raise ValueError(f'Invalid {group}.{key}')
    if not math.isfinite(c['aperture']['yaw']): raise ValueError('Invalid yaw')
    return c
