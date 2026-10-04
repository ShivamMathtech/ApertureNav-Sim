"""World-frame aperture geometry and conservative rectangular clearance."""
from dataclasses import dataclass
import math
import numpy as np

@dataclass(frozen=True)
class Aperture:
    center: np.ndarray
    width: float
    height: float
    yaw: float = 0.0
    thickness: float = 0.15

    @property
    def normal(self):
        return np.array([math.cos(self.yaw), math.sin(self.yaw), 0.0])

    @property
    def lateral(self):
        return np.array([-math.sin(self.yaw), math.cos(self.yaw), 0.0])

    def coordinates(self, position):
        d = np.asarray(position) - self.center
        return np.array([d @ self.normal, d @ self.lateral, d[2]])

@dataclass(frozen=True)
class Clearance:
    horizontal: float
    vertical: float
    minimum: float
    safe: bool

def wrap_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

def projected_width(width, length, relative_yaw, yaw_uncertainty=0.0):
    # Maximize footprint over the entire yaw uncertainty interval.
    lo, hi = relative_yaw - yaw_uncertainty, relative_yaw + yaw_uncertainty
    candidates = [lo, hi]
    alpha = math.atan2(length, width)
    for k in range(math.floor(lo / math.pi) - 1, math.ceil(hi / math.pi) + 2):
        for a in (alpha, math.pi - alpha):
            v = a + k * math.pi
            if lo <= v <= hi:
                candidates.append(v)
    return max(width * abs(math.cos(a)) + length * abs(math.sin(a)) for a in candidates)

def clearance(aperture, drone, safety, position, yaw):
    _, lateral, vertical = aperture.coordinates(position)
    span = projected_width(drone['width'], drone['length'],
                           wrap_angle(yaw - aperture.yaw), safety['yaw_uncertainty'])
    reserve = safety['margin'] + safety['position_uncertainty']
    h = (aperture.width - span) / 2 - abs(lateral) - reserve
    v = (aperture.height - drone['height']) / 2 - abs(vertical) - reserve
    return Clearance(h, v, min(h, v), bool(h > 1e-9 and v > 1e-9))

def swept_collision(aperture, drone, start, end, yaw):
    """Check linear swept AABB against a wall slab containing one opening."""
    a, b = aperture.coordinates(start), aperture.coordinates(end)
    span_y = projected_width(drone['width'], drone['length'], yaw - aperture.yaw)
    span_x = projected_width(drone['length'], drone['width'], yaw - aperture.yaw)
    slab = (aperture.thickness + span_x) / 2
    dx = b[0] - a[0]
    if abs(dx) < 1e-12:
        if abs(a[0]) > slab:
            return False
        times = [0.0, 1.0]
    else:
        t0, t1 = sorted(((-slab - a[0]) / dx, (slab - a[0]) / dx))
        lo, hi = max(0., t0), min(1., t1)
        if lo > hi:
            return False
        times = [lo, hi]
    for t in times:
        p = a + t * (b - a)
        if abs(p[1]) + span_y / 2 >= aperture.width / 2:
            return True
        if abs(p[2]) + drone['height'] / 2 >= aperture.height / 2:
            return True
    return False

def waypoints(aperture, approach_distance, exit_distance):
    return np.array([aperture.center - approach_distance * aperture.normal,
                     aperture.center,
                     aperture.center + exit_distance * aperture.normal])

def minimum_jerk(start, end, duration, dt):
    if duration <= 0 or dt <= 0:
        raise ValueError('duration and dt must be positive')
    t = np.linspace(0., 1., max(2, math.ceil(duration / dt) + 1))
    blend = 10*t**3 - 15*t**4 + 6*t**5
    return np.asarray(start) + blend[:, None] * (np.asarray(end) - start)
