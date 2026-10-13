"""Simplest possible radar model for the lightweight backend.

Geometric point-target radar: no occlusion, no noise, no clutter. Each target
inside the sensor range and field of view yields exactly one detection per
measurement, in the CARLA `RadarDetection` layout (velocity, azimuth, altitude,
depth). `points_per_second` is ignored.

Conventions follow carla `sensor.other.radar`: sensor frame x forward, y right,
z up; azimuth positive toward +y; altitude positive up; velocity is the
relative radial speed, positive when the target and sensor move apart.
"""

import math
from dataclasses import dataclass

from geometry import world_to_local


@dataclass
class Target:
    north: float
    east: float
    z: float = 0.0
    v_north: float = 0.0
    v_east: float = 0.0

    def step(self, dt):
        """Constant-velocity motion."""
        self.north += self.v_north * dt
        self.east += self.v_east * dt

    @staticmethod
    def parse(text):
        """'north,east[,z[,v_north,v_east]]' in metres and metres/second."""
        values = [float(v) for v in text.split(",")]
        if len(values) not in (2, 3, 5):
            raise ValueError("target must be north,east[,z[,v_north,v_east]]: " + text)
        return Target(*values)


def detect(targets, sensor_loc, sensor_axes, sensor_vel, h_fov_deg, v_fov_deg, range_m):
    """Returns a list of (velocity, azimuth, altitude, depth) tuples."""
    detections = []
    for t in targets:
        rel = (t.north - sensor_loc[0], t.east - sensor_loc[1], t.z - sensor_loc[2])
        depth = math.sqrt(sum(c * c for c in rel))
        if depth == 0.0 or depth > range_m:
            continue
        lx, ly, lz = world_to_local(sensor_axes, rel)
        azimuth = math.atan2(ly, lx)
        altitude = math.atan2(lz, math.hypot(lx, ly))
        if abs(azimuth) > math.radians(h_fov_deg) / 2 or abs(altitude) > math.radians(v_fov_deg) / 2:
            continue
        dv = (t.v_north - sensor_vel[0], t.v_east - sensor_vel[1], -sensor_vel[2])
        velocity = sum(dv[i] * rel[i] for i in range(3)) / depth
        detections.append((velocity, azimuth, altitude, depth))
    return detections
