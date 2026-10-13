"""Pure-Python rotation helpers using Unreal/CARLA conventions.

Frame: x forward, y right, z up, left-handed; Rotator(pitch, yaw, roll) in
degrees, applied as in FRotationMatrix. A matrix is three rows, each row being
the (x, y, z) world-frame components of the local forward, right and up axes
(columns of the local->world transform).
"""

import math


def rotator_to_axes(pitch_deg, yaw_deg, roll_deg):
    """Returns (forward, right, up) world-frame unit vectors."""
    sp, cp = math.sin(math.radians(pitch_deg)), math.cos(math.radians(pitch_deg))
    sy, cy = math.sin(math.radians(yaw_deg)), math.cos(math.radians(yaw_deg))
    sr, cr = math.sin(math.radians(roll_deg)), math.cos(math.radians(roll_deg))
    forward = (cp * cy, cp * sy, sp)
    right = (sr * sp * cy - cr * sy, sr * sp * sy + cr * cy, -sr * cp)
    up = (-(cr * sp * cy + sr * sy), cy * sr - cr * sp * sy, cr * cp)
    return forward, right, up


def local_to_world(axes, v):
    f, r, u = axes
    return tuple(v[0] * f[i] + v[1] * r[i] + v[2] * u[i] for i in range(3))


def world_to_local(axes, v):
    return tuple(sum(a[i] * v[i] for i in range(3)) for a in axes)


def axes_to_rotator(axes):
    """Inverse of rotator_to_axes. Returns (pitch, yaw, roll) in degrees."""
    f, r, u = axes
    yaw = math.atan2(f[1], f[0])
    pitch = math.atan2(f[2], math.hypot(f[0], f[1]))
    sy_axis = (-math.sin(yaw), math.cos(yaw), 0.0)
    roll = math.atan2(sum(u[i] * sy_axis[i] for i in range(3)),
                      sum(r[i] * sy_axis[i] for i in range(3)))
    return math.degrees(pitch), math.degrees(yaw), math.degrees(roll)


def compose(parent_axes, child_axes):
    """Axes of a child frame whose own axes are given in the parent's frame."""
    return tuple(local_to_world(parent_axes, a) for a in child_axes)
