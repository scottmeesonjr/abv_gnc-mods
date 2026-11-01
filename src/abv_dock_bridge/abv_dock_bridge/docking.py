"""Docking config loading and geometry.

The geometry functions mirror DockingGeometry in abv_common/DockingConfig.h
exactly, so the bridge, the logger, navigation and guidance all agree on what
"gap", "closing velocity" and "offset" mean.

Conventions (table frame, OptiTrack):
  psi = TargetPose[2], the passive face's outward normal direction
  n = (cos psi, sin psi), t = (-sin psi, cos psi)
  The ABV's active face is PortOffset ahead of its center along its +x axis,
  so a perfectly aligned ABV points at yaw psi + pi.
"""

import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory


DEFAULTS = {
    'TargetPose': [0.0, 0.0, 0.0],
    'PortOffset': 0.30,
    'StandoffDistance': 0.50,
    'ApproachVelocity': 0.075,
    'ApproachTimeout': 30.0,
    'CaptureRange': 0.01,
    'DwellTime': 5.0,
    'Cycles': 10,
    'SerialPort': '/dev/ttyACM0',
    'BaudRate': 115200,
    'StatusTimeout': 0.5,
    'SimulateContact': False,
}


def load_docking_config(path=None):
    """Read the Docking block of abv_bringup's installed config.yaml."""
    if path is None:
        path = os.path.join(get_package_share_directory('abv_bringup'), 'config', 'config.yaml')
    with open(path, 'r') as f:
        root = yaml.safe_load(f) or {}
    cfg = dict(DEFAULTS)
    cfg.update(root.get('Docking') or {})
    if len(cfg['TargetPose']) != 3:
        raise ValueError('Docking.TargetPose must have 3 values')
    return cfg


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def face_point(cfg, x, y, yaw):
    return (x + cfg['PortOffset'] * math.cos(yaw),
            y + cfg['PortOffset'] * math.sin(yaw))


def face_gap(cfg, x, y, yaw):
    """m, gap between the ABV's active face and the target face along n."""
    tx, ty, psi = cfg['TargetPose']
    fx, fy = face_point(cfg, x, y, yaw)
    return (fx - tx) * math.cos(psi) + (fy - ty) * math.sin(psi)


def lateral_offset(cfg, x, y, yaw):
    """m, sideways offset of the ABV's face center from the target center along t."""
    tx, ty, psi = cfg['TargetPose']
    fx, fy = face_point(cfg, x, y, yaw)
    return (fx - tx) * -math.sin(psi) + (fy - ty) * math.cos(psi)


def angular_offset(cfg, yaw):
    """rad, ABV heading error relative to perfectly aligned (psi + pi)."""
    return wrap_angle(yaw - (cfg['TargetPose'][2] + math.pi))


def closing_velocity(cfg, vx, vy):
    """m/s, velocity along the docking axis, + = toward the target."""
    psi = cfg['TargetPose'][2]
    return -(vx * math.cos(psi) + vy * math.sin(psi))
