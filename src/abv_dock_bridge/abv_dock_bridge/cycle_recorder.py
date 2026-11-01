"""Per-cycle docking record (TEST.09), kept free of ROS so it can be unit tested.

A cycle opens on ARM and closes on UNLOCK (or DISARM, or the next ARM).
Inputs are plain numbers; dock_logger_node feeds them from ROS topics.
"""

import math
from collections import deque

from abv_dock_bridge import docking

CSV_COLUMNS = [
    'cycle', 'wall_time', 'condition', 'fixture_roll_deg',
    'commanded_radial_m', 'commanded_angle_deg', 'commanded_speed_m_s',
    'start_gap_m', 'start_lateral_m', 'start_angle_deg',
    'max_speed_m_s', 'alert_tripped',
    'result', 'time_to_capture_s', 'contact_speed_m_s',
    'contact_lateral_m', 'contact_angle_deg', 'mechanism_cycle_count',
]

CMD_ARM, CMD_DISARM, CMD_LOCK, CMD_UNLOCK = 1, 2, 3, 4

# s, window before the lock edge used to find the contact speed: the lock
# report can arrive a sample or two after the faces stop
CONTACT_WINDOW_S = 0.5


class CycleRecorder:

    def __init__(self, cfg, write_row):
        self.cfg = cfg
        self.write_row = write_row      # callable(dict)
        self.cycle_number = 0
        self.condition = ''
        self.fixture_roll_deg = 0.0
        # test settings of the current dock run (from the guidance command)
        self.commanded_radial_m = 0.0
        self.commanded_angle_deg = 0.0
        self.commanded_speed_m_s = cfg['ApproachVelocity']
        self.pose = None                # (x, y, yaw)
        self.locked = False
        self.mech_cycles = 0
        self.recent = deque()           # (t, closing_velocity) samples
        self.open = None                # dict for the cycle in progress

    # ------------------------------------------------------------- inputs
    def on_state(self, t, x, y, yaw, valid):
        if valid:
            self.pose = (x, y, yaw)

    def on_alert(self, t, speed, closing, alert):
        self.recent.append((t, closing))
        while self.recent and t - self.recent[0][0] > CONTACT_WINDOW_S:
            self.recent.popleft()
        if self.open is not None and self.open['result'] == '':
            self.open['max_speed_m_s'] = max(self.open['max_speed_m_s'], speed)
            if alert:
                self.open['alert_tripped'] = True

    def on_status(self, t, locked, cycle_count):
        self.mech_cycles = cycle_count
        if locked and not self.locked and self.open is not None and self.open['result'] == '':
            self.open['result'] = 'CAPTURED'
            self.open['time_to_capture_s'] = t - self.open['_t_arm']
            self.open['contact_speed_m_s'] = max((c for _, c in self.recent), default=float('nan'))
            if self.pose is not None:
                x, y, yaw = self.pose
                self.open['contact_lateral_m'] = docking.lateral_offset(self.cfg, x, y, yaw)
                self.open['contact_angle_deg'] = math.degrees(docking.angular_offset(self.cfg, yaw))
        self.locked = locked

    def on_dock_run(self, radial_m, angle_rad, speed_m_s):
        """A new 'dock' guidance command: goal x, yaw, y (speed 0 = config)."""
        self.commanded_radial_m = radial_m
        self.commanded_angle_deg = math.degrees(angle_rad)
        self.commanded_speed_m_s = speed_m_s if speed_m_s > 0.0 else self.cfg['ApproachVelocity']

    def on_command(self, t, code, wall_time):
        if code == CMD_ARM:
            self.close('NO LOCK')
            self.start(t, wall_time)
        elif code in (CMD_UNLOCK, CMD_DISARM):
            self.close('NO LOCK')

    # ------------------------------------------------------------ helpers
    def start(self, t, wall_time):
        self.cycle_number += 1
        row = {k: '' for k in CSV_COLUMNS}
        row.update({
            'cycle': self.cycle_number,
            'wall_time': wall_time,
            'condition': self.condition,
            'fixture_roll_deg': self.fixture_roll_deg,
            'commanded_radial_m': self.commanded_radial_m,
            'commanded_angle_deg': self.commanded_angle_deg,
            'commanded_speed_m_s': self.commanded_speed_m_s,
            'max_speed_m_s': 0.0,
            'alert_tripped': False,
            '_t_arm': t,
        })
        if self.pose is not None:
            x, y, yaw = self.pose
            row['start_gap_m'] = docking.face_gap(self.cfg, x, y, yaw)
            row['start_lateral_m'] = docking.lateral_offset(self.cfg, x, y, yaw)
            row['start_angle_deg'] = math.degrees(docking.angular_offset(self.cfg, yaw))
        self.open = row

    def close(self, result_if_none):
        if self.open is None:
            return
        row = self.open
        if row['result'] == '':
            row['result'] = result_if_none
        row['mechanism_cycle_count'] = self.mech_cycles
        row.pop('_t_arm', None)
        self.write_row(row)
        self.open = None
