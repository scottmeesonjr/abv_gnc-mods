"""Docking test logs, written to ~/abv_dock_logs/ (sim and real vehicle alike).

Two files per dock run, both named dock_<kind>_<YYYYmmdd_HHMMSS>[_<run name>].csv:

  dock_log_...        one row per cycle (TEST.09): offsets, contact speed, result.
                      See cycle_recorder.CSV_COLUMNS.
  dock_telemetry_...  one row per navigation update (25 Hz) from the dock command
                      until the run completes or is aborted: pose, velocity,
                      thruster command, closing speed, gap, alert, mechanism
                      state and the current cycle / step. For time-history plots
                      (analysis/plot_dock_run.m).

The run name and note come from /abv/test/run_name and /abv/test/condition,
which the GUI or commander publish just before the dock command; the files
open 0.5 s after the command so both have arrived. Cycles armed by hand
outside a dock run go to a dock_log_..._manual.csv file.

Parameters: log_dir (default ~/abv_dock_logs), fixture_roll_deg (default 0.0).
"""

import csv
import datetime
import math
import os
import re

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String

from abv_msgs.msg import (AbvGuidanceCommand, AbvState, AbvThrusterStatus,
                          DockCommand, DockStatus, NavAlert)

from abv_dock_bridge import docking
from abv_dock_bridge.cycle_recorder import CSV_COLUMNS, CycleRecorder

TELEMETRY_COLUMNS = [
    't_s', 'x_m', 'y_m', 'yaw_deg', 'vx_m_s', 'vy_m_s', 'wz_deg_s', 'nav_valid',
    'speed_m_s', 'closing_m_s', 'gap_m', 'lateral_m', 'alert',
    'fx_cmd', 'fy_cmd', 'tz_cmd',
    'mech_state', 'locked', 'cycle', 'step',
]

OPEN_DELAY_S = 0.5


def clean_name(name):
    name = re.sub(r'[^A-Za-z0-9_-]+', '_', name.strip()).strip('_')
    return name[:60]


class DockLogger(Node):

    def __init__(self):
        super().__init__('abv_dock_logger')
        self.declare_parameter('log_dir', os.path.expanduser('~/abv_dock_logs'))
        # dynamic typing so fixture_roll_deg:=4 (int) works as well as 4.0
        self.declare_parameter('fixture_roll_deg', 0.0, ParameterDescriptor(dynamic_typing=True))

        self.log_dir = os.path.expanduser(self.get_parameter('log_dir').value)
        os.makedirs(self.log_dir, exist_ok=True)

        self.cfg = docking.load_docking_config()
        self.recorder = CycleRecorder(self.cfg, self.write_row)
        self.recorder.fixture_roll_deg = float(self.get_parameter('fixture_roll_deg').value)

        self.cycle_file = None
        self.cycle_writer = None
        self.tele_file = None
        self.tele_writer = None
        self.run_t0 = None
        self.pending_name = ''
        self.open_timer = None

        # latest values for the telemetry row
        self.thrust = (0.0, 0.0, 0.0)
        self.alert = False
        self.mech_state = 0
        self.locked = False
        self.cycle = 0
        self.step = ''

        self.create_subscription(AbvState, 'abv/state', self.on_state, 10)
        self.create_subscription(NavAlert, 'abv/nav/alert', self.on_alert, 10)
        self.create_subscription(DockStatus, 'abv/dock/status', self.on_status, 10)
        self.create_subscription(DockCommand, 'abv/dock/cmd', self.on_command, 10)
        self.create_subscription(String, 'abv/test/condition', self.on_condition, 10)
        self.create_subscription(String, 'abv/test/run_name', self.on_run_name, 10)
        self.create_subscription(String, 'abv/dock/event', self.on_event, 10)
        self.create_subscription(AbvGuidanceCommand, 'abv/guidance/command', self.on_guidance_command, 10)
        self.create_subscription(AbvThrusterStatus, 'abv/thrusters/status', self.on_thrust, 10)

        self.get_logger().info('Docking logs go to ' + self.log_dir + ' (two files per run)')

    # ------------------------------------------------------------ files
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def open_run_files(self, name, telemetry=True):
        self.close_files()
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        suffix = '_' + name if name else ''

        path = os.path.join(self.log_dir, 'dock_log_{}{}.csv'.format(stamp, suffix))
        self.cycle_file = open(path, 'w', newline='')
        self.cycle_writer = csv.DictWriter(self.cycle_file, fieldnames=CSV_COLUMNS)
        self.cycle_writer.writeheader()
        self.cycle_file.flush()
        self.get_logger().info('Cycle log: ' + path)

        if telemetry:
            tpath = os.path.join(self.log_dir, 'dock_telemetry_{}{}.csv'.format(stamp, suffix))
            self.tele_file = open(tpath, 'w', newline='')
            self.tele_writer = csv.writer(self.tele_file)
            self.tele_writer.writerow(TELEMETRY_COLUMNS)
            self.run_t0 = self.now()
            self.get_logger().info('Telemetry log: ' + tpath)

        self.recorder.cycle_number = 0
        self.recorder.run_name = name

    def close_telemetry(self):
        if self.tele_file is not None:
            self.tele_file.close()
        self.tele_file = None
        self.tele_writer = None
        self.run_t0 = None

    def close_files(self):
        self.close_telemetry()
        if self.cycle_file is not None:
            self.cycle_file.close()
        self.cycle_file = None
        self.cycle_writer = None

    def open_pending_run(self):
        if self.open_timer is not None:
            self.open_timer.cancel()
            self.destroy_timer(self.open_timer)
            self.open_timer = None
        self.open_run_files(self.pending_name)
        self.pending_name = ''

    # ------------------------------------------------------------ inputs
    def on_guidance_command(self, msg):
        if msg.type.lower() != 'dock':
            return
        g = msg.goal_state.position
        self.recorder.close('NO LOCK')   # a new run ends any open cycle
        self.recorder.on_dock_run(g.x, g.yaw, g.y)
        self.get_logger().info('Dock run: radial {:.3f} m, angle {:.1f} deg, speed {:.3f} m/s'.format(
            self.recorder.commanded_radial_m, self.recorder.commanded_angle_deg,
            self.recorder.commanded_speed_m_s))
        if self.open_timer is not None:
            self.open_timer.cancel()
            self.destroy_timer(self.open_timer)
        self.open_timer = self.create_timer(OPEN_DELAY_S, self.open_pending_run)

    def on_run_name(self, msg):
        self.pending_name = clean_name(msg.data)

    def on_condition(self, msg):
        self.recorder.condition = msg.data
        if msg.data:
            self.get_logger().info('Test condition set: ' + msg.data)

    def on_event(self, msg):
        parts = msg.data.split('|')
        if parts[0] == 'STEP' and len(parts) >= 5:
            self.cycle = int(parts[1])
            self.step = parts[4]
        elif parts[0] in ('COMPLETE', 'ABORTED'):
            self.step = parts[0]
            self.write_telemetry_row(None)   # last row shows how the run ended
            self.close_telemetry()

    def on_thrust(self, msg):
        self.thrust = (msg.fx, msg.fy, msg.tz)

    def on_alert(self, msg):
        self.alert = msg.alert
        self.recorder.on_alert(self.now(), msg.speed, msg.closing_velocity, msg.alert)

    def on_status(self, msg):
        self.mech_state = msg.state
        self.locked = msg.locked and msg.link_ok
        self.recorder.on_status(self.now(), self.locked, msg.cycle_count)

    def on_command(self, msg):
        if msg.command == DockCommand.CMD_ARM:
            if self.open_timer is not None:
                self.open_pending_run()        # ARM beat the open delay
            elif self.cycle_writer is None:
                self.recorder.close('NO LOCK')
                self.open_run_files('manual', telemetry=False)
        self.recorder.on_command(self.now(), msg.command,
                                 datetime.datetime.now().isoformat(timespec='seconds'))

    def on_state(self, msg):
        self.recorder.on_state(self.now(), msg.position.x, msg.position.y, msg.position.yaw, msg.valid)
        self.write_telemetry_row(msg)

    # ------------------------------------------------------------ writers
    def write_telemetry_row(self, msg):
        if self.tele_writer is None:
            return
        if msg is None:
            row = [round(self.now() - self.run_t0, 3)] + [''] * 7 + [''] * 5 + \
                  list(self.thrust) + [self.mech_state, int(self.locked), self.cycle, self.step]
        else:
            p, v = msg.position, msg.velocity
            row = [
                round(self.now() - self.run_t0, 3),
                round(p.x, 5), round(p.y, 5), round(math.degrees(p.yaw), 4),
                round(v.x, 5), round(v.y, 5), round(math.degrees(v.yaw), 4), int(msg.valid),
                round(math.hypot(v.x, v.y), 5),
                round(docking.closing_velocity(self.cfg, v.x, v.y), 5),
                round(docking.face_gap(self.cfg, p.x, p.y, p.yaw), 5),
                round(docking.lateral_offset(self.cfg, p.x, p.y, p.yaw), 5),
                int(self.alert),
                round(self.thrust[0], 4), round(self.thrust[1], 4), round(self.thrust[2], 4),
                self.mech_state, int(self.locked), self.cycle, self.step,
            ]
        self.tele_writer.writerow(row)

    def write_row(self, row):
        if self.cycle_writer is None:
            self.open_run_files('manual', telemetry=False)
        clean = {}
        for k in CSV_COLUMNS:
            v = row.get(k, '')
            clean[k] = round(v, 4) if isinstance(v, float) else v
        self.cycle_writer.writerow(clean)
        self.cycle_file.flush()
        if self.tele_file is not None:
            self.tele_file.flush()
        self.get_logger().info('Cycle {} logged: {}'.format(clean['cycle'], clean['result']))

    def close(self):
        self.recorder.close('NO LOCK (logger stopped)')
        self.close_files()


def main(args=None):
    rclpy.init(args=args)
    node = DockLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
