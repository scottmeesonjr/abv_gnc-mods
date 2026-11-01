"""Writes one CSV row per docking cycle (TEST.09) to ~/abv_dock_logs/.

Works the same in sim and on the real vehicle: it only listens to topics.
  /abv/state           start and contact offsets
  /abv/nav/alert       max speed, alert trips, contact speed
  /abv/dock/status     lock edge, mechanism cycle count
  /abv/dock/cmd        ARM opens a cycle, UNLOCK / DISARM closes it
  /abv/test/condition  free-text note set from abv_commander, copied into each row
  /abv/guidance/command  'dock' runs: commanded radial / angular offset and speed

Parameters: log_dir (default ~/abv_dock_logs), fixture_roll_deg (default 0.0,
the static roll the passive half is mounted at for SYS.09 runs).
"""

import csv
import datetime
import os

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String

from abv_msgs.msg import AbvGuidanceCommand, AbvState, DockCommand, DockStatus, NavAlert

from abv_dock_bridge import docking
from abv_dock_bridge.cycle_recorder import CSV_COLUMNS, CycleRecorder


class DockLogger(Node):

    def __init__(self):
        super().__init__('abv_dock_logger')
        self.declare_parameter('log_dir', os.path.expanduser('~/abv_dock_logs'))
        # dynamic typing so fixture_roll_deg:=4 (int) works as well as 4.0
        self.declare_parameter('fixture_roll_deg', 0.0, ParameterDescriptor(dynamic_typing=True))

        log_dir = os.path.expanduser(self.get_parameter('log_dir').value)
        os.makedirs(log_dir, exist_ok=True)
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        self.path = os.path.join(log_dir, 'dock_log_{}.csv'.format(stamp))
        self.file = open(self.path, 'w', newline='')
        self.writer = csv.DictWriter(self.file, fieldnames=CSV_COLUMNS)
        self.writer.writeheader()
        self.file.flush()

        cfg = docking.load_docking_config()
        self.recorder = CycleRecorder(cfg, self.write_row)
        self.recorder.fixture_roll_deg = float(self.get_parameter('fixture_roll_deg').value)

        self.create_subscription(AbvState, 'abv/state', self.on_state, 10)
        self.create_subscription(NavAlert, 'abv/nav/alert', self.on_alert, 10)
        self.create_subscription(DockStatus, 'abv/dock/status', self.on_status, 10)
        self.create_subscription(DockCommand, 'abv/dock/cmd', self.on_command, 10)
        self.create_subscription(String, 'abv/test/condition', self.on_condition, 10)
        self.create_subscription(AbvGuidanceCommand, 'abv/guidance/command', self.on_guidance_command, 10)

        self.get_logger().info('Logging docking cycles to ' + self.path)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_state(self, msg):
        self.recorder.on_state(self.now(), msg.position.x, msg.position.y, msg.position.yaw, msg.valid)

    def on_alert(self, msg):
        self.recorder.on_alert(self.now(), msg.speed, msg.closing_velocity, msg.alert)

    def on_status(self, msg):
        self.recorder.on_status(self.now(), msg.locked and msg.link_ok, msg.cycle_count)

    def on_command(self, msg):
        self.recorder.on_command(self.now(), msg.command, datetime.datetime.now().isoformat(timespec='seconds'))

    def on_guidance_command(self, msg):
        if msg.type.lower() != 'dock':
            return
        g = msg.goal_state.position
        self.recorder.on_dock_run(g.x, g.yaw, g.y)
        self.get_logger().info('Dock run: radial {:.3f} m, angle {:.1f} deg, speed {:.3f} m/s'.format(
            self.recorder.commanded_radial_m, self.recorder.commanded_angle_deg,
            self.recorder.commanded_speed_m_s))

    def on_condition(self, msg):
        self.recorder.condition = msg.data
        self.get_logger().info('Test condition set: ' + msg.data)

    def write_row(self, row):
        clean = {}
        for k in CSV_COLUMNS:
            v = row.get(k, '')
            clean[k] = round(v, 4) if isinstance(v, float) else v
        self.writer.writerow(clean)
        self.file.flush()
        self.get_logger().info('Cycle {} logged: {}'.format(clean['cycle'], clean['result']))

    def close(self):
        self.recorder.close('NO LOCK (logger stopped)')
        self.file.close()


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
