"""Bridge between ROS 2 and the docking mechanism Arduino (real vehicle only).

  /abv/state       -> V lines (closing velocity, face gap) at the nav rate, capped at 25 Hz
  /abv/dock/cmd    -> C lines (ARM / DISARM / LOCK / UNLOCK)
  S lines          -> /abv/dock/status (DockStatus), with link_ok

In simulation abv_simulator publishes /abv/dock/status itself
(Docking.SimulateContact), so this node is only launched with sim:=false.
"""

import time

import rclpy
from rclpy.node import Node

from abv_msgs.msg import AbvState, DockCommand, DockStatus

from abv_dock_bridge import docking
from abv_dock_bridge import protocol

try:
    import serial
except ImportError:  # reported clearly at startup instead of a stack trace
    serial = None


class DockBridge(Node):

    def __init__(self):
        super().__init__('abv_dock_bridge')
        self.cfg = docking.load_docking_config()

        self.declare_parameter('serial_port', self.cfg['SerialPort'])
        self.declare_parameter('baud_rate', int(self.cfg['BaudRate']))
        self.port = self.get_parameter('serial_port').value
        self.baud = int(self.get_parameter('baud_rate').value)
        self.status_timeout = float(self.cfg['StatusTimeout'])

        self.ser = None
        self.rx_buffer = ''
        self.last_status = None
        self.last_status_time = 0.0
        self.last_v_sent = 0.0
        self.last_open_attempt = 0.0
        self.link_was_ok = False

        self.status_pub = self.create_publisher(DockStatus, 'abv/dock/status', 10)
        self.create_subscription(AbvState, 'abv/state', self.state_callback, 10)
        self.create_subscription(DockCommand, 'abv/dock/cmd', self.command_callback, 10)

        self.create_timer(0.01, self.read_serial)        # 100 Hz serial read
        self.create_timer(0.1, self.publish_link_status)  # link watchdog

        if serial is None:
            self.get_logger().error('pyserial is not installed: sudo apt install python3-serial')
        self.open_serial()

    # ---------------------------------------------------------------- serial
    def open_serial(self):
        self.last_open_attempt = time.monotonic()
        if serial is None:
            return
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0)
            self.rx_buffer = ''
            self.get_logger().info('Opened {} at {} baud'.format(self.port, self.baud))
        except (serial.SerialException, OSError) as e:
            self.ser = None
            self.get_logger().warn('Could not open {}: {} (retrying every 2 s)'.format(self.port, e),
                                   throttle_duration_sec=10.0)

    def close_serial(self, reason):
        self.get_logger().error('Serial link lost: {}'.format(reason))
        try:
            if self.ser is not None:
                self.ser.close()
        except Exception:
            pass
        self.ser = None

    def write_line(self, line):
        if self.ser is None:
            return False
        try:
            self.ser.write(line.encode('ascii'))
            return True
        except (serial.SerialException, OSError) as e:
            self.close_serial(e)
            return False

    def read_serial(self):
        if self.ser is None:
            if time.monotonic() - self.last_open_attempt > 2.0:
                self.open_serial()
            return
        try:
            data = self.ser.read(self.ser.in_waiting or 1)
        except (serial.SerialException, OSError) as e:
            self.close_serial(e)
            return
        if not data:
            return
        self.rx_buffer += data.decode('ascii', errors='replace')
        while '\n' in self.rx_buffer:
            line, self.rx_buffer = self.rx_buffer.split('\n', 1)
            self.handle_line(line.strip())
        if len(self.rx_buffer) > 256:  # garbage with no newline
            self.rx_buffer = ''

    def handle_line(self, line):
        if not line:
            return
        if line.startswith('L,'):
            self.get_logger().info('[arduino] ' + line[2:])
            return
        status = protocol.parse_status_line(line)
        if status is None:
            self.get_logger().warn('Ignoring unexpected line from Arduino: ' + line,
                                   throttle_duration_sec=5.0)
            return
        self.last_status = status
        self.last_status_time = time.monotonic()
        self.publish_status(link_ok=True)

    # --------------------------------------------------------------- ROS side
    def state_callback(self, msg):
        now = time.monotonic()
        if now - self.last_v_sent < 0.038:  # cap at ~25 Hz
            return
        self.last_v_sent = now
        if not msg.valid:
            # no trustworthy nav: send no V line, so the Arduino's 200 ms
            # watchdog puts the mechanism in its safe state
            return
        p, v = msg.position, msg.velocity
        closing = docking.closing_velocity(self.cfg, v.x, v.y)
        gap = docking.face_gap(self.cfg, p.x, p.y, p.yaw)
        self.write_line(protocol.format_velocity_line(closing, gap))

    def command_callback(self, msg):
        try:
            line = protocol.format_command_line(msg.command)
        except ValueError as e:
            self.get_logger().warn(str(e))
            return
        if self.write_line(line):
            self.get_logger().info('Sent ' + line.strip())
        else:
            self.get_logger().error('Could not send {}: serial link down'.format(line.strip()))

    def publish_status(self, link_ok):
        msg = DockStatus()
        msg.stamp = self.get_clock().now().to_msg()
        s = self.last_status
        if s is not None:
            msg.state = s['state']
            msg.locked = s['locked']
            msg.hall_1 = s['hall_1']
            msg.hall_2 = s['hall_2']
            msg.estop = s['estop']
            msg.cycle_count = s['cycle_count']
            msg.magnet_pwm = s['magnet_pwm']
            msg.fault_code = s['fault_code']
        msg.link_ok = link_ok
        if not link_ok:
            # never report a stale lock as real
            msg.locked = False
        self.status_pub.publish(msg)

    def publish_link_status(self):
        link_ok = (self.last_status is not None and
                   time.monotonic() - self.last_status_time <= self.status_timeout)
        if link_ok != self.link_was_ok:
            if link_ok:
                self.get_logger().info('Mechanism link OK')
            else:
                self.get_logger().error('Mechanism link DOWN (no status for {} s)'.format(self.status_timeout))
            self.link_was_ok = link_ok
        if not link_ok:
            self.publish_status(link_ok=False)


def main(args=None):
    rclpy.init(args=args)
    node = DockBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser is not None:
            node.ser.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
