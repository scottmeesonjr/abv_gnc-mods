import math

from abv_dock_bridge import docking, protocol
from abv_dock_bridge.cycle_recorder import CycleRecorder

# sim placeholder target: face at x = 3.4, facing -x
CFG = dict(docking.DEFAULTS, TargetPose=[3.4, 0.9, math.pi], PortOffset=0.30)


def test_geometry_matches_cpp():
    # standoff pose from DockingGeometry::chaserPoseAtGap(cfg, 0.5) = (2.6, 0.9, 0)
    assert abs(docking.face_gap(CFG, 2.6, 0.9, 0.0) - 0.5) < 1e-9
    assert abs(docking.face_gap(CFG, 3.1, 0.9, 0.0)) < 1e-9
    assert abs(docking.closing_velocity(CFG, 0.075, 0.0) - 0.075) < 1e-9
    assert abs(docking.lateral_offset(CFG, 2.6, 0.8, 0.0) - 0.1) < 1e-9
    assert abs(docking.angular_offset(CFG, 0.07) - 0.07) < 1e-9


def test_protocol_roundtrip():
    assert protocol.format_velocity_line(0.075, 0.5) == 'V,0.0750,0.5000\n'
    assert protocol.format_command_line(4) == 'C,UNLOCK\n'
    s = protocol.parse_status_line('S,3,1,1,1,0,7,0,0')
    assert s['locked'] and s['state'] == 3 and s['cycle_count'] == 7
    assert protocol.parse_status_line('S,3,1') is None
    assert protocol.parse_status_line('garbage') is None


def test_recorder_captured_and_missed_cycles():
    rows = []
    r = CycleRecorder(CFG, rows.append)
    r.on_state(0.0, 2.6, 0.9, 0.0, True)
    r.on_command(0.0, 1, 'w')                     # ARM
    r.on_alert(9.0, 0.074, 0.074, False)
    r.on_state(9.1, 3.1, 0.9, 0.0, True)
    r.on_status(9.2, True, 0)                    # lock edge
    r.on_command(14.0, 4, 'w')                   # UNLOCK
    r.on_status(14.1, False, 1)
    r.on_command(20.0, 1, 'w')                   # ARM, never locks
    r.on_command(50.0, 4, 'w')                   # UNLOCK
    assert [row['result'] for row in rows] == ['CAPTURED', 'NO LOCK']
    assert abs(rows[0]['contact_speed_m_s'] - 0.074) < 1e-9
    assert abs(rows[0]['start_gap_m'] - 0.5) < 1e-9
    assert rows[0]['alert_tripped'] is False


def test_recorder_uses_commanded_settings():
    rows = []
    r = CycleRecorder(CFG, rows.append)
    r.on_dock_run(0.1, math.radians(4.0), 0.05)
    r.on_state(0.0, 2.6, 0.8, 0.0, True)
    r.on_command(0.0, 1, 'w')
    r.on_command(30.0, 4, 'w')
    assert rows[0]['commanded_radial_m'] == 0.1
    assert abs(rows[0]['commanded_angle_deg'] - 4.0) < 1e-9
    assert rows[0]['commanded_speed_m_s'] == 0.05
    r.on_dock_run(0.0, 0.0, 0.0)                 # 0 = config speed
    assert r.commanded_speed_m_s == CFG['ApproachVelocity']
