"""Serial line protocol between abv_dock_bridge and the docking_controller Arduino.

Every line is ASCII, comma separated, ending in '\\n'. Keep this file and
docking_controller/SerialProtocol.cpp in step.

Host -> Arduino
  V,<closing_m_s>,<gap_m>        25 Hz. Closing velocity (+ toward target) and face gap.
                                 Also the link heartbeat: if none arrives for
                                 200 ms the Arduino goes to its safe state.
  C,ARM | C,DISARM | C,LOCK | C,UNLOCK

Arduino -> Host
  S,<state>,<locked>,<hall1>,<hall2>,<estop>,<cycles>,<pwm>,<fault>   ~20 Hz
      state: 0 IDLE, 1 ARMED, 2 CAPTURE, 3 LOCKED, 4 RELEASE, 5 FAULT
      locked/hall1/hall2/estop: 0 or 1
  L,<free text>                  log line, printed by the bridge
"""

COMMAND_NAMES = {1: 'ARM', 2: 'DISARM', 3: 'LOCK', 4: 'UNLOCK'}


def format_velocity_line(closing, gap):
    return 'V,{:.4f},{:.4f}\n'.format(closing, gap)


def format_command_line(code):
    name = COMMAND_NAMES.get(code)
    if name is None:
        raise ValueError('unknown DockCommand code {}'.format(code))
    return 'C,{}\n'.format(name)


def parse_status_line(line):
    """Return a dict for an 'S,...' line, or None if it isn't a valid status line."""
    fields = line.strip().split(',')
    if len(fields) != 9 or fields[0] != 'S':
        return None
    try:
        v = [int(f) for f in fields[1:]]
    except ValueError:
        return None
    return {
        'state': v[0],
        'locked': bool(v[1]),
        'hall_1': bool(v[2]),
        'hall_2': bool(v[3]),
        'estop': bool(v[4]),
        'cycle_count': v[5],
        'magnet_pwm': v[6],
        'fault_code': v[7],
    }
