# Docking mechanism telemetry, published by abv_dock_bridge on /abv/dock/status.
# Fields from the Arduino status line, plus link_ok and stamp added by the bridge.

# Mechanism state machine (matches firmware enum)
uint8 STATE_IDLE=0
uint8 STATE_ARMED=1
uint8 STATE_CAPTURE=2
uint8 STATE_LOCKED=3
uint8 STATE_RELEASE=4
uint8 STATE_FAULT=5

builtin_interfaces/Time stamp   # host time the status line was received
uint8 state
bool locked                     # true only when hall_1 AND hall_2 both read locked (SYS.05)
bool hall_1
bool hall_2
bool estop                      # true = E-stop pressed, actuators de-energized
uint16 cycle_count              # engagement cycles completed since firmware boot
uint8 magnet_pwm                # 0-255, 0 if no electromagnet fitted
uint8 fault_code                # 0 = no fault
bool link_ok                    # false if no status line within Docking.StatusTimeout
