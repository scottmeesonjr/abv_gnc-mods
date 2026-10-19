# Command to the docking mechanism, published on /abv/dock/cmd
# by guidance (CyclicDockGenerator) or abv_commander; forwarded to the Arduino by abv_dock_bridge.

uint8 CMD_ARM=1      # enable actuators, wait for capture
uint8 CMD_DISARM=2   # de-energize actuators
uint8 CMD_LOCK=3     # drive latch to locked
uint8 CMD_UNLOCK=4   # drive latch to unlocked (release)

builtin_interfaces/Time stamp
uint8 command
