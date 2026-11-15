#!/bin/bash
# Runs the firmware state machine on the PC against a simulated latch.
# Usage (from this folder): ./run_tests.sh
set -e
cd "$(dirname "$0")"
FW=../docking_controller
g++ -std=c++17 -Wall -Wextra -I. -I$FW -x c++ $FW/docking_controller.ino $FW/SerialProtocol.cpp \
    $FW/Sensors.cpp $FW/Actuator.cpp $FW/Magnet.cpp test_main.cpp -include algorithm -o /tmp/dock_fw_test
/tmp/dock_fw_test
