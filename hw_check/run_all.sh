#!/usr/bin/env bash
# Runs every check in order, bottom layer first.
# motor_test.py is NOT in here on purpose, it spins props, run it yourself.
#
#   ./hw_check/run_all.sh            (with MAVROS off)

cd "$(dirname "$0")" || exit 1

python3 jetson/check_system.py
python3 jetson/check_zed.py

python3 link/check_link.py
if [ $? -eq 1 ]; then
    echo -e "\n\033[31mlink is down, every pi check would fail too. fix the link first.\033[0m"
    exit 1
fi
python3 link/check_dvl.py

python3 pi/check_fcu.py
python3 pi/check_outputs.py

echo -e "\nnext: start MAVROS, then run  python3 hw_check/link/check_mavros.py"
echo "      and  ros2 run monitor basic_monitor  for the topic rates"
