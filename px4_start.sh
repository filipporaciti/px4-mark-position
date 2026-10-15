#!/bin/bash

cleanup() {
    echo -e "\n\n[INFO] Terminate..."
    pkill -9 -f "gz sim"
    pkill -9 -f "px4"
    pkill -9 -f "ruby"
    exit
}

trap cleanup INT TERM EXIT HUP

export PX4_PATH="./PX4-Autopilot"

cd $PX4_PATH

echo "Start PX4 Firmware..."

if [ "$1" == "sim_cam_up" ]; then
    export PX4_GZ_WORLD=aruco_roof
    make px4_sitl gz_x500_mono_cam_up
elif [ "$1" == "sim_cam_down" ]; then
    export PX4_GZ_WORLD=aruco_floor
    make px4_sitl gz_x500_mono_cam_down
else
    echo "Usage: $0 {sim_cam_up|sim_cam_down}"
    exit 1
fi
# ------------------------------