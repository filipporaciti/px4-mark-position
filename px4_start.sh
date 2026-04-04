#!/bin/bash

cleanup() {
    echo -e "\n\n[INFO] Terminate..."
    pkill -9 -f "gz sim"
    pkill -9 -f "px4"
    pkill -9 -f "ruby"
    exit
}

trap cleanup INT TERM EXIT HUP

# --- ENV ---
export PX4_PATH="./PX4-Autopilot"

export PX4_GZ_WORLD=baylands
# ------------------------------

cd $PX4_PATH

# --- START ---
echo "Start PX4 Firmware..."
make px4_sitl gz_x500_mono_cam_down

#HEADLESS=1 ./build/px4_sitl_default/bin/px4
# ------------------------------
