#!/bin/bash

cleanup() {
    echo -e "\n\n[INFO] Terminate..."
    pkill -9 -f "gz sim"
    pkill -9 -f "px4"
    pkill -9 -f "ruby"
    pkill -9 -f "gst-launch-1.0"
    exit
}

trap cleanup INT TERM EXIT HUP

# --- ENV ---
export PX4_PATH="./PX4-Autopilot"

export PX4_GZ_WORLD=aruco_floor
# ------------------------------

cd $PX4_PATH

# --- START ---
echo "Start camera link server on port 5001"
gst-launch-1.0 -v udpsrc port=5600 ! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! mpegtsmux ! udpsink host=127.0.0.1 port=5001 > /dev/null 2>&1 &

echo "Start PX4 Firmware..."
make px4_sitl gz_x500_mono_cam_down

#HEADLESS=1 ./build/px4_sitl_default/bin/px4
# ------------------------------
