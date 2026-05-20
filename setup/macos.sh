./setup/global.sh

cd PX4-Autopilot

./Tools/setup/macos.sh --sim-tools

echo "Build Firmware"
make px4_sitl

echo "Complete"
