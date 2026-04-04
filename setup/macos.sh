git submodule update --init --recursive --force

cd PX4-Autopilot

git submodule update --init --recursive --force

./Tools/setup/macos.sh --sim-tools

brew install gnu-sed

# BUILD SETUP
export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:$(brew --prefix qt@5)
export LIBRARY_PATH=$LIBRARY_PATH:$(brew --prefix)/lib

gsed -i '83c rotor_velocity_message.set_velocity(i, static_cast<double>(outputs[i]));' ./src/modules/simulation/gz_bridge/GZMixingInterfaceESC.cpp


echo "Build Firmware"
make px4_sitl

echo "Complete"
