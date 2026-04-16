git submodule update --init --recursive --force

cd PX4-Autopilot

git submodule update --init --recursive --force

cp -r ../my_models/* ./Tools/simulation/gz/models

cp -r ../my_worlds/* ./Tools/simulation/gz/worlds

cp ../my_models/x500_mono_cam_up/4015_gz_x500_mono_cam_up ./ROMFS/px4fmu_common/init.d-posix/airframes
sed -i '' '84a\'$'\n''4015_gz_x500_mono_cam_up' ./ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt