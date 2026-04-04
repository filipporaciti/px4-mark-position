git submodule update --init --recursive --force

cd PX4-Autopilot

git submodule update --init --recursive --force

cp -r ../my_models/* ./Tools/simulation/gz/models

cp -r ../my_worlds/* ./Tools/simulation/gz/worlds
